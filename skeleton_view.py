"""Live OpenSim mesh view for the GUI, plus the URDF renderer used by comparison videos.

SkeletonView poses OpenSim .vtp geometry from /joint_states. skeleton_for_model()
still returns the RViz URDF (render_compare / render_motion).
"""

import json
import subprocess
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import tkinter as tk
import vtk
from PIL import Image, ImageTk
from vtk.util.numpy_support import vtk_to_numpy

ROOT = Path(__file__).resolve().parent

# Same links human_model.launch.py drops when upper_body_only is set.
_LEG_MARKERS = (
    "Leg", "Foot", "Toe", "FTSensor",
    "RecFem", "BicFem", "TibAnt", "GasLat", "GasMed",
)


def _hide_link(name):
    if any(marker in name for marker in _LEG_MARKERS):
        return True
    if "Hand" in name:
        return True
    return name.endswith("_RH") or name.endswith("_LH")


def _upper_body_urdf(xml_text):
    root = ET.fromstring(xml_text)
    drop = {
        link.get("name")
        for link in root.findall("link")
        if link.get("name") and _hide_link(link.get("name"))
    }
    for link in list(root.findall("link")):
        if link.get("name") in drop:
            root.remove(link)
    for joint in list(root.findall("joint")):
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.get("link") if parent is not None else ""
        child_name = child.get("link") if child is not None else ""
        if parent_name in drop or child_name in drop:
            root.remove(joint)
    return root


def _xyz(text):
    if not text:
        return np.zeros(3)
    return np.array([float(part) for part in text.split()], dtype=float)


def _origin(elem):
    if elem is None:
        return np.zeros(3), np.zeros(3)
    return _xyz(elem.get("xyz")), _xyz(elem.get("rpy"))


def _rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis rpy: yaw about Z, then pitch about Y, then roll about X."""
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _transform(xyz, rpy):
    pose = np.eye(4)
    pose[:3, :3] = _rpy_matrix(*rpy)
    pose[:3, 3] = xyz
    return pose


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    length = np.linalg.norm(axis)
    if length < 1e-9 or abs(angle) < 1e-12:
        return np.eye(4)
    x, y, z = axis / length
    c, s = np.cos(angle), np.sin(angle)
    v = 1 - c
    pose = np.eye(4)
    pose[:3, :3] = np.array([
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ])
    return pose


def _vtk_matrix(pose):
    matrix = vtk.vtkMatrix4x4()
    for row in range(4):
        for col in range(4):
            matrix.SetElement(row, col, float(pose[row, col]))
    return matrix


def _find_urdf(name="human.urdf"):
    candidates = [
        ROOT / "src" / "argus_server" / "urdf" / name,
        ROOT / "install" / "argus_server" / "share" / "argus_server" / "urdf" / name,
    ]
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(f"{name} not found")


def _mesh_file(filename):
    name = filename.rsplit("/", 1)[-1]
    for folder in (
        ROOT / "src" / "argus_server" / "meshes",
        ROOT / "install" / "argus_server" / "share" / "argus_server" / "meshes",
    ):
        path = folder / name
        if path.is_file():
            return path
    return None


def _model_folder(model="rajagopal_upper"):
    folder = Path(model).expanduser()
    if not folder.is_dir():
        folder = ROOT / "models" / str(model)
    return folder


def _model_config(model="bimanual_upper_arm"):
    folder = _model_folder(model)
    return folder, json.loads((folder / "argus_model.json").read_text())


def skeleton_for_model(model="bimanual_upper_arm"):
    """SkeletonModel with the URDF and leg visibility from the model's argus_model.json."""
    _folder, config = _model_config(model)
    skeleton = config.get("analysis", {}).get("skeleton", {})
    return SkeletonModel(config.get("urdf", "human.urdf"), upper_body=skeleton.get("hide_legs", True),
                         visible_links=skeleton.get("visible_links"))


# Extra OpenSim bodies drawn with a tracked segment (forearm is ulna + radius).
_COMPANION_BODIES = {
    "radius_r": ("ulna_r",),
    "radius_l": ("ulna_l",),
}


def _visible_bodies(skeleton):
    if skeleton.get("visible_bodies"):
        return set(skeleton["visible_bodies"])
    bodies = set(skeleton.get("links") or {})
    for name in list(bodies):
        bodies.update(_COMPANION_BODIES.get(name, ()))
    return bodies


def _find_pose_bones():
    for candidate in (
        ROOT / "install" / "argus_server" / "lib" / "argus_server" / "pose_bones",
        ROOT / "build" / "argus_server" / "pose_bones",
    ):
        if candidate.is_file():
            return candidate
    return None


def _wxyz_pose(quat, position):
    w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    n = w * w + x * x + y * y + z * z
    s = 0.0 if n < 1e-18 else 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    pose = np.eye(4)
    pose[:3, :3] = (
        (1.0 - (yy + zz), xy - wz, xz + wy),
        (xy + wz, 1.0 - (xx + zz), yz - wx),
        (xz - wy, yz + wx, 1.0 - (xx + yy)),
    )
    pose[:3, 3] = position
    return pose


def _body_meshes(osim_path):
    """(body name, mesh file, scale xyz, rgb) for each Mesh on a Body."""
    root = ET.parse(osim_path).getroot()
    meshes = []
    for body in root.iter("Body"):
        name = body.get("name")
        if not name:
            continue
        for mesh in body.iter("Mesh"):
            file_elem = mesh.find("mesh_file")
            if file_elem is None or not (file_elem.text or "").strip():
                continue
            scale_elem = mesh.find("scale_factors")
            scale = _xyz((scale_elem.text if scale_elem is not None else None) or "1 1 1")
            color = (0.86, 0.75, 0.68)
            color_elem = mesh.find("Appearance/color")
            if color_elem is not None and (color_elem.text or "").strip():
                parts = [float(p) for p in color_elem.text.split()]
                if len(parts) >= 3 and max(parts[:3]) < 0.999:
                    color = tuple(parts[:3])
            meshes.append((name, file_elem.text.strip(), scale, color))
    return meshes


def _vtp_file(folder, filename):
    name = Path(filename).name
    for path in (folder / "Geometry" / name, folder / name):
        if path.is_file():
            return path
    return None


def opensim_skeleton_for_model(model="bimanual_upper_arm", *, all_bodies=False):
    folder, config = _model_config(model)
    osim_name = config.get("model", "model.osim")
    osim_path = folder / osim_name
    if not osim_path.is_file():
        osim_path = folder / "model.osim"
    skeleton = config.get("analysis", {}).get("skeleton", {})
    visible = None if all_bodies else _visible_bodies(skeleton)
    return OpenSimSkeletonModel(folder, osim_path, visible)


class OpenSimSkeletonModel:
    """OpenSim body meshes posed by OpenSim FK (pose_bones --server)."""

    def __init__(self, folder, osim_path, visible_bodies):
        self.folder = Path(folder)
        self.osim_path = Path(osim_path)
        self.visible_bodies = set(visible_bodies) if visible_bodies else None
        self.angles = {}
        self.actors = []
        self._mesh_cache = {}
        self._proc = None
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.067, 0.075, 0.094)
        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.AddRenderer(self.renderer)
        self.window.SetSize(420, 560)
        self.capture = vtk.vtkWindowToImageFilter()
        self.capture.SetInput(self.window)
        self._load_visuals()
        self._start_fk()
        self.apply({})
        self._frame_camera()

    def _start_fk(self):
        binary = _find_pose_bones()
        if binary is None:
            raise FileNotFoundError(
                "pose_bones is not built. After sourcing source_env.sh: "
                "colcon build --packages-select argus_server --cmake-target pose_bones"
            )
        self._proc = subprocess.Popen(
            [str(binary), "--server", str(self.folder)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        poses = self._fk({})
        if poses is None:
            err = ""
            if self._proc.poll() is not None:
                err = (self._proc.stderr.read() or "").strip()
            self.close()
            raise RuntimeError(err or "pose_bones --server failed to load the OpenSim model")

    def _fk(self, angles):
        if self._proc is None or self._proc.stdin is None or self._proc.stdout is None:
            return None
        try:
            self._proc.stdin.write(json.dumps(angles) + "\n")
            self._proc.stdin.flush()
            line = self._proc.stdout.readline()
        except BrokenPipeError:
            return None
        if not line:
            return None
        try:
            return json.loads(line).get("bones") or {}
        except json.JSONDecodeError:
            return None

    def _load_visuals(self):
        for body, filename, scale, color in _body_meshes(self.osim_path):
            if self.visible_bodies is not None and body not in self.visible_bodies:
                continue
            path = _vtp_file(self.folder, filename)
            if path is None:
                continue
            poly = self._scaled_mesh(path, scale)
            if poly is None:
                continue
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(poly)
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color)
            actor.GetProperty().SetAmbient(0.25)
            actor.GetProperty().SetDiffuse(0.75)
            self.actors.append((body, actor))
            self.renderer.AddActor(actor)

    def _scaled_mesh(self, path, scale):
        key = (str(path), tuple(np.round(scale, 6)))
        cached = self._mesh_cache.get(key)
        if cached is not None:
            return cached
        reader = vtk.vtkXMLPolyDataReader()
        reader.SetFileName(str(path))
        reader.Update()
        output = reader.GetOutput()
        if output is None or output.GetNumberOfPoints() == 0:
            return None
        transform = vtk.vtkTransform()
        transform.Scale(float(scale[0]), float(scale[1]), float(scale[2]))
        filt = vtk.vtkTransformPolyDataFilter()
        filt.SetTransform(transform)
        filt.SetInputData(output)
        filt.Update()
        self._mesh_cache[key] = filt.GetOutput()
        return filt.GetOutput()

    def _resolved_angles(self):
        angles = dict(self.angles)
        for side in ("r", "l"):
            coupled = f"shoulder1_r2_{side}"
            source = f"elv_angle_{side}"
            if coupled not in angles and source in angles:
                angles[coupled] = -float(angles[source])
        return angles

    def apply(self, angles):
        self.angles = dict(angles)
        bones = self._fk(self._resolved_angles())
        if not bones:
            return
        identity = np.eye(4)
        for body, actor in self.actors:
            record = bones.get(body) or {}
            quat = record.get("orientation_wxyz")
            position = record.get("position")
            if quat is None or position is None:
                pose = identity
            else:
                pose = _wxyz_pose(quat, position)
            actor.SetUserMatrix(_vtk_matrix(pose))

    def _frame_camera(self):
        mins = np.full(3, np.inf)
        maxs = np.full(3, -np.inf)
        for _body, actor in self.actors:
            bounds = actor.GetBounds()
            mins = np.minimum(mins, (bounds[0], bounds[2], bounds[4]))
            maxs = np.maximum(maxs, (bounds[1], bounds[3], bounds[5]))
        center = (mins + maxs) / 2
        distance = float(max(np.max(maxs - mins), 0.4) * 2.75)
        camera = self.renderer.GetActiveCamera()
        camera.SetFocalPoint(float(center[0]), float(center[1]), float(center[2]))
        camera.SetViewUp(0.0, 1.0, 0.0)
        camera.SetPosition(
            float(center[0] - distance),
            float(center[1] + distance * 0.08),
            float(center[2] + distance * 0.55),
        )
        self.renderer.ResetCameraClippingRange()

    def render(self, width, height):
        width = max(80, int(width))
        height = max(80, int(height))
        self.window.SetSize(width, height)
        self.window.Render()
        self.capture.Modified()
        self.capture.Update()
        image = self.capture.GetOutput()
        width, height, _depth = image.GetDimensions()
        raw = vtk_to_numpy(image.GetPointData().GetScalars())
        frame = np.flipud(raw.reshape(height, width, -1)[:, :, :3])
        return np.ascontiguousarray(frame)

    def close(self):
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        try:
            if proc.stdin:
                proc.stdin.close()
        except BrokenPipeError:
            pass
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


class SkeletonModel:
    """Human from a URDF (upper body of human.urdf by default). Joint names match /joint_states.

    visible_links, if given, limits the drawn meshes to those links (the rest still move the chain).
    """

    def __init__(self, urdf_name="human.urdf", upper_body=True, visible_links=None):
        self.visible_links = set(visible_links) if visible_links else None
        urdf_path = _find_urdf(urdf_name)
        root = _upper_body_urdf(urdf_path.read_text()) if upper_body else ET.fromstring(urdf_path.read_text())
        self.joints = []
        children = {}
        child_links = set()
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            parent_name = parent.get("link")
            child_name = child.get("link")
            xyz, rpy = _origin(joint.find("origin"))
            axis_elem = joint.find("axis")
            axis = _xyz(axis_elem.get("xyz") if axis_elem is not None else "1 0 0")
            mimic = joint.find("mimic")
            record = {
                "name": joint.get("name"),
                "type": joint.get("type", "fixed"),
                "parent": parent_name,
                "child": child_name,
                "origin": _transform(xyz, rpy),
                "axis": axis,
                "mimic": None if mimic is None else (
                    mimic.get("joint"),
                    float(mimic.get("multiplier") or 1.0),
                    float(mimic.get("offset") or 0.0),
                ),
            }
            self.joints.append(record)
            children.setdefault(parent_name, []).append(record)
            child_links.add(child_name)
        self.children = children
        self.roots = [
            link.get("name")
            for link in root.findall("link")
            if link.get("name") and link.get("name") not in child_links
        ]
        if "world" in self.roots:
            self.roots = ["world"]
        self.angles = {}
        self.actors = []
        self._mesh_cache = {}
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.067, 0.075, 0.094)
        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.AddRenderer(self.renderer)
        self.window.SetSize(420, 560)
        self.capture = vtk.vtkWindowToImageFilter()
        self.capture.SetInput(self.window)
        self._load_visuals(root)
        self._frame_camera()

    def _load_visuals(self, root):
        for link in root.findall("link"):
            name = link.get("name")
            if self.visible_links is not None and name not in self.visible_links:
                continue
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh") if visual.find("geometry") is not None else None
                if mesh is None:
                    continue
                path = _mesh_file(mesh.get("filename") or "")
                if path is None:
                    continue
                scale = _xyz(mesh.get("scale") or "1 1 1")
                if scale.shape[0] != 3:
                    scale = np.ones(3)
                poly = self._scaled_mesh(path, scale)
                if poly is None:
                    continue
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputData(poly)
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                color = self._color(visual)
                actor.GetProperty().SetColor(*color)
                actor.GetProperty().SetAmbient(0.25)
                actor.GetProperty().SetDiffuse(0.75)
                xyz, rpy = _origin(visual.find("origin"))
                self.actors.append((name, _transform(xyz, rpy), actor))
                self.renderer.AddActor(actor)
        self.apply({})

    def _scaled_mesh(self, path, scale):
        key = (str(path), tuple(np.round(scale, 6)))
        cached = self._mesh_cache.get(key)
        if cached is not None:
            return cached
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(path))
        reader.Update()
        output = reader.GetOutput()
        if output is None or output.GetNumberOfPoints() == 0:
            return None
        transform = vtk.vtkTransform()
        transform.Scale(float(scale[0]), float(scale[1]), float(scale[2]))
        filt = vtk.vtkTransformPolyDataFilter()
        filt.SetTransform(transform)
        filt.SetInputData(output)
        filt.Update()
        self._mesh_cache[key] = filt.GetOutput()
        return filt.GetOutput()

    def _color(self, visual):
        color = visual.find("material/color")
        if color is None or not color.get("rgba"):
            return (0.86, 0.75, 0.68)
        parts = [float(part) for part in color.get("rgba").split()]
        return tuple(parts[:3])

    def _resolved_angles(self):
        # shoulder1_r2 is not in /joint_states. OpenSim sets it to -elv_angle
        # and solves shoulder_rot on top of that. Apply the same couple here,
        # unless a message already carries the joint.
        angles = dict(self.angles)
        for joint in self.joints:
            mimic = joint.get("mimic")
            if mimic is None or joint["name"] in angles:
                continue
            source, multiplier, offset = mimic
            angles[joint["name"]] = multiplier * float(angles.get(source, 0.0)) + offset
        return angles

    def _link_poses(self):
        poses = {}
        angles = self._resolved_angles()

        def visit(link, parent_pose):
            poses[link] = parent_pose
            for joint in self.children.get(link, []):
                angle = 0.0
                if joint["type"] in ("revolute", "continuous"):
                    angle = float(angles.get(joint["name"], 0.0))
                motion = joint["origin"] @ _axis_rotation(joint["axis"], angle)
                visit(joint["child"], parent_pose @ motion)

        for link in self.roots:
            visit(link, np.eye(4))
        return poses

    def apply(self, angles):
        self.angles = dict(angles)
        poses = self._link_poses()
        identity = np.eye(4)
        for link, visual_pose, actor in self.actors:
            pose = poses.get(link, identity) @ visual_pose
            actor.SetUserMatrix(_vtk_matrix(pose))

    def _frame_camera(self):
        mins = np.full(3, np.inf)
        maxs = np.full(3, -np.inf)
        for _link, _visual, actor in self.actors:
            bounds = actor.GetBounds()
            mins = np.minimum(mins, (bounds[0], bounds[2], bounds[4]))
            maxs = np.maximum(maxs, (bounds[1], bounds[3], bounds[5]))
        center = (mins + maxs) / 2
        distance = float(max(np.max(maxs - mins), 0.4) * 2.1)
        camera = self.renderer.GetActiveCamera()
        camera.SetFocalPoint(float(center[0]), float(center[1]), float(center[2]))
        camera.SetViewUp(0.0, 0.0, 1.0)
        camera.SetPosition(
            float(center[0] + distance * 0.55),
            float(center[1] - distance),
            float(center[2] + distance * 0.08),
        )
        self.renderer.ResetCameraClippingRange()

    def render(self, width, height):
        width = max(80, int(width))
        height = max(80, int(height))
        self.window.SetSize(width, height)
        self.window.Render()
        self.capture.Modified()
        self.capture.Update()
        image = self.capture.GetOutput()
        width, height, _depth = image.GetDimensions()
        raw = vtk_to_numpy(image.GetPointData().GetScalars())
        frame = np.flipud(raw.reshape(height, width, -1)[:, :, :3])
        return np.ascontiguousarray(frame)


class SkeletonView(tk.Frame):
    """Bottom-right panel. Pass joint angles in radians with set_joints."""

    def __init__(self, parent):
        super().__init__(parent, bg="#181b22", width=440, height=560, takefocus=0)
        self.pack_propagate(False)
        self.caption = tk.Label(
            self, text="Upper Limb  ·  waiting for joint states",
            bg="#181b22", fg="#8b919a", font=("DejaVu Sans", 11), takefocus=0,
        )
        self.caption.pack(fill="x", pady=(8, 4))
        self.image_label = tk.Label(self, bg="#181b22", takefocus=0)
        self.image_label.pack(fill="both", expand=True)
        self.model = opensim_skeleton_for_model()
        self.title = "Upper Limb"
        self.lock = threading.Lock()
        self.pending = {}
        self.dirty = True
        self.seen_joints = False
        self.photo = None
        self.drawn_size = None
        self.bind("<Configure>", self._resized)
        self.after(80, self._tick)

    def set_joints(self, angles):
        with self.lock:
            self.pending = dict(angles)
            self.dirty = True
            self.seen_joints = True

    def set_model(self, model, title):
        """Show another model (name under models/ or folder); the old joint angles are dropped."""
        replacement = opensim_skeleton_for_model(model)
        old = self.model
        self.model = replacement
        old.window.Finalize()
        if hasattr(old, "close"):
            old.close()
        self.title = title
        with self.lock:
            self.pending = {}
            self.dirty = True
            self.seen_joints = False

    def _resized(self, event):
        if event.widget is not self or event.width < 80 or event.height < 80:
            return
        size = (event.width, event.height)
        if size != self.drawn_size:
            self.dirty = True

    def _tick(self):
        try:
            if self.dirty:
                with self.lock:
                    angles = dict(self.pending)
                    self.dirty = False
                    live = self.seen_joints
                self.model.apply(angles)
                width = self.winfo_width()
                height = self.winfo_height() - self.caption.winfo_height()
                if width < 80 or height < 80:
                    width, height = 420, 500
                frame = self.model.render(width, height)
                self.drawn_size = (self.winfo_width(), self.winfo_height())
                image = Image.fromarray(frame)
                self.photo = ImageTk.PhotoImage(image)
                self.image_label.configure(image=self.photo)
                self.caption.configure(
                    text=f"{self.title}  ·  live" if live else f"{self.title}  ·  waiting for joint states"
                )
        except tk.TclError:
            return
        except Exception as exc:
            self.caption.configure(text=f"{self.title}  ·  {exc}")
        self.after(70, self._tick)
