"""Relative orientation of two module cameras from shared ORB features.

Cameras come from the Kalibr camchain-imucam files the modules use
(argus_wide_calib/<module>-...-camchain-imucam.yaml): intrinsics, distortion
(equidistant or radtan), the camera-IMU rotation, and the camera-IMU time
shift (t_imu = t_cam + timeshift_cam_imu).

relative_rotation() returns the orientation of camera b in camera a's frame
(R_a_b: b coordinates to a coordinates). It fits both an essential matrix and
a pure rotation; when the rotation alone explains the matches (little
parallax: a distant scene or a short baseline) the rotation fit is used,
because the essential matrix is degenerate there.
"""

from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


@dataclass
class Camera:
    name: str
    K: np.ndarray
    D: np.ndarray
    model: str          # "equidistant" or "radtan"
    imu_cam: R          # camera orientation in the IMU frame
    timeshift: float    # t_imu = t_cam + timeshift (s)
    size: tuple         # (width, height) the intrinsics are for

    @property
    def focal(self):
        return float(self.K[0, 0] + self.K[1, 1]) / 2.0

    def for_size(self, width, height):
        """Same camera with intrinsics scaled to another image size."""
        if (width, height) == tuple(self.size) or not self.size[0]:
            return self
        sx, sy = width / self.size[0], height / self.size[1]
        K = self.K.copy()
        K[0, :] *= sx
        K[1, :] *= sy
        return replace(self, K=K, size=(width, height))


def find_camera_file(calib_dir, module):
    files = sorted(Path(calib_dir).expanduser().glob(f"{module}-*camchain-imucam.yaml"))
    if len(files) != 1:
        return None
    return files[0]


def load_camera(path, name=None):
    data = yaml.safe_load(Path(path).read_text())["cam0"]
    fx, fy, cx, cy = data["intrinsics"]
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    T_cam_imu = np.array(data["T_cam_imu"], dtype=float)
    return Camera(
        name=name or Path(path).name.split("-")[0],
        K=K,
        D=np.array(data.get("distortion_coeffs", [0.0] * 4), dtype=float),
        model=data.get("distortion_model", "radtan"),
        imu_cam=R.from_matrix(T_cam_imu[:3, :3]).inv(),
        timeshift=float(data.get("timeshift_cam_imu", 0.0)),
        size=tuple(data.get("resolution", (0, 0))),
    )


def undistort(camera, pixels):
    """Pixel coordinates (N, 2) to normalized image coordinates (N, 2)."""
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    if camera.model == "equidistant":
        out = cv2.fisheye.undistortPoints(points, camera.K, camera.D.reshape(-1, 1))
    else:
        out = cv2.undistortPoints(points, camera.K, camera.D)
    return out.reshape(-1, 2)


@dataclass
class Features:
    pixels: np.ndarray      # (N, 2)
    bearings: np.ndarray    # (N, 3) unit vectors in the camera frame
    descriptors: np.ndarray
    keypoints: list


def make_orb(nfeatures=1500):
    return cv2.ORB_create(nfeatures=nfeatures, scaleFactor=1.2, nlevels=8, fastThreshold=12)


def detect(image, camera, orb):
    keypoints, descriptors = orb.detectAndCompute(image, None)
    if descriptors is None or len(keypoints) < 8:
        return None
    pixels = np.array([k.pt for k in keypoints])
    normalized = undistort(camera, pixels)
    bearings = np.column_stack([normalized, np.ones(len(normalized))])
    bearings /= np.linalg.norm(bearings, axis=1, keepdims=True)
    return Features(pixels, bearings, descriptors, keypoints)


@dataclass
class Measurement:
    ok: bool
    rotation: R = None      # R_a_b
    matches: int = 0
    inliers: int = 0
    model: str = ""
    residual_deg: float = float("nan")
    reason: str = ""
    pairs: np.ndarray = None    # (M, 2) indices of inlier matches, for drawing

    @property
    def ratio(self):
        return self.inliers / self.matches if self.matches else 0.0


def match(fa, fb, ratio=0.8):
    """Mutual nearest ORB matches that pass Lowe's ratio test, as (M, 2) indices."""
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = matcher.knnMatch(fa.descriptors, fb.descriptors, k=2)
    backward = {m.queryIdx: m.trainIdx for m in matcher.match(fb.descriptors, fa.descriptors)}
    pairs = [(m.queryIdx, m.trainIdx) for pair in forward if len(pair) == 2
             for m, n in [pair] if m.distance < ratio * n.distance and backward.get(m.trainIdx) == m.queryIdx]
    return np.array(pairs, dtype=int).reshape(-1, 2)


def _angles(a, b):
    return np.arccos(np.clip(np.sum(a * b, axis=1), -1.0, 1.0))


def rotation_ransac(ba, bb, threshold, iterations=300, rng=None):
    """Rotation C with ba ~ C bb (points at infinity), and its inlier mask."""
    rng = rng or np.random.default_rng(0)
    n = len(ba)
    best = np.zeros(n, bool)
    for _ in range(iterations):
        pick = rng.choice(n, 2, replace=False)
        if np.linalg.norm(np.cross(ba[pick[0]], ba[pick[1]])) < 1e-3:
            continue
        guess, _ = R.align_vectors(ba[pick], bb[pick])
        inliers = _angles(ba, guess.apply(bb)) < threshold
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 3:
        return None, best
    for _ in range(2):
        fit, _ = R.align_vectors(ba[best], bb[best])
        best = _angles(ba, fit.apply(bb)) < threshold
        if best.sum() < 3:
            return None, best
    fit, _ = R.align_vectors(ba[best], bb[best])
    return fit, best


def relative_rotation(fa, fb, camera_a, camera_b, px_threshold=2.0, ratio=0.8):
    """R_a_b from two feature sets, or a Measurement with ok False and the reason."""
    if fa is None or fb is None:
        return Measurement(False, reason="no features")
    pairs = match(fa, fb, ratio)
    return estimate(fa.bearings, fb.bearings, pairs, np.sqrt(camera_a.focal * camera_b.focal), px_threshold)


def estimate(bearings_a, bearings_b, pairs, focal, px_threshold=2.0):
    """R_a_b from matched bearings (pairs: (M, 2) indices), essential matrix or pure rotation."""
    if len(pairs) < 12:
        return Measurement(False, matches=len(pairs), reason="too few matches")
    ba, bb = bearings_a[pairs[:, 0]], bearings_b[pairs[:, 1]]
    threshold = px_threshold / focal

    rot_fit, rot_inliers = rotation_ransac(ba, bb, threshold)

    na = ba[:, :2] / ba[:, 2:3]
    nb = bb[:, :2] / bb[:, 2:3]
    ess_fit, ess_inliers = None, np.zeros(len(pairs), bool)
    E, mask = cv2.findEssentialMat(na, nb, focal=1.0, pp=(0.0, 0.0), method=cv2.RANSAC,
                                   prob=0.999, threshold=threshold)
    if E is not None and E.shape[0] >= 3 and mask is not None:
        _, R_ba, _t, pose_mask = cv2.recoverPose(E[:3], na, nb, focal=1.0, pp=(0.0, 0.0), mask=mask.copy())
        ess_inliers = pose_mask.ravel() > 0
        ess_fit = R.from_matrix(R_ba).inv()

    # A rotation that explains nearly every essential inlier means the
    # translation is not observable, and the essential matrix rotation with it.
    if rot_fit is not None and rot_inliers.sum() >= 0.9 * max(ess_inliers.sum(), 1):
        fit, inliers, model = rot_fit, rot_inliers, "rotation"
    elif ess_fit is not None:
        fit, inliers, model = ess_fit, ess_inliers, "essential"
    else:
        return Measurement(False, matches=len(pairs), reason="no model")
    residual = np.degrees(np.median(_angles(ba[inliers], fit.apply(bb[inliers])))) if model == "rotation" \
        else float("nan")
    return Measurement(True, fit, len(pairs), int(inliers.sum()), model, residual, pairs=pairs[inliers])


class VirtualView:
    """A pinhole view between two cameras, so both images can be compared without lens
    distortion or roll. Bearings in the view are rotated into each camera."""

    def __init__(self, axis_a, axis_b, size=(640, 480), focal=320.0, down=(0.0, -1.0, 0.0)):
        z = axis_a + axis_b
        z = z / np.linalg.norm(z) if np.linalg.norm(z) > 1e-6 else axis_a
        # Upright in the common frame (OpenSim ground: Y up) unless looking straight up or down.
        down = np.asarray(down, float)
        x = np.cross(down, z)
        if np.linalg.norm(x) < 0.3:
            x = np.cross([1.0, 0.0, 0.0], z)
        x /= np.linalg.norm(x)
        self.orientation = R.from_matrix(np.column_stack([x, np.cross(z, x), z]))   # view in the common frame
        self.size, self.focal = size, focal
        w, h = size
        u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
        rays = np.column_stack([(u.ravel() - w / 2) / focal, (v.ravel() - h / 2) / focal, np.ones(w * h)])
        self.rays = rays / np.linalg.norm(rays, axis=1, keepdims=True)

    def bearings(self, pixels):
        w, h = self.size
        rays = np.column_stack([(pixels[:, 0] - w / 2) / self.focal, (pixels[:, 1] - h / 2) / self.focal,
                                np.ones(len(pixels))])
        return rays / np.linalg.norm(rays, axis=1, keepdims=True)

    def render(self, image, camera, camera_orientation):
        """The camera's image seen from this view; camera_orientation is the camera in the common frame.
        Returns the image and the rotation from view to camera coordinates."""
        to_camera = camera_orientation.inv() * self.orientation
        rays = to_camera.apply(self.rays)
        w, h = self.size
        ok = rays[:, 2] > 0.1
        pixels = np.full((len(rays), 2), -1.0)
        points = rays[ok].reshape(-1, 1, 3)
        if camera.model == "equidistant":
            projected, _ = cv2.fisheye.projectPoints(points, np.zeros(3), np.zeros(3), camera.K, camera.D.reshape(-1, 1))
        else:
            projected, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), camera.K, camera.D)
        pixels[ok] = projected.reshape(-1, 2)
        map_x = pixels[:, 0].reshape(h, w).astype(np.float32)
        map_y = pixels[:, 1].reshape(h, w).astype(np.float32)
        view = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        valid = cv2.remap(np.full(image.shape, 255, np.uint8), map_x, map_y, cv2.INTER_NEAREST,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return view, cv2.erode(valid, np.ones((15, 15), np.uint8)), to_camera


def guided_match(fa, fb, radius_px, ratio=0.8):
    """Matches whose pixels lie within radius_px of each other (both in the same virtual view),
    best and second best taken among those only, kept when mutual."""
    if fa is None or fb is None or len(fa.pixels) < 2 or len(fb.pixels) < 2:
        return np.zeros((0, 2), int)
    bits_a = np.unpackbits(fa.descriptors, axis=1).astype(np.float32)
    bits_b = np.unpackbits(fb.descriptors, axis=1).astype(np.float32)
    distance = (bits_a @ (1.0 - bits_b).T + (1.0 - bits_a) @ bits_b.T).astype(np.float64)   # Hamming
    gap = np.linalg.norm(fa.pixels[:, None, :] - fb.pixels[None, :, :], axis=2)
    distance[gap > radius_px] = np.inf
    best_b = np.argmin(distance, axis=1)
    best_a = np.argmin(distance, axis=0)
    pairs = []
    for i, j in enumerate(best_b):
        d = distance[i, j]
        if not np.isfinite(d) or best_a[j] != i:
            continue
        row = distance[i].copy()
        row[j] = np.inf
        if np.isfinite(row.min()) and d >= ratio * row.min():
            continue
        pairs.append((i, j))
    return np.array(pairs, dtype=int).reshape(-1, 2)


def guided_rotation(image_a, camera_a, belief_a, image_b, camera_b, belief_b, orb,
                    px_threshold=2.0, search_deg=20.0, max_axis_deg=120.0):
    """R_a_b, matching both images in a common undistorted view placed by the believed camera
    orientations (belief_*: camera in a common frame). Returns (Measurement, drawing inputs)."""
    axis_a, axis_b = belief_a.apply([0.0, 0.0, 1.0]), belief_b.apply([0.0, 0.0, 1.0])
    if np.degrees(np.arccos(np.clip(np.dot(axis_a, axis_b), -1, 1))) > max_axis_deg:
        return Measurement(False, reason="cameras face apart"), None
    view = VirtualView(axis_a, axis_b)
    va, mask_a, view_to_a = view.render(image_a, camera_a, belief_a)
    vb, mask_b, view_to_b = view.render(image_b, camera_b, belief_b)
    features = []
    for image, mask, to_camera in ((va, mask_a, view_to_a), (vb, mask_b, view_to_b)):
        keypoints, descriptors = orb.detectAndCompute(image, mask)
        if descriptors is None or len(keypoints) < 8:
            return Measurement(False, reason="no shared view"), None
        pixels = np.array([k.pt for k in keypoints])
        features.append(Features(pixels, to_camera.apply(view.bearings(pixels)), descriptors, keypoints))
    fa, fb = features
    pairs = guided_match(fa, fb, view.focal * np.tan(np.radians(search_deg)))
    return estimate(fa.bearings, fb.bearings, pairs, view.focal, px_threshold), (va, vb, fa, fb)


def draw(image_a, image_b, fa, fb, measurement, title=""):
    """Side-by-side inlier matches, for the debug folder."""
    a = cv2.cvtColor(image_a, cv2.COLOR_GRAY2BGR)
    b = cv2.cvtColor(image_b, cv2.COLOR_GRAY2BGR)
    canvas = np.zeros((max(a.shape[0], b.shape[0]), a.shape[1] + b.shape[1], 3), np.uint8)
    canvas[:a.shape[0], :a.shape[1]] = a
    canvas[:b.shape[0], a.shape[1]:] = b
    if measurement.pairs is not None:
        for i, j in measurement.pairs:
            p = tuple(int(v) for v in fa.pixels[i])
            q = (int(fb.pixels[j][0]) + a.shape[1], int(fb.pixels[j][1]))
            cv2.line(canvas, p, q, (60, 200, 90), 1, cv2.LINE_AA)
    cv2.putText(canvas, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA)
    return canvas


def to_gray(msg):
    """sensor_msgs Image or CompressedImage to an 8-bit grayscale array."""
    if hasattr(msg, "format"):
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_GRAYSCALE)
    encoding = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(h, step)
    if encoding in ("mono8", "8uc1"):
        return np.ascontiguousarray(rows[:, :w])
    if encoding in ("mono16", "16uc1"):
        return np.ascontiguousarray((rows[:, :2 * w].view(np.uint16) >> 8).astype(np.uint8))
    if encoding in ("bgr8", "rgb8", "8uc3"):
        color = rows[:, :3 * w].reshape(h, w, 3)
        return cv2.cvtColor(color, cv2.COLOR_RGB2GRAY if encoding == "rgb8" else cv2.COLOR_BGR2GRAY)
    if encoding in ("bgra8", "rgba8"):
        color = rows[:, :4 * w].reshape(h, w, 4)
        return cv2.cvtColor(color, cv2.COLOR_RGBA2GRAY if encoding == "rgba8" else cv2.COLOR_BGRA2GRAY)
    if encoding in ("yuv422", "uyvy", "yuv422_uyvy"):
        return cv2.cvtColor(rows[:, :2 * w].reshape(h, w, 2), cv2.COLOR_YUV2GRAY_UYVY)
    if encoding in ("yuyv", "yuv422_yuy2"):
        return cv2.cvtColor(rows[:, :2 * w].reshape(h, w, 2), cv2.COLOR_YUV2GRAY_YUY2)
    raise ValueError(f"unsupported image encoding {msg.encoding}")
