"""Vision-based drift correction between Argus modules (optional companion to argus_server).

Every period_s seconds the node subscribes briefly to each module's camera,
takes the set of frames (one per module) closest together in header time, and
pairs each frame with its module's tracked orientation interpolated at the
frame's header stamp (shifted by the Kalibr camera-IMU time offset). ORB
features shared by two cameras give their relative orientation. Confident
pairs are solved for one rotation per module that makes the server's belief
agree with the images (the anchor module, on the model's root body, is held
fixed), and the change since the last zero is blended into the correction the
server applies. See fusion.py for the frames involved.

Works with either pose source (OpenVINS odomimu_throttled or the Mahony
mahoney_odomimu). The server works the same without this node; corrections
are made for one zero only and are dropped by the server after the next.

    ros2 run argus_drift drift_node --ros-args -p model:=rajagopal_upper -p period_s:=10.0
"""

import json
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from argus_drift import vision
from argus_drift.estimator import DriftEstimator, Settings, adjacent_module_pairs, pose_from_samples

POSE_SUFFIX = {"openvins": "odomimu_throttled", "vio": "odomimu_throttled", "odomimu": "odomimu_throttled",
               "mahoney": "mahoney_odomimu", "mahony": "mahoney_odomimu"}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def to_quat_msg(rotation, msg):
    x, y, z, w = rotation.as_quat()
    msg.x, msg.y, msg.z, msg.w = float(x), float(y), float(z), float(w)
    return msg


def from_quat_msg(q):
    return R.from_quat([q.x, q.y, q.z, q.w])


class DriftNode(Node):
    def __init__(self):
        super().__init__("argus_drift")
        p = self.declare_parameter
        self.model_name = p("model", "rajagopal_upper").value
        models_dir = Path(p("models_dir", "~/ProjectArgus/models").value).expanduser()
        calib_dir = Path(p("calib_dir", "~/ProjectArgus/argus_wide_calib").value).expanduser()
        pose_source = p("pose_source", "openvins").value.strip().lower()
        self.period = float(p("period_s", 5.0).value)
        self.capture_timeout = float(p("capture_timeout_s", 1.0).value)
        self.frames_per_module = int(p("frames_per_module", 3).value)
        self.image_topic = p("image_topic", "cam0/image_raw").value.strip("/")
        self.compressed = p("image_transport", "raw").value.strip().lower() == "compressed"
        anchor = p("anchor", "").value.strip()
        self.max_pose_gap = float(p("max_pose_gap_s", 0.15).value)
        settings = Settings(
            max_sync=float(p("max_sync_s", 0.05).value),
            max_speed=float(p("max_angular_speed_dps", 90.0).value),
            still_dps=float(p("still_dps", 30.0).value),
            max_wait=float(p("max_wait_s", 8.0).value),
            search_deg=float(p("search_deg", 45.0).value),
            group_gauges=bool(p("group_gauges", True).value),
            nfeatures=int(p("orb_features", 1500).value),
            px_threshold=float(p("ransac_px", 2.0).value),
            min_inliers=int(p("min_inliers", 25).value),
            min_ratio=float(p("min_inlier_ratio", 0.30).value),
            gate_deg=float(p("reject_deg", 75.0).value),
            max_off_deg=float(p("max_off_deg", 15.0).value),
            outlier_deg=float(p("pair_outlier_deg", 8.0).value),
            baseline_samples=int(p("baseline_samples", 2).value),
            gain=float(p("gain", 0.7).value),
            max_step=float(p("max_step_deg", 12.0).value),
            yaw_only=bool(p("yaw_only", True).value),
            world_up=tuple(p("world_up", [0.0, 0.0, 1.0]).value),
        )
        debug_dir = p("debug_dir", "").value.strip()
        self.debug_dir = Path(debug_dir).expanduser() if debug_dir else None
        reference_topic = p("reference_topic", "/argus/drift_reference").value
        correction_topic = p("correction_topic", "/argus/drift_correction").value

        if pose_source not in POSE_SUFFIX:
            raise SystemExit(f"pose_source must be openvins or mahoney, got {pose_source!r}")
        folder = Path(self.model_name).expanduser()
        if not folder.is_dir():
            folder = models_dir / self.model_name
        config = json.loads((folder / "argus_model.json").read_text())
        self.modules = list(config["modules"])
        if not anchor:
            root_body = config.get("root", {}).get("body")
            anchor = next((m for m, body in config["modules"].items() if body == root_body), self.modules[0])
        self.anchor = anchor

        self.cameras = {}
        for module in self.modules:
            path = vision.find_camera_file(calib_dir, module)
            if path is None:
                self.get_logger().warn(f"{module}: no single {module}-*camchain-imucam.yaml in {calib_dir}; "
                                       "its camera is not used")
                continue
            self.cameras[module] = vision.load_camera(path, module)
        priority = {m: config["sensor_weights"].get(body, 1.0) for m, body in config["modules"].items()}
        adjacent = adjacent_module_pairs(config["modules"], config.get("body_parents"))
        self.estimator = DriftEstimator(
            self.cameras, self.anchor, settings, priority, adjacent_pairs=adjacent or None)
        self.get_logger().info(
            f"model {config['name']}, anchor {self.anchor} ({config['modules'].get(self.anchor)}), "
            f"poses from {POSE_SUFFIX[pose_source]}, cameras {sorted(self.cameras)}, every {self.period:g} s"
            + (f", adjacent {sorted(tuple(sorted(p)) for p in adjacent)}" if adjacent else ""))

        self.lock = threading.Lock()
        self.poses = {m: deque(maxlen=4000) for m in self.modules}
        self.epoch = None           # server zero stamp (builtin_interfaces/Time)
        self.reference = {}         # module -> R_ref
        self.server_drift = {}      # module -> W the server applies
        self.baseline = {}          # module -> list of B samples, then the mean
        self.busy = False
        self.cycle = 0
        self.zero_capture_due = None
        self.capture_requested = None

        sensors = ReentrantCallbackGroup()
        timers = MutuallyExclusiveCallbackGroup()
        for module in self.modules:
            self.create_subscription(Odometry, f"/{module}/{POSE_SUFFIX[pose_source]}",
                                     lambda msg, m=module: self.on_pose(m, msg), qos_profile_sensor_data,
                                     callback_group=sensors)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, reference_topic, self.on_reference, latched, callback_group=sensors)
        self.correction_pub = self.create_publisher(TFMessage, correction_topic, 10)
        self.debug_pub = self.create_publisher(String, "/argus/drift_debug", 10)
        self.create_timer(self.period, self.request_capture, callback_group=timers)
        self.create_timer(0.1, self.check_capture, callback_group=timers)
        if self.debug_dir:
            self.debug_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------ inputs

    def on_pose(self, module, msg):
        t = stamp_seconds(msg.header.stamp)
        q = msg.pose.pose.orientation
        with self.lock:
            buffer = self.poses[module]
            if buffer and t <= buffer[-1][0]:
                if t < buffer[-1][0] - 1.0:
                    buffer.clear()      # clock jumped back (bag restarted)
                else:
                    return
            buffer.append((t, (q.x, q.y, q.z, q.w)))

    def on_reference(self, msg):
        with self.lock:
            epoch = None
            reference, drift = {}, {}
            for item in msg.transforms:
                epoch = item.header.stamp
                name = item.child_frame_id
                if name.endswith("/drift"):
                    drift[name[:-len("/drift")]] = from_quat_msg(item.transform.rotation)
                else:
                    reference[name] = from_quat_msg(item.transform.rotation)
            if epoch is None:
                return
            new_zero = self.epoch is None or Time.from_msg(epoch) != Time.from_msg(self.epoch)
            self.epoch, self.reference, self.server_drift = epoch, reference, drift
            if new_zero:
                self.baseline = {}
                self.zero_capture_due = time.monotonic() + 1.0
        if new_zero:
            self.get_logger().info("server zeroed: new drift baseline; first capture in 1 s")

    def pose_at(self, module, t):
        """Tracked orientation at t (slerp), and angular speed (deg/s); None if not covered."""
        with self.lock:
            samples = list(self.poses[module])
        times = np.array([s[0] for s in samples])
        quats = [s[1] for s in samples]
        return pose_from_samples(times, quats, t, self.max_pose_gap)

    # ------------------------------------------------------------ capture

    def request_capture(self):
        if self.capture_requested is None:
            self.capture_requested = time.monotonic()

    def current_speeds(self):
        """Each module's angular speed (deg/s) just before its newest pose."""
        speeds = {}
        for module in self.modules:
            with self.lock:
                newest = self.poses[module][-1][0] if self.poses[module] else None
            pose = self.pose_at(module, newest - 0.03) if newest is not None else None
            speeds[module] = pose[1] if pose else None
        return speeds

    def check_capture(self):
        """Start a requested capture once every module is still, or when it has waited max_wait."""
        if self.zero_capture_due is not None and time.monotonic() >= self.zero_capture_due:
            self.zero_capture_due = None
            self.capture_requested = time.monotonic()
        if self.capture_requested is None or self.busy:
            return
        with self.lock:
            zeroed = self.epoch is not None
        if not zeroed:
            self.capture_requested = None
            self.get_logger().info("waiting for the server's zero (/reset_tracking)", throttle_duration_sec=30.0)
            return
        waited = time.monotonic() - self.capture_requested
        if not self.estimator.still(self.current_speeds()) and waited < self.estimator.s.max_wait:
            return
        self.capture_requested = None
        self.busy = True
        threading.Thread(target=self.run_cycle, daemon=True).start()

    def capture_frames(self):
        """A few frames per camera. The cameras are only subscribed while capturing (raw images are
        large over WiFi); the subscriptions live on a short-lived node spun only by this thread."""
        frames = {m: [] for m in self.cameras}
        node = rclpy.create_node("argus_drift_capture", context=self.context)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(node)
        kind = CompressedImage if self.compressed else Image
        suffix = "/compressed" if self.compressed else ""

        def keep(module, msg):
            if len(frames[module]) < self.frames_per_module:
                frames[module].append((stamp_seconds(msg.header.stamp), msg))

        for module in self.cameras:
            node.create_subscription(kind, f"/{module}/{self.image_topic}{suffix}",
                                     lambda msg, m=module: keep(m, msg), qos_profile_sensor_data)
        started = time.monotonic()
        try:
            while time.monotonic() - started < self.capture_timeout:
                executor.spin_once(timeout_sec=0.02)
                if all(len(f) >= self.frames_per_module for f in frames.values()):
                    break
        finally:
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
        return {m: f for m, f in frames.items() if f}

    # ------------------------------------------------------------ processing

    def run_cycle(self):
        try:
            frames = self.capture_frames()
            with self.lock:
                epoch, reference, server_drift = self.epoch, dict(self.reference), dict(self.server_drift)
            self.process(frames, epoch, reference, server_drift)
        except Exception as exc:    # one bad cycle must not stop the node
            self.get_logger().error(f"drift cycle failed: {exc!r}")
        finally:
            self.busy = False

    def process(self, frames, epoch, reference, server_drift):
        self.cycle += 1
        chosen = self.estimator.pick_synced(frames)
        images, undecodable = {}, []
        for module, (t, msg) in chosen.items():
            try:
                images[module] = (t, vision.to_gray(msg))
            except ValueError as exc:
                undecodable.append(f"{module}: skipped ({exc})")
        with self.lock:
            if epoch is None or self.epoch is None or Time.from_msg(epoch) != Time.from_msg(self.epoch):
                return
            baseline = dict(self.baseline)
        result = self.estimator.cycle(images, self.pose_at, reference, server_drift, baseline,
                                      draw=self.debug_dir is not None)
        with self.lock:
            if self.epoch is None or Time.from_msg(epoch) != Time.from_msg(self.epoch):
                return      # zeroed again meanwhile; this cycle belongs to the old zero
            self.baseline = result.baseline
        if result.corrections:
            out = TFMessage()
            for module, rotation in result.corrections.items():
                item = TransformStamped()
                item.header.stamp = epoch
                item.header.frame_id = "ground"
                item.child_frame_id = module
                to_quat_msg(rotation, item.transform.rotation)
                out.transforms.append(item)
            self.correction_pub.publish(out)

        summary = f"drift cycle {self.cycle}: " + "; ".join(undecodable + result.lines)
        self.get_logger().info(summary)
        self.debug_pub.publish(String(data=summary))
        if self.debug_dir:
            for name, image in result.drawings:
                cv2.imwrite(str(self.debug_dir / f"{self.cycle:05d}_{name}.jpg"), image)
            report = {"cycle": self.cycle, "frames": {m: len(f) for m, f in frames.items()}, **result.report}
            with open(self.debug_dir / "drift_log.jsonl", "a", encoding="utf-8") as handle:
                handle.write(json.dumps(report) + "\n")


def main():
    rclpy.init()
    node = DriftNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
