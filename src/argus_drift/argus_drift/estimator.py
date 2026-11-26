"""One drift-correction cycle, without ROS: shared by the live node and drift_offline.py.

Given one camera frame per module (header time and grayscale image), the
tracked orientations, the server's zero reference and its current
corrections, cycle() matches adjacent camera pairs (parent-child bodies when
configured; otherwise every pair), solves the per-module ground rotations
(fusion.py), keeps the baselines after a zero, and returns the new corrections.

Matching is guided by the belief: both images of a pair are re-rendered into
one undistorted view placed where the server believes the two cameras look,
and features are only matched near where the belief puts them. The fisheye
distortion and the different camera rolls then no longer stop ORB.

Modules linked to the anchor (root body) are corrected relative to it: W
is a rotation in ground, left-multiplied as R_GB = W R_ref R_cur C. A
group of modules linked only to each other (one arm) can observe relative
rotations within the group, but not the group's common heading. With
yaw_only (heading drift), those floating groups are skipped: applying a
yaw step there would move the unobservable gauge freedom and ratchet.
Without yaw_only the same leftover, held as a world-fixed W, walks into
distal joints (pronation) after the limb moves; those modules store a
body-fixed Δ = R_imu^{-1} G R_imu and apply R_cur' = R_cur Δ instead.
Baselines are kept per module and per fixed module.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as R

from argus_drift import fusion, vision


def pose_from_samples(times, quats, t, max_gap):
    """Orientation at t by slerp between the samples around it (quats xyzw), and the angular speed
    (deg/s) between them. None when the samples do not cover t within max_gap."""
    if len(times) < 2:
        return None
    i = int(np.searchsorted(times, t))
    if i == 0 or i == len(times):
        # Just outside the samples: the end sample, if it is close enough.
        nearest = 0 if i == 0 else len(times) - 1
        if abs(times[nearest] - t) > max_gap / 2:
            return None
        i = 1 if i == 0 else len(times) - 1
    t0, t1 = times[i - 1], times[i]
    if t1 - t0 > max_gap:
        return None
    pair = R.from_quat([quats[i - 1], quats[i]])
    relative = pair[0].inv() * pair[1]
    speed = np.degrees(relative.magnitude()) / max(t1 - t0, 1e-6)
    fraction = float(np.clip((t - t0) / max(t1 - t0, 1e-9), 0.0, 1.0))
    return pair[0] * R.from_rotvec(relative.as_rotvec() * fraction), float(speed)


@dataclass
class Settings:
    max_sync: float = 0.05
    max_speed: float = 90.0
    still_dps: float = 30.0        # a capture waits for every module to turn slower than this ...
    max_wait: float = 8.0          # ... for at most this long
    nfeatures: int = 1500
    px_threshold: float = 2.0
    search_deg: float = 45.0
    min_inliers: int = 25
    min_ratio: float = 0.30
    min_matches: int = 12        # pairs needed before RANSAC (vision.estimate)
    gate_deg: float = 75.0
    # Hard cap: reject a pair whose measured relative rotation disagrees with the
    # pose belief by more than this, even with many inliers (high-inlier wrong
    # matches were walking heading corrections up tens of degrees).
    max_off_deg: float = 15.0
    outlier_deg: float = 8.0
    baseline_samples: int = 2
    gain: float = 0.7
    max_step: float = 12.0
    rel_max_step: float = 8.0      # cap on floating-group body Δ per capture
    yaw_only: bool = True
    torso_path: bool = False  # skip groups with no camera path to the torso
    rel_parent: bool = False  # snap child IMU to parent from the camera pair (no leftover G)
    rel_bones: bool = False  # forearm Δ = change since still, model elbow hinge only
    rel_min_inliers: int = 25
    rel_min_ratio: float = 0.30
    rel_snap_step: float = 15.0  # max Δ change per capture (deg)
    rel_gyro_max_deg: float = 30.0  # skip bone Δ if camera hinge disagrees with IMU since still
    world_up: tuple = (0.0, 0.0, 1.0)
    group_gauges: bool = True


@dataclass
class CycleResult:
    corrections: dict = field(default_factory=dict)   # module -> new W (ground); includes inherited dW
    body: dict = field(default_factory=dict)          # module -> new IMU-body Δ (relative leftover)
    clear_body: list = field(default_factory=list)    # modules whose Δ is reset (now torso-connected)
    body_fixed: dict = field(default_factory=dict)    # module -> True if a body Δ was updated
    baseline: dict = field(default_factory=dict)      # "module@fixed" -> list of samples, or the mean rotation
    lines: list = field(default_factory=list)
    report: dict = field(default_factory=dict)
    drawings: list = field(default_factory=list)      # (name, BGR image) for matched pairs


def adjacent_module_pairs(modules, body_parents):
    """Frozensets of {module_a, module_b} for each parent-child body edge.

    modules: module name -> body name. body_parents: body -> parent body.
    Pairs whose bodies are not a direct parent-child link are omitted (e.g. torso
    vs forearm, or left vs right arm).
    """
    body_to_modules = {}
    for module, body in modules.items():
        body_to_modules.setdefault(body, []).append(module)
    pairs = set()
    for body, parent in (body_parents or {}).items():
        for a in body_to_modules.get(body, []):
            for b in body_to_modules.get(parent, []):
                pairs.add(frozenset((a, b)))
    return pairs


def module_parents(modules, body_parents, anchor):
    """Kinematic parent module of each module (child module -> parent module)."""
    body_to_module = {body: module for module, body in modules.items()}
    parents = {}
    for module, body in modules.items():
        parent_body = (body_parents or {}).get(body)
        if parent_body in body_to_module:
            parents[module] = body_to_module[parent_body]
        elif module != anchor:
            parents[module] = anchor
    return parents


def parent_child_pair(a, b, parents):
    """(child, parent) if a-b is a kinematic parent-child module pair."""
    if not parents:
        return None
    if parents.get(a) == b:
        return a, b
    if parents.get(b) == a:
        return b, a
    return None


def rel_delta_raw(child, parent, rotation, a, b, imu_R, cameras):
    """Body Δ that makes the child's IMU match the camera-relative to the parent.

    rotation is V_ab from guided_rotation (camera b in camera a). Applied as
    imu_child' = imu_child * Δ, with Δ = imu_c^{-1} * imu_p * cam_p * V * cam_c^{-1}
    (V inverted when a is the child).
    """
    cam_c = cameras[child].imu_cam
    cam_p = cameras[parent].imu_cam
    v = rotation.inv() if a == child else rotation
    return imu_R[child].inv() * imu_R[parent] * cam_p * v * cam_c.inv()


def bone_relative_delta(child, parent, rotation, a, b, imu_R, cameras, mounts):
    """Δ on the child IMU so R_bone_c = R_bone_p * R_p_c from the cameras.

    Server: R_bone = W * R_ref * R_cur * Δ * C. C is the known sensor-bone
    mount. Camera V is cam_b in cam_a. With inherited W the mounts cancel in
    Δ, but the constraint is the current bone-bone, not a still-pose leftover.
    """
    cam_c = cameras[child].imu_cam
    cam_p = cameras[parent].imu_cam
    v_pc = rotation.inv() if a == child else rotation
    C_p = mounts[parent] if parent in mounts else R.identity()
    C_c = mounts[child] if child in mounts else R.identity()
    r_pc = C_p.inv() * cam_p * v_pc * cam_c.inv() * C_c
    return imu_R[child].inv() * imu_R[parent] * C_p * r_pc * C_c.inv()


# Rajagopal PinJoint elbow: +Z of humerus_*_offset, xyz Euler (rad) in the parent bone.
_ELBOW_OFFSET_XYZ = {
    "humerus_r": (-0.0228627, 0.228018, 0.0051689),
    "humerus_l": (0.0228627, -0.228018, 0.0051689),
}


def elbow_axis_parent_bone(parent_body):
    """Unit elbow-flexion axis in the parent (humerus) bone frame."""
    off = _ELBOW_OFFSET_XYZ.get(parent_body)
    if off is None:
        return np.array([0.0, 0.0, 1.0])
    return R.from_euler("xyz", off).apply([0.0, 0.0, 1.0])


def hinge_in_child_imu(imu_c, imu_p, C_p, hinge_bone_p):
    """Elbow axis in the child IMU, where body Δ is applied. None if degenerate."""
    hinge_w = (imu_p * C_p).apply(np.asarray(hinge_bone_p, float))
    hinge = imu_c.inv().apply(hinge_w)
    n = np.linalg.norm(hinge)
    if n < 1e-8:
        return None
    return hinge / n


def project_hinge_delta(delta, hinge_imu):
    """Keep only the PinJoint elbow component of an IMU-frame Δ (no pronation)."""
    if hinge_imu is None:
        return None
    return fusion.twist(delta, hinge_imu)


def signed_hinge_deg(delta, hinge_imu):
    if hinge_imu is None:
        return None
    angle = float(np.degrees(np.dot(fusion.twist(delta, hinge_imu).as_rotvec(), hinge_imu)))
    return (angle + 180.0) % 360.0 - 180.0


def imu_relative_since(imu_c, imu_p, imu_c0, imu_p0):
    """Parent-relative IMU rotation of the child since a reference pose."""
    now = imu_p.inv() * imu_c
    then = imu_p0.inv() * imu_c0
    return then.inv() * now


def distal_arm_pair(a, b, parents, modules):
    """(child, parent) if this is a radius–humerus camera pair."""
    kin = parent_child_pair(a, b, parents)
    if not kin or not modules:
        return None
    child, par = kin
    if str(modules.get(child, "")).startswith("radius"):
        return child, par
    return None


def descendants(module, parents):
    """Kinematic children, grandchildren, … of module (parents: child -> parent)."""
    kids = [child for child, parent in (parents or {}).items() if parent == module]
    out = list(kids)
    for child in kids:
        out.extend(descendants(child, parents))
    return out


def inherit_ground(module, ground, has_abs, parents):
    """Ground W this module should use: its own if it has been torso-connected, else the parent's."""
    cur, seen = module, set()
    while cur is not None and cur not in seen:
        if cur in has_abs:
            return ground.get(cur, R.identity())
        seen.add(cur)
        cur = (parents or {}).get(cur)
    return ground.get(module, R.identity())


class DriftEstimator:
    def __init__(self, cameras, anchor, settings=None, priority=None, adjacent_pairs=None,
                 modules=None, mounts=None):
        self.cameras = cameras
        self.anchor = anchor
        self.s = settings or Settings()
        self.priority = dict(priority or {})    # module -> weight, picks a group's fixed module
        # None: match every pair. Set: only these frozenset({a, b}) module pairs.
        self.adjacent_pairs = adjacent_pairs
        self.modules = dict(modules or {})      # module -> OpenSim body
        self.mounts = dict(mounts or {})        # module -> C (sensor-bone)
        self._still_imu = {}                    # module -> R_imu at bone baseline

    def pick_synced(self, frames):
        """frames: module -> [(t, payload)]. One per module, as close together in time as possible."""
        best_key, best = None, {}
        for listing in frames.values():
            for t0, _ in listing:
                chosen = {}
                for module, items in frames.items():
                    t, payload = min(items, key=lambda item: abs(item[0] - t0))
                    if abs(t - t0) <= self.s.max_sync:
                        chosen[module] = (t, payload)
                times = [t for t, _ in chosen.values()]
                key = (len(chosen), -(max(times) - min(times)))
                if best_key is None or key > best_key:
                    best_key, best = key, chosen
        return best

    def still(self, speeds):
        """True when every module turns slower than still_dps (speeds: module -> deg/s or None)."""
        return all(v is not None and v < self.s.still_dps for v in speeds.values())

    def _baseline_for(self, module, baseline):
        """Baseline used to turn the current correction into an implied G.

        Prefer the anchor gauge (heading-observable). Fall back to any
        module@fixed baseline so belief tracks corrections after a local-gauge
        solve when yaw_only is off.
        """
        base = baseline.get(f"{module}@{self.anchor}")
        if isinstance(base, R):
            return base
        prefix = f"{module}@"
        for key, val in baseline.items():
            if key.startswith(prefix) and isinstance(val, R):
                return val
        return None

    def _predicted(self, module, baseline, current, imu_R=None, body_fixed=None,
                   ground=None, body=None, has_abs=None, parents=None):
        """G the belief implies now (identity before the module has a baseline).

        Applied orientation is W_ground * R_ref * R_cur * Δ_body, so the implied
        G is W_ground * Ad_{R_imu}(Δ_body). W_ground is this module's own if it
        has been solved vs the torso, otherwise the parent's (a humerus
        correction rides onto the forearm).
        """
        if module == self.anchor:
            return R.identity()
        if ground is not None:
            w = R.identity()
            m, seen = module, set()
            while m is not None and m not in seen:
                seen.add(m)
                if m in (has_abs or set()) and m in (ground or {}):
                    w = ground[m]
                    break
                m = (parents or {}).get(m)
            delta = (body or {}).get(module, R.identity())
            if imu_R and module in imu_R:
                ri = imu_R[module]
                return w * ri * delta * ri.inv()
            return w
        if body_fixed and body_fixed.get(module) and imu_R and module in imu_R:
            delta = current.get(module, R.identity())
            ri = imu_R[module]
            return ri * delta * ri.inv()
        base = self._baseline_for(module, baseline)
        if base is not None:
            return base.inv() * current.get(module, R.identity())
        return R.identity()

    def cycle(self, images, pose_at, reference, current, baseline, draw=False, body_fixed=None,
              ground=None, body=None, has_abs=None, parents=None):
        """images: module -> (header time, gray image); pose_at(module, t) -> (R, deg/s) or None;
        reference: module -> R_ref; current: module -> W (legacy single rotation);
        ground/body/has_abs: hierarchical state (W vs torso, Δ on the IMU);
        parents: child module -> parent module."""
        s = self.s
        out = CycleResult(baseline=dict(baseline))
        lines, report = out.lines, out.report
        if images:
            times = [t for t, _ in images.values()]
            report["spread_ms"] = round(1000 * (max(times) - min(times)), 1)
            report["frame_times"] = {m: t for m, (t, _image) in images.items()}

        P, imu_R, grays, cameras, skipped, speeds = {}, {}, {}, {}, {}, {}
        for module, (t, image) in images.items():
            if module not in reference:
                skipped[module] = "not in the server's zero"
                continue
            camera = self.cameras[module]
            pose = pose_at(module, t + camera.timeshift)
            if pose is None:
                skipped[module] = "no tracked pose at the frame time"
                continue
            orientation, speed = pose
            speeds[module] = round(speed, 1)
            if speed > s.max_speed:
                skipped[module] = f"moving {speed:.0f} deg/s"
                continue
            imu_R[module] = reference[module] * orientation
            P[module] = imu_R[module] * camera.imu_cam
            grays[module] = image
            cameras[module] = camera.for_size(image.shape[1], image.shape[0])
        report["skipped"] = skipped
        report["speeds"] = speeds
        for module, why in skipped.items():
            lines.append(f"{module}: skipped ({why})")

        predicted = {m: self._predicted(m, out.baseline, current, imu_R, body_fixed,
                                        ground, body, has_abs, parents) for m in P}
        orb = vision.make_orb(s.nfeatures)
        edges, pairs_report, snap_meas, bone_meas = [], [], [], []
        names = sorted(P)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if self.adjacent_pairs is not None and frozenset((a, b)) not in self.adjacent_pairs:
                    continue
                belief_a, belief_b = predicted[a] * P[a], predicted[b] * P[b]
                m, views = vision.guided_rotation(grays[a], cameras[a], belief_a, grays[b], cameras[b], belief_b,
                                                  orb, s.px_threshold, s.search_deg,
                                                  min_matches=s.min_matches)
                entry = {"pair": f"{a}-{b}", "matches": m.matches, "inliers": m.inliers, "model": m.model}
                accepted = m.ok and m.inliers >= s.min_inliers and m.ratio >= s.min_ratio
                # Keep the full measured relative rotation for the multi-camera
                # solve. Rewriting it as belief*cam-yaw (yaw_only_relative) is
                # nearly blind across an elbow and made G pose-dependent.
                pair_rotation = m.rotation
                if m.ok:
                    full_off = fusion.angle_deg((belief_a.inv() * belief_b).inv() * m.rotation)
                    if s.yaw_only:
                        # Gate on heading residual only; pitch/roll match error is
                        # dropped later by projecting each solved G to world yaw.
                        _, yaw = fusion.yaw_only_relative(
                            belief_a, belief_b, m.rotation, s.world_up)
                        off = abs(yaw)
                        entry["off_full_deg"] = round(full_off, 2)
                        entry["yaw_deg"] = round(yaw, 2)
                    else:
                        off = full_off
                    entry["off_deg"] = round(off, 2)
                    entry["measured_xyzw"] = [round(float(v), 6) for v in m.rotation.as_quat()]
                    entry["uncorrected_xyzw"] = [round(float(v), 6) for v in (P[a].inv() * P[b]).as_quat()]
                    # Heading-only: a ~180 deg flip is not yaw drift. Full SO(3)
                    # uses max_off_deg as the only hard cap (ablation: 180).
                    if s.yaw_only and accepted and off > 90.0:
                        accepted, m.reason = False, f"{off:.1f} deg yaw from belief (flip)"
                    elif accepted and off > s.max_off_deg:
                        accepted, m.reason = False, f"{off:.1f} deg from belief"
                    elif accepted and off > s.gate_deg and m.inliers < 2 * s.min_inliers:
                        accepted, m.reason = False, f"{off:.1f} deg from belief"
                    elif not accepted:
                        m.reason = f"{m.inliers} inliers of {m.matches}"
                entry["accepted"] = accepted
                if not accepted:
                    entry["reason"] = m.reason
                pairs_report.append(entry)
                kin = parent_child_pair(a, b, parents)
                bone_kin = distal_arm_pair(a, b, parents, self.modules) if s.rel_bones else None
                if accepted:
                    if (s.rel_parent and kin) or bone_kin:
                        entry["world_edge"] = False
                    else:
                        weight = min(1.0, m.inliers / (2.0 * s.min_inliers)) * min(1.0, m.ratio / 0.6)
                        edges.append((a, b, pair_rotation, weight))
                rel_ok = (m.ok and pair_rotation is not None
                          and m.inliers >= s.rel_min_inliers and m.ratio >= s.rel_min_ratio
                          and off <= 90.0)
                if s.rel_parent and kin and not bone_kin and rel_ok:
                    child, par = kin
                    snap_meas.append((child, par, pair_rotation, a, b, m.inliers))
                    entry["rel_snap"] = True
                if bone_kin and rel_ok:
                    child, par = bone_kin
                    bone_meas.append((child, par, pair_rotation, a, b, m.inliers))
                    entry["bone_rel"] = True
                if draw and m.ok and views is not None:
                    va, vb, fa, fb = views
                    title = f"{a}-{b} {m.model} {m.inliers}/{m.matches} " + ("ok" if accepted else m.reason)
                    out.drawings.append((f"{a}_{b}", vision.draw(va, vb, fa, fb, m, title)))
        report["pairs"] = pairs_report
        for entry in pairs_report:
            lines.append(f"{entry['pair']}: {entry['inliers']}/{entry['matches']} {entry.get('model', '')} "
                         + ("accepted" if entry["accepted"] else f"rejected ({entry.get('reason', '')})"))

        if not edges and not snap_meas and not bone_meas:
            lines.append("no confident camera pairs; no correction")
            return out
        drift = {}
        ground_state = dict(ground or {})
        body_state = dict(body or {})
        abs_now = set(has_abs or [])
        identity = R.identity()
        kept = []
        G, gauge = {}, {}
        if edges:
            if s.group_gauges:
                G, gauge, kept = fusion.solve_groups(self.anchor, self.priority, P, edges, s.outlier_deg)
            else:
                if self.anchor not in P:
                    lines.append(f"anchor {self.anchor} has no usable frame; no correction")
                    return out
                G, kept, _r = fusion.solve_robust(self.anchor, P, edges, s.outlier_deg)
                gauge = {m: self.anchor for m in G}
            report["dropped_pairs"] = len(edges) - len(kept)
            report["fixed"] = {m: g for m, g in gauge.items()}
            if s.yaw_only:
                up_ground = np.asarray(s.world_up, float)
                G = {m: fusion.yaw_about(up_ground, fusion.signed_yaw_deg(g, up_ground))
                     for m, g in G.items()}
            for module, g in G.items():
                fixed = gauge[module]
                if module == fixed:
                    continue
                parent = (parents or {}).get(module)
                # Distal IMUs are seated on the parent by rel_parent / bone-relative, not world W.
                if (s.rel_parent or s.rel_bones) and parent not in (None, self.anchor):
                    continue
                now_body = False
                if fixed != self.anchor:
                    if parent == fixed:
                        now_body = True
                    elif s.yaw_only or s.torso_path:
                        lines.append(f"{module} (to {fixed}): skipped (no path to {self.anchor})")
                        continue
                    else:
                        now_body = True
                key = f"{module}@{fixed}"
                if now_body:
                    w = body_state.get(module, identity)
                    g_use = fusion.as_body(g, imu_R[module]) if module in imu_R else g
                    up = (imu_R[module].inv().apply(np.asarray(s.world_up, float))
                          if module in imu_R else reference[module].apply(np.asarray(s.world_up, float)))
                else:
                    w = ground_state.get(module, identity)
                    g_use = g
                    up = reference[module].apply(np.asarray(s.world_up, float))
                base = out.baseline.get(key)
                if not isinstance(base, R):
                    samples = (base or []) + [w * g_use.inv()]
                    if len(samples) >= s.baseline_samples:
                        out.baseline[key] = fusion.mean_rotation(samples)
                        lines.append(f"{module} (to {fixed}): baseline set from {len(samples)} captures")
                    else:
                        out.baseline[key] = samples
                        lines.append(f"{module} (to {fixed}): baseline {len(samples)}/{s.baseline_samples}")
                    continue
                confidence = max(wt for a, b, _v, wt in kept if module in (a, b))
                step_yaw = s.yaw_only
                step_cap = s.rel_max_step if (now_body and s.yaw_only) else s.max_step
                new, full, applied = fusion.step_toward(
                    w, base * g_use, s.gain * confidence, step_cap, step_yaw, up)
                drift[module] = round(full, 3)
                if now_body:
                    out.body[module] = new
                    out.body_fixed[module] = True
                    body_state[module] = new
                    lines.append(f"{module} (to {fixed}): drift {full:.2f} deg, applied {applied:.2f} deg, "
                                 f"body Δ now {fusion.angle_deg(new):.2f} deg")
                else:
                    out.corrections[module] = new
                    ground_state[module] = new
                    abs_now.add(module)
                    if fusion.angle_deg(body_state.get(module, identity)) > 1e-6:
                        out.clear_body.append(module)
                        body_state[module] = identity
                    lines.append(f"{module} (to {fixed}): drift {full:.2f} deg, applied {applied:.2f} deg, "
                                 f"ground W now {fusion.angle_deg(new):.2f} deg")

        if s.rel_parent and snap_meas:
            up = np.asarray(s.world_up, float)
            by_child = {}
            for child, par, rotation, a, b, inliers in snap_meas:
                if child not in imu_R or par not in imu_R:
                    continue
                raw = rel_delta_raw(child, par, rotation, a, b, imu_R, cameras)
                by_child.setdefault(child, []).append((par, raw, inliers))
            for child, items in by_child.items():
                par = items[0][0]
                raw = fusion.mean_rotation([r for _p, r, _i in items])
                key = f"{child}@{par}"
                base = out.baseline.get(key)
                if not isinstance(base, R):
                    samples = (base or []) + [raw]
                    if len(samples) >= s.baseline_samples:
                        out.baseline[key] = fusion.mean_rotation(samples)
                        lines.append(f"{child} (rel {par}): baseline set from {len(samples)} captures")
                    else:
                        out.baseline[key] = samples
                        lines.append(f"{child} (rel {par}): baseline {len(samples)}/{s.baseline_samples}")
                    continue
                # Snap: Δ = Δ_raw * B^{-1} so still-pose relative is unchanged.
                target = raw * base.inv()
                current_d = body_state.get(child, identity)
                new, full, applied = fusion.step_toward(
                    current_d, target, s.gain, s.rel_snap_step, False, up)
                out.body[child] = new
                out.body_fixed[child] = True
                body_state[child] = new
                drift[child] = round(full, 3)
                lines.append(f"{child} (rel {par}): snap {full:.2f} deg, applied {applied:.2f} deg, "
                             f"body Δ now {fusion.angle_deg(new):.2f} deg")

        if s.rel_bones and bone_meas:
            by_child = {}
            for child, par, rotation, a, b, inliers in bone_meas:
                if child not in imu_R or par not in imu_R:
                    continue
                raw = bone_relative_delta(child, par, rotation, a, b, imu_R, cameras, self.mounts)
                by_child.setdefault(child, []).append((par, raw, inliers))
            for child, items in by_child.items():
                par = items[0][0]
                raw = fusion.mean_rotation([r for _p, r, _i in items])
                key = f"{child}@{par}"
                base = out.baseline.get(key)
                if not isinstance(base, R):
                    samples = (base or []) + [raw]
                    if len(samples) >= s.baseline_samples:
                        out.baseline[key] = fusion.mean_rotation(samples)
                        self._still_imu[child] = imu_R[child]
                        self._still_imu[par] = imu_R[par]
                        lines.append(f"{child} (bone {par}): still baseline from {len(samples)} captures")
                    else:
                        out.baseline[key] = samples
                        lines.append(f"{child} (bone {par}): baseline {len(samples)}/{s.baseline_samples}")
                    continue
                # Change since still: keep the mocap-zeroed rest pose.
                target = raw * base.inv()
                angle = fusion.angle_deg(target)
                if angle > 90.0:
                    lines.append(f"{child} (bone {par}): skipped ({angle:.1f} deg from still, likely flip)")
                    continue
                C_p = self.mounts[par] if par in self.mounts else identity
                parent_body = self.modules.get(par, "")
                hinge_bone = elbow_axis_parent_bone(parent_body)
                hinge = hinge_in_child_imu(imu_R[child], imu_R[par], C_p, hinge_bone)
                if hinge is None:
                    lines.append(f"{child} (bone {par}): skipped (degenerate elbow axis)")
                    continue
                cam_hinge = signed_hinge_deg(target, hinge)
                still = self._still_imu
                if child in still and par in still:
                    gyro_d = imu_relative_since(
                        imu_R[child], imu_R[par], still[child], still[par])
                    hinge_p = C_p.apply(hinge_bone)
                    pn = np.linalg.norm(hinge_p)
                    gyro_hinge = (signed_hinge_deg(gyro_d, hinge_p / pn) if pn > 1e-8 else None)
                    if gyro_hinge is not None and abs(cam_hinge - gyro_hinge) > s.rel_gyro_max_deg:
                        lines.append(
                            f"{child} (bone {par}): skipped (cam hinge {cam_hinge:.1f} vs "
                            f"IMU {gyro_hinge:.1f} deg)")
                        continue
                projected = project_hinge_delta(target, hinge)
                current_d = body_state.get(child, identity)
                n_inl = max(i for _p, _r, i in items)
                confidence = min(1.0, n_inl / (2.0 * s.min_inliers))
                up = np.asarray(s.world_up, float)
                new, full, applied = fusion.step_toward(
                    current_d, projected, s.gain * confidence, s.rel_snap_step, False, up)
                out.body[child] = new
                out.body_fixed[child] = True
                body_state[child] = new
                drift[child] = round(full, 3)
                lines.append(
                    f"{child} (bone {par}): hinge {cam_hinge:.2f} deg, applied {applied:.2f} deg, "
                    f"body Δ now {fusion.angle_deg(new):.2f} deg ({n_inl} inliers)")

        report["drift_deg"] = drift
        report["inherited"] = [m for m in out.corrections if m not in drift]
        report["body_fixed"] = {m: True for m in out.body}
        report["corrections_deg"] = {m: round(fusion.angle_deg(r), 3) for m, r in out.corrections.items()}
        report["body_deg"] = {m: round(fusion.angle_deg(r), 3) for m, r in out.body.items()}
        return out
