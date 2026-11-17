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

Modules linked to the anchor (root body) are corrected relative to it. A
group of modules linked only to each other (one arm) can observe relative
rotations within the group, but not the group's common heading. With
yaw_only (heading drift), those floating groups are skipped: applying a
yaw step there would move the unobservable gauge freedom and ratchet.
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
    gate_deg: float = 75.0
    # Hard cap: reject a pair whose measured relative rotation disagrees with the
    # pose belief by more than this, even with many inliers (high-inlier wrong
    # matches were walking heading corrections up tens of degrees).
    max_off_deg: float = 15.0
    outlier_deg: float = 8.0
    baseline_samples: int = 2
    gain: float = 0.7
    max_step: float = 12.0
    yaw_only: bool = True
    world_up: tuple = (0.0, 0.0, 1.0)
    group_gauges: bool = True


@dataclass
class CycleResult:
    corrections: dict = field(default_factory=dict)   # module -> new W (ground)
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


class DriftEstimator:
    def __init__(self, cameras, anchor, settings=None, priority=None, adjacent_pairs=None):
        self.cameras = cameras
        self.anchor = anchor
        self.s = settings or Settings()
        self.priority = dict(priority or {})    # module -> weight, picks a group's fixed module
        # None: match every pair. Set: only these frozenset({a, b}) module pairs.
        self.adjacent_pairs = adjacent_pairs

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

    def _predicted(self, module, baseline, current):
        """G the belief implies now (identity before the module has a baseline)."""
        if module == self.anchor:
            return R.identity()
        base = self._baseline_for(module, baseline)
        if base is not None:
            return base.inv() * current.get(module, R.identity())
        return R.identity()

    def cycle(self, images, pose_at, reference, current, baseline, draw=False):
        """images: module -> (header time, gray image); pose_at(module, t) -> (R, deg/s) or None;
        reference: module -> R_ref; current: module -> W the server applies; baseline as returned before."""
        s = self.s
        out = CycleResult(baseline=dict(baseline))
        lines, report = out.lines, out.report
        if images:
            times = [t for t, _ in images.values()]
            report["spread_ms"] = round(1000 * (max(times) - min(times)), 1)
            report["frame_times"] = {m: t for m, (t, _image) in images.items()}

        P, grays, cameras, skipped, speeds = {}, {}, {}, {}, {}
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
            P[module] = reference[module] * orientation * camera.imu_cam
            grays[module] = image
            cameras[module] = camera.for_size(image.shape[1], image.shape[0])
        report["skipped"] = skipped
        report["speeds"] = speeds
        for module, why in skipped.items():
            lines.append(f"{module}: skipped ({why})")

        predicted = {m: self._predicted(m, out.baseline, current) for m in P}
        orb = vision.make_orb(s.nfeatures)
        edges, pairs_report = [], []
        names = sorted(P)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if self.adjacent_pairs is not None and frozenset((a, b)) not in self.adjacent_pairs:
                    continue
                belief_a, belief_b = predicted[a] * P[a], predicted[b] * P[b]
                m, views = vision.guided_rotation(grays[a], cameras[a], belief_a, grays[b], cameras[b], belief_b,
                                                  orb, s.px_threshold, s.search_deg)
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
                    # Heading-only: a ~180 deg flip is not yaw drift. Reject large
                    # yaw disagreements even with many inliers; soft gate_deg still
                    # catches mid-sized offs when support is thin.
                    if accepted and off > 90.0:
                        accepted, m.reason = False, f"{off:.1f} deg yaw from belief (flip)"
                    elif accepted and off > s.max_off_deg:
                        accepted, m.reason = False, f"{off:.1f} deg yaw from belief"
                    elif accepted and off > s.gate_deg and m.inliers < 2 * s.min_inliers:
                        accepted, m.reason = False, f"{off:.1f} deg yaw from belief"
                    elif not accepted:
                        m.reason = f"{m.inliers} inliers of {m.matches}"
                entry["accepted"] = accepted
                if not accepted:
                    entry["reason"] = m.reason
                pairs_report.append(entry)
                if accepted:
                    weight = min(1.0, m.inliers / (2.0 * s.min_inliers)) * min(1.0, m.ratio / 0.6)
                    edges.append((a, b, pair_rotation, weight))
                if draw and m.ok and views is not None:
                    va, vb, fa, fb = views
                    title = f"{a}-{b} {m.model} {m.inliers}/{m.matches} " + ("ok" if accepted else m.reason)
                    out.drawings.append((f"{a}_{b}", vision.draw(va, vb, fa, fb, m, title)))
        report["pairs"] = pairs_report
        for entry in pairs_report:
            lines.append(f"{entry['pair']}: {entry['inliers']}/{entry['matches']} {entry.get('model', '')} "
                         + ("accepted" if entry["accepted"] else f"rejected ({entry.get('reason', '')})"))

        if not edges:
            lines.append("no confident camera pairs; no correction")
            return out
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
        # G is a ground-frame correction. Heading drift is yaw about world up;
        # drop pitch/roll that match noise put into the relative rotations.
        if s.yaw_only:
            up_ground = np.asarray(s.world_up, float)
            G = {m: fusion.yaw_about(up_ground, fusion.signed_yaw_deg(g, up_ground))
                 for m, g in G.items()}
        drift = {}
        for module, g in G.items():
            fixed = gauge[module]
            if module == fixed:
                continue
            # A floating group (no path to the anchor) cannot observe common
            # heading. yaw_only would step that gauge freedom and ratchet
            # (forearm vs upper-arm pairs were walking tens of degrees).
            if s.yaw_only and fixed != self.anchor:
                lines.append(f"{module} (to {fixed}): skipped (heading not observable without {self.anchor})")
                continue
            key = f"{module}@{fixed}"
            w = current.get(module, R.identity())
            base = out.baseline.get(key)
            if not isinstance(base, R):
                samples = (base or []) + [w * g.inv()]
                if len(samples) >= s.baseline_samples:
                    out.baseline[key] = fusion.mean_rotation(samples)
                    lines.append(f"{module} (to {fixed}): baseline set from {len(samples)} captures")
                else:
                    out.baseline[key] = samples
                    lines.append(f"{module} (to {fixed}): baseline {len(samples)}/{s.baseline_samples}")
                continue
            confidence = max(wt for a, b, _v, wt in kept if module in (a, b))
            # Drift turns the module's own world about its vertical, which the zero maps into ground.
            up = reference[module].apply(np.asarray(s.world_up, float))
            new, full, applied = fusion.step_toward(w, base * g, s.gain * confidence, s.max_step, s.yaw_only, up)
            out.corrections[module] = new
            drift[module] = round(full, 3)
            lines.append(f"{module} (to {fixed}): drift {full:.2f} deg, applied {applied:.2f} deg, "
                         f"correction now {fusion.angle_deg(new):.2f} deg")
        report["drift_deg"] = drift
        report["corrections_deg"] = {m: round(fusion.angle_deg(r), 3) for m, r in out.corrections.items()}
        return out
