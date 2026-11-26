"""Per-module drift corrections from camera-pair rotations.

Notation (all rotations map the named child frame into the parent frame):
  P_m   camera m in ground as the server believes it without correction:
        R_ref_m * R_cur_m * R_imu_cam_m (R_ref from the server's zero,
        R_cur the module's tracked IMU orientation at the frame time)
  V_ab  camera b in camera a, measured from the images
  G_m   rotation in ground that makes the belief match the images:
        (G_a P_a)^T (G_b P_b) = V_ab, with G fixed to identity on the anchor
        module (it defines ground; its own heading is not observable).

G also contains whatever the zero left inconsistent between cameras (mount
and zero-pose errors), which is not drift. So right after a zero each module's
baseline B_m = W_m G_m^-1 is recorded (W_m the correction the server applies
then), and the correction wanted later is W_m = B_m G_m: only the change
since the zero.

Modules whose cameras connect to the anchor keep W in ground (left-multiply).
A floating group (no path to the anchor) only observes relative rotations;
holding that G as a world-fixed W ratchets into distal joints as the limb
moves. Those modules store a body-fixed Δ = R_imu^-1 G R_imu (right-multiply
on R_cur) so the leftover stays in the IMU, not in ground.
"""

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation as R


def angle_deg(rotation):
    return float(np.degrees(np.linalg.norm(rotation.as_rotvec())))


def mean_rotation(rotations, weights=None):
    return R.concatenate(rotations).mean(weights=weights)


def connected(anchor, edges):
    """Modules linked to the anchor through measured pairs, in breadth-first order."""
    order, frontier = [anchor], deque([anchor])
    while frontier:
        node = frontier.popleft()
        for a, b, _v, _w in edges:
            other = b if a == node else a if b == node else None
            if other is not None and other not in order:
                order.append(other)
                frontier.append(other)
    return order


def _candidates(module, G, P, edges):
    rotations, weights = [], []
    for a, b, V, w in edges:
        if b == module and a in G:
            rotations.append(G[a] * P[a] * V * P[b].inv())
        elif a == module and b in G:
            rotations.append(G[b] * P[b] * V.inv() * P[a].inv())
        else:
            continue
        weights.append(w)
    return rotations, weights


def solve_ground(anchor, P, edges, sweeps=10):
    """G for every module connected to the anchor (anchor identity)."""
    order = connected(anchor, edges)
    G = {anchor: R.identity()}
    for module in order[1:]:
        rotations, weights = _candidates(module, G, P, edges)
        G[module] = mean_rotation(rotations, weights)
    for _ in range(sweeps):
        for module in order[1:]:
            rotations, weights = _candidates(module, {k: v for k, v in G.items() if k != module}, P, edges)
            G[module] = mean_rotation(rotations, weights)
    return G


def edge_residuals(G, P, edges):
    """Disagreement (deg) of each edge with the solved G."""
    out = []
    for a, b, V, _w in edges:
        if a in G and b in G:
            out.append(angle_deg(((G[a] * P[a]).inv() * (G[b] * P[b])).inv() * V))
        else:
            out.append(float("nan"))
    return out


def solve_robust(anchor, P, edges, outlier_deg=3.0):
    """solve_ground, dropping the worst edge while it disagrees by more than outlier_deg
    and the remaining edges still over-determine its modules."""
    edges = list(edges)
    while True:
        G = solve_ground(anchor, P, edges)
        residuals = edge_residuals(G, P, edges)
        worst = int(np.nanargmax(residuals)) if edges and np.any(np.isfinite(residuals)) else None
        if worst is None or residuals[worst] <= outlier_deg:
            return G, edges, residuals
        trial = edges[:worst] + edges[worst + 1:]
        if set(connected(anchor, trial)) != set(connected(anchor, edges)):
            return G, edges, residuals
        edges = trial


def components(edges):
    """Groups of modules linked by edges."""
    groups = []
    for a, b, _v, _w in edges:
        hit = [g for g in groups if a in g or b in g]
        merged = {a, b}.union(*hit) if hit else {a, b}
        groups = [g for g in groups if g not in hit] + [merged]
    return groups


def solve_groups(anchor, priority, P, edges, outlier_deg=3.0):
    """solve_robust for each linked group of modules. A group with the anchor is solved with the
    anchor fixed; another group with its highest-priority module fixed (a local gauge: only the
    rotations within that group are observed). Returns (G, gauge per module, kept edges)."""
    G, gauge, kept = {}, {}, []
    for group in components(edges):
        local = anchor if anchor in group else max(sorted(group), key=lambda m: priority.get(m, 0.0))
        group_edges = [e for e in edges if e[0] in group]
        solved, group_kept, _r = solve_robust(local, P, group_edges, outlier_deg)
        for module, rotation in solved.items():
            G[module] = rotation
            gauge[module] = local
        kept += group_kept
    return G, gauge, kept


def as_body(G, R_imu):
    """Ground-frame G expressed in the IMU: Δ = R_imu^{-1} G R_imu."""
    return R_imu.inv() * G * R_imu


def twist(rotation, axis):
    """The part of a rotation about an axis (swing-twist decomposition)."""
    q = rotation.as_quat()
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    projected = np.dot(q[:3], axis) * axis
    t = np.r_[projected, q[3]]
    n = np.linalg.norm(t)
    if n < 1e-9:
        return R.identity()
    return R.from_quat(t / n)


def signed_yaw_deg(rotation, up):
    """Signed angle (deg) of `rotation`'s twist about `up`, wrapped to (-180, 180]."""
    up = np.asarray(up, float)
    up = up / np.linalg.norm(up)
    angle = float(np.degrees(np.dot(twist(rotation, up).as_rotvec(), up)))
    return (angle + 180.0) % 360.0 - 180.0


def yaw_about(up, deg):
    up = np.asarray(up, float)
    return R.from_rotvec(up / np.linalg.norm(up) * np.radians(deg))


def yaw_only_relative(belief_a, belief_b, measured_Rab, world_up):
    """Heading residual of a measured R_a_b vs the believed relative pose.

    belief_* are camera orientations in ground; measured_Rab takes camera-b
    coordinates into camera-a. Returns (V, yaw_deg) where yaw_deg is the signed
    heading residual (camera-a frame, about gravity) and V is belief_rel composed
    with that yaw. Used for gating; the multi-camera solve should keep the full
    measured rotation and project each solved G to world yaw instead.
    """
    Rab_belief = belief_a.inv() * belief_b
    up_a = belief_a.inv().apply(np.asarray(world_up, float))
    yaw = signed_yaw_deg(Rab_belief.inv() * measured_Rab, up_a)
    return Rab_belief * yaw_about(up_a, yaw), yaw


def step_toward(current, target, gain, max_step_deg, yaw_only, up):
    """Move the correction part of the way to the target. Returns (new, full change deg, applied deg).

    With yaw_only, only the shortest rotation about `up` (gravity in this frame) is applied,
    so a 180 deg essential-matrix flip becomes a few degrees the other way, not a walk around.
    """
    delta = target * current.inv()
    if yaw_only:
        delta = yaw_about(up, signed_yaw_deg(delta, up))
    full = angle_deg(delta)
    rotvec = delta.as_rotvec() * gain
    applied = np.degrees(np.linalg.norm(rotvec))
    if applied > max_step_deg:
        rotvec *= max_step_deg / applied
        applied = max_step_deg
    return R.from_rotvec(rotvec) * current, full, float(applied)
