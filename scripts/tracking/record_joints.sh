#!/usr/bin/env bash
set -Eeuo pipefail

# Record the joint angles argus_server publishes, on this machine.
# Usage:
#   record_joints.sh                # ~/bags/joints_<time>
#   record_joints.sh lift_trial     # ~/bags/lift_trial_joints
#   record_joints.sh --poses NAME   # also record the module poses the server uses
# Topics: /joint_states (sensor_msgs/JointState, angles in radians), plus
# /reset_tracking and /load_zero_pose so the bag shows when tracking was zeroed
# and against which pose. The server is not started here. Ctrl-C stops and
# closes the bag.

with_poses=0
if [[ "${1:-}" == "--poses" ]]; then
    with_poses=1
    shift
fi
if [[ $# -gt 1 ]]; then
    echo "Usage: $0 [--poses] [name]" >&2
    exit 1
fi
name="${1:-}"
if [[ -n "$name" && ( "$name" == */* || "$name" == .* || "$name" == *..* ) ]]; then
    echo "Name must be a single folder name, not a path" >&2
    exit 1
fi
if [[ -n "$name" ]]; then
    out="${HOME}/bags/${name}_joints"
else
    out="${HOME}/bags/joints_$(date +%Y%m%d_%H%M%S)"
fi
if [[ -e "$out" ]]; then
    echo "${out} already exists" >&2
    exit 1
fi
mkdir -p "${HOME}/bags"

# ROS setup scripts read unset variables.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

topics=(/joint_states /reset_tracking /load_zero_pose)
if [[ "$with_poses" -eq 1 ]]; then
    for module in argus0 argus1 argus2 argus3 argus4; do
        topics+=("/${module}/odomimu_throttled")
    done
fi

publishers=$(timeout 5 ros2 topic info /joint_states 2>/dev/null | awk '/Publisher count/ {print $3}' || true)
if [[ "${publishers:-0}" == "0" ]]; then
    echo "Note: nothing is publishing /joint_states yet. Recording anyway; start argus_server when ready."
fi

echo "Recording ${topics[*]}"
echo "  to ${out}"
echo "Ctrl-C to stop."
exec ros2 bag record -o "$out" "${topics[@]}"
