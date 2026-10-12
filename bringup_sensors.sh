#!/usr/bin/env bash
set -Eeuo pipefail

# Start the camera and IMU nodes on a Pi.
# Usage: bringup_sensors.sh argus2
# The Pi workspace is ~/ArgusModule. Ctrl-C stops both nodes.

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 argusX" >&2
    exit 1
fi

name="$1"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/argus_common.sh"
ip=$(argus_resolve "$name")

echo "Starting camera and IMU on ${name} (${ip})"

remote=$(cat <<'EOF'
set -Eeo pipefail
source /opt/ros/humble/setup.bash
if [[ ! -f "$HOME/ArgusModule/install/setup.bash" ]]; then
    echo "Missing ~/ArgusModule/install/setup.bash on the Pi" >&2
    exit 1
fi
source "$HOME/ArgusModule/install/setup.bash"
ns="/$(hostname)"
echo "Namespace ${ns}"
set -m
launch_pids=()
stopping=0
stop_nodes() {
    [[ "$stopping" -eq 1 ]] && return
    stopping=1
    local pid
    for pid in "${launch_pids[@]}"; do
        kill -INT -"$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${launch_pids[@]}"; do
        kill -TERM -"$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap stop_nodes EXIT INT TERM HUP
ros2 run argus_bmi270 bmi270_fast_node --ros-args -r __ns:="${ns}" -p topic:=imu0 &
launch_pids+=($!)
ros2 run argus_usb_cam usb_cam_node --ros-args -r __ns:="${ns}" &
launch_pids+=($!)
wait || true
EOF
)

# Ctrl-C reaches this shell before the remote trap can run. Stop the nodes
# directly if the SSH session is torn down first.
stop_remote() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${ARGUS_SSH_USER}@${ip}" \
        "pkill -INT -x bmi270_fast_node; pkill -INT -x usb_cam_node; pkill -INT -f '[r]os2 run argus_bmi270'; pkill -INT -f '[r]os2 run argus_usb_cam'" \
        >/dev/null 2>&1 || true
}
trap stop_remote INT TERM

ssh -tt "${ARGUS_SSH_USER}@${ip}" "bash -lc $(printf '%q' "$remote")" || true
trap - INT TERM
stop_remote
