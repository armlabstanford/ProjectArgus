#!/usr/bin/env bash
set -Eeuo pipefail

# Start OpenVINS on a Pi, subscribed to that Pi's namespaced camera and IMU.
# Usage: bringup_openvins.sh argus2
# Config: ~/openvins_ws/src/open_vins/config/argus/estimator_config.yaml
# Ctrl-C stops the estimator.

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 argusX" >&2
    exit 1
fi

name="$1"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/argus_common.sh"
ip=$(argus_resolve "$name")

echo "Starting OpenVINS on ${name} (${ip})"

remote=$(cat <<'EOF'
set -Eeo pipefail
source /opt/ros/humble/setup.bash
if [[ ! -f "$HOME/openvins_ws/install/setup.bash" ]]; then
    echo "Missing ~/openvins_ws/install/setup.bash on the Pi" >&2
    exit 1
fi
source "$HOME/openvins_ws/install/setup.bash"
config="$HOME/openvins_ws/src/open_vins/config/argus/estimator_config.yaml"
if [[ ! -f "$config" ]]; then
    echo "Missing ${config} on the Pi" >&2
    exit 1
fi
ns="/$(hostname)"
echo "OpenVINS namespace ${ns}, topics ${ns}/imu0 and ${ns}/cam0/image_raw"
exec ros2 run ov_msckf run_subscribe_msckf --ros-args \
    -r __ns:="${ns}" \
    -p config_path:="${config}" \
    -p use_stereo:=false \
    -p max_cameras:=1 \
    -p imu0_rostopic:="${ns}/imu0" \
    -p cam0_rostopic:="${ns}/cam0/image_raw"
EOF
)

stop_remote() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${ARGUS_SSH_USER}@${ip}" \
        "pkill -INT -x run_subscribe_msckf; pkill -INT -f '[r]os2 run ov_msckf'" \
        >/dev/null 2>&1 || true
}
trap stop_remote INT TERM

ssh -tt "${ARGUS_SSH_USER}@${ip}" "bash -lc $(printf '%q' "$remote")" || true
trap - INT TERM
stop_remote
