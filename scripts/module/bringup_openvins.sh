#!/usr/bin/env bash
set -Eeuo pipefail

# Start OpenVINS on a Pi, subscribed to that Pi's namespaced camera and IMU.
# Also starts argus_odom_throttle, which republishes <ns>/odomimu at 50 Hz as
# <ns>/odomimu_throttled for the IK server, so the full IMU-rate stream stays off Wi-Fi.
# Usage: bringup_openvins.sh argus2
# Config: ~/openvins_ws/src/open_vins/config/argus/estimator_config.yaml
# Ctrl-C stops the estimator and the relay.

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 argusX" >&2
    exit 1
fi

name="$1"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")/.." && pwd)/argus_common.sh"
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
# Unicast discovery to the hotspot host; Wi-Fi multicast is unreliable.
if [[ -f "$HOME/ArgusModule/fastdds_unicast.xml" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/ArgusModule/fastdds_unicast.xml"
fi
relay="$HOME/ArgusModule/install/argus_odom_throttle/lib/argus_odom_throttle/odom_throttle"
if [[ -x "$relay" ]]; then
    source "$HOME/ArgusModule/install/setup.bash"
    "$relay" --ros-args -r __ns:="${ns}" &
else
    echo "argus_odom_throttle is not built on this Pi; ${ns}/odomimu_throttled will be missing"
fi
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

# Process names are truncated to 15 characters: run_subscribe_msckf is run_subscribe_m.
stop_remote() {
    "${ARGUS_SSH[@]}" -o BatchMode=yes -o ConnectTimeout=5 "${ARGUS_SSH_USER}@${ip}" \
        "pkill -INT -x run_subscribe_m; pkill -INT -f '[r]os2 run ov_msckf'; pkill -INT -x odom_throttle" \
        >/dev/null 2>&1 || true
}
trap stop_remote INT TERM

"${ARGUS_SSH[@]}" -tt "${ARGUS_SSH_USER}@${ip}" "bash -lc $(printf '%q' "$remote")" || true
trap - INT TERM
stop_remote
