#!/usr/bin/env bash
set -Eeuo pipefail

# IMU-only Mahony orientation on a Pi. Subscribes to that Pi's imu0 and
# publishes /<hostname>/mahoney_odomimu. The first 10 s of gyroscope samples
# are averaged and removed as bias, so the module should be still at start.
# Usage: bringup_mahoney.sh argus2
# The IMU node (bringup_sensors.sh) has to already be running. Ctrl-C stops it.

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 argusX" >&2
    exit 1
fi

name="$1"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")/.." && pwd)/argus_common.sh"
ip=$(argus_resolve "$name")

echo "Starting Mahony orientation on ${name} (${ip})"

remote=$(cat <<'EOF'
set -Eeo pipefail
source /opt/ros/humble/setup.bash
if [[ ! -f "$HOME/ArgusModule/install/setup.bash" ]]; then
    echo "Missing ~/ArgusModule/install/setup.bash on the Pi" >&2
    exit 1
fi
source "$HOME/ArgusModule/install/setup.bash"
if ! ros2 pkg prefix argus_mahoney >/dev/null 2>&1; then
    echo "argus_mahoney is not built on this Pi" >&2
    exit 1
fi
ns="/$(hostname)"
if [[ -f "$HOME/ArgusModule/fastdds_unicast.xml" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/ArgusModule/fastdds_unicast.xml"
fi
echo "Mahony namespace ${ns}, imu ${ns}/imu0, odom ${ns}/mahoney_odomimu"
echo "Hold the module still for the 10 s gyroscope calibration."
exec ros2 run argus_mahoney mahoney_node --ros-args -r __ns:="${ns}"
EOF
)

stop_remote() {
    "${ARGUS_SSH[@]}" -o BatchMode=yes -o ConnectTimeout=5 "${ARGUS_SSH_USER}@${ip}" \
        "pkill -INT -x mahoney_node; pkill -INT -f '[r]os2 run argus_mahoney'" \
        >/dev/null 2>&1 || true
}
trap stop_remote INT TERM

"${ARGUS_SSH[@]}" -tt "${ARGUS_SSH_USER}@${ip}" "bash -lc $(printf '%q' "$remote")" || true
trap - INT TERM
stop_remote
