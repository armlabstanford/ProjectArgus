#!/usr/bin/env bash
set -Eeuo pipefail

# Play a bag from ~/bags/ on a Pi.
# Usage: play_bag.sh argus2 [bag-name]
# With no bag name, the newest folder in ~/bags is played.
# Topics stay as recorded, for example /argus2/imu0.

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 argusX [bag-name]" >&2
    exit 1
fi

name="$1"
bag_name="${2:-}"
if [[ -n "$bag_name" && ( "$bag_name" == */* || "$bag_name" == .* || "$bag_name" == *..* ) ]]; then
    echo "Bag name must be a single folder name, not a path" >&2
    exit 1
fi
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/argus_common.sh"
ip=$(argus_resolve "$name")

if [[ -n "$bag_name" ]]; then
    echo "Playing ~/bags/${bag_name} on ${name} (${ip})"
else
    echo "Playing the newest bag on ${name} (${ip})"
fi

remote=$(cat <<'EOF'
set -Eeo pipefail
source /opt/ros/humble/setup.bash
if [[ -n "${BAG_NAME}" ]]; then
    bag_dir="$HOME/bags/${BAG_NAME}"
else
    bag_dir=$(find "$HOME/bags" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')
fi
if [[ -z "${bag_dir}" || ! -f "${bag_dir}/metadata.yaml" ]]; then
    echo "No playable bag${BAG_NAME:+ at ~/bags/${BAG_NAME}}" >&2
    exit 1
fi
echo "Playing ${bag_dir}"
exec ros2 bag play "${bag_dir}"
EOF
)

stop_remote() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${ARGUS_SSH_USER}@${ip}" \
        "pkill -INT -f '[b]in/ros2 bag play'" \
        >/dev/null 2>&1 || true
}
trap stop_remote INT TERM

ssh -tt "${ARGUS_SSH_USER}@${ip}" "BAG_NAME=$(printf '%q' "$bag_name") bash -lc $(printf '%q' "$remote")" || true
trap - INT TERM
stop_remote
