#!/usr/bin/env bash
set -Eeuo pipefail

# Record /$(hostname)/imu0 and /$(hostname)/cam0/image_raw on a Pi into ~/bags/.
# Usage: record_bag.sh argus2 [bag-name]
# With no bag name, the folder is the current time. Ctrl-C closes the bag.

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 argusX [bag-name]" >&2
    exit 1
fi

name="$1"
bag_name="${2:-$(date +%Y%m%d_%H%M%S)}"
if [[ "$bag_name" == */* || "$bag_name" == .* || "$bag_name" == *..* ]]; then
    echo "Bag name must be a single folder name, not a path" >&2
    exit 1
fi
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/argus_common.sh"
ip=$(argus_resolve "$name")

echo "Recording on ${name} (${ip}) to ~/bags/${bag_name}"

remote=$(cat <<'EOF'
set -Eeo pipefail
source /opt/ros/humble/setup.bash
if [[ ! -f "$HOME/ArgusModule/install/setup.bash" ]]; then
    echo "Missing ~/ArgusModule/install/setup.bash on the Pi" >&2
    exit 1
fi
source "$HOME/ArgusModule/install/setup.bash"
ns="/$(hostname)"
mkdir -p "$HOME/bags"
echo "Writing ~/bags/${BAG_NAME} from ${ns}/imu0 and ${ns}/cam0/image_raw"
# Foreground. A background job on this SSH tty is stopped at once (SIGTTIN)
# and never writes metadata.yaml.
ros2 bag record -o "$HOME/bags/${BAG_NAME}" "${ns}/imu0" "${ns}/cam0/image_raw"
EOF
)

# Ctrl-C from the mass runner signals this shell. Ask the recorder to exit
# before SSH drops, so it can write metadata.yaml.
stopping=0
ssh_pid=
finish_bag() {
    [[ "$stopping" -eq 1 ]] && return
    stopping=1
    ssh -o BatchMode=yes -o ConnectTimeout=5 "${ARGUS_SSH_USER}@${ip}" \
        "pkill -INT -f '[b]in/ros2 bag record'" >/dev/null 2>&1 || true
    if [[ -n "${ssh_pid}" ]]; then
        local i
        for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 \
                 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40; do
            kill -0 "$ssh_pid" 2>/dev/null || return
            sleep 0.5
        done
        kill -TERM "$ssh_pid" 2>/dev/null || true
        wait "$ssh_pid" 2>/dev/null || true
    fi
}
trap finish_bag INT TERM

ssh -tt "${ARGUS_SSH_USER}@${ip}" "BAG_NAME=$(printf '%q' "$bag_name") bash -lc $(printf '%q' "$remote")" </dev/null &
ssh_pid=$!
wait "$ssh_pid" || true
trap - INT TERM
