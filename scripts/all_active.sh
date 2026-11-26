#!/usr/bin/env bash
set -Eeuo pipefail

# Run one module script on every argus host that answers ping
# on the network this computer is using.
# Usage:
#   all_active.sh bringup
#   all_active.sh openvins
#   all_active.sh mahoney
#   all_active.sh record [bag-name]
#   all_active.sh play [bag-name]
#   all_active.sh fetch [bag-name]
#   all_active.sh clear [bag-name]
#   all_active.sh compress [bag-name ...]
#   all_active.sh decompress bag-name [bag-name ...]
#   all_active.sh cook bag-name [--delete-original]
#   all_active.sh pull
#   all_active.sh remove_uncooked [--yes]
# Ctrl-C stops every host still running.

here=$(cd "$(dirname "$0")" && pwd)
# shellcheck disable=SC1091
source "${here}/argus_common.sh"

usage() {
    echo "Usage: $0 bringup | openvins | mahoney | record [bag-name] | play [bag-name] | fetch [bag-name] | clear [bag-name] | compress [bag-name ...] | decompress bag-name [bag-name ...] | cook bag-name [--delete-original] | pull | remove_uncooked [--yes]" >&2
    exit 1
}

if [[ $# -lt 1 ]]; then
    usage
fi

case "$1" in
    bringup|bringup_sensors|bringup_sensors.sh)
        script="${here}/module/bringup_sensors.sh"
        shift
        [[ $# -eq 0 ]] || usage
        ;;
    openvins|bringup_openvins|bringup_openvins.sh)
        script="${here}/module/bringup_openvins.sh"
        shift
        [[ $# -eq 0 ]] || usage
        ;;
    mahoney|mahony|bringup_mahoney|bringup_mahoney.sh)
        script="${here}/module/bringup_mahoney.sh"
        shift
        [[ $# -eq 0 ]] || usage
        ;;
    record|record_bag|record_bag.sh)
        script="${here}/bags/record_bag.sh"
        shift
        [[ $# -le 1 ]] || usage
        ;;
    play|play_bag|play_bag.sh)
        script="${here}/bags/play_bag.sh"
        shift
        [[ $# -le 1 ]] || usage
        ;;
    fetch|fetch_bags|fetch_bags.sh)
        script="${here}/bags/fetch_bags.sh"
        shift
        [[ $# -le 1 ]] || usage
        ;;
    clear|clear_bags|clear_bags.sh)
        script="${here}/bags/clear_bags.sh"
        shift
        [[ $# -le 1 ]] || usage
        ;;
    compress|compress_bags|compress_bags.sh)
        script="${here}/bags/compress_bags.sh"
        shift
        ;;
    decompress|decompress_bags|decompress_bags.sh)
        script="${here}/bags/decompress_bags.sh"
        shift
        [[ $# -ge 1 ]] || usage
        ;;
    cook|cook_bag|cook_bag.sh)
        script="${here}/bags/cook_bag.sh"
        shift
        [[ $# -eq 1 || ( $# -eq 2 && "$2" == "--delete-original" ) ]] || usage
        ;;
    pull|pull_module|pull_module.sh)
        script="${here}/module/pull_module.sh"
        shift
        [[ $# -eq 0 ]] || usage
        ;;
    remove_uncooked|remove_uncooked.sh)
        script="${here}/bags/remove_uncooked.sh"
        shift
        [[ $# -eq 0 || ( $# -eq 1 && "$1" == "--yes" ) ]] || usage
        ;;
    *)
        usage
        ;;
esac

declare -A seen=()
active=()
skipped=()
while read -r name ip; do
    [[ -n "$name" ]] || continue
    [[ -n "${seen[$name]:-}" ]] && continue
    # 3 pings; any reply counts. One lost packet on Wi-Fi must not drop a module.
    if ping -c 3 -i 0.2 -W 1 "$ip" >/dev/null 2>&1; then
        seen[$name]=1
        active+=("$name")
    else
        skipped+=("$name ($ip)")
    fi
done < <(argus_each_host)

if [[ ${#skipped[@]} -gt 0 ]]; then
    echo "Skipping, no ping reply: ${skipped[*]}" >&2
fi
if [[ ${#active[@]} -eq 0 ]]; then
    echo "No active argus modules" >&2
    exit 1
fi

echo "Running $(basename "$script") on ${active[*]}"

# Own process group per host. A background script ignores SIGINT, so Ctrl-C
# must TERM that group or this shell waits on the SSH session forever.
set -m
pids=()
for name in "${active[@]}"; do
    "$script" "$name" "$@" </dev/null &
    pids+=("$!")
done

stopping=0
stop_all() {
    [[ "$stopping" -eq 1 ]] && return
    stopping=1
    echo
    echo "Stopping ${#pids[@]} modules"
    local pid i live loops
    # Recording has to receive SIGINT and finish metadata.yaml. Signal the
    # script itself and give it time. Other commands stop with the process group.
    if [[ "$(basename "$script")" == "record_bag.sh" ]]; then
        for pid in "${pids[@]}"; do
            kill -TERM "$pid" 2>/dev/null || true
        done
        loops="1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60"
    else
        for pid in "${pids[@]}"; do
            kill -TERM -"$pid" 2>/dev/null || true
        done
        loops="1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24"
    fi
    for i in $loops; do
        live=0
        for pid in "${pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                live=1
                break
            fi
        done
        [[ "$live" -eq 0 ]] && break
        sleep 0.5
    done
    for pid in "${pids[@]}"; do
        kill -KILL -"$pid" 2>/dev/null || true
    done
    wait || true
    exit 130
}
trap stop_all INT TERM

fail=0
for pid in "${pids[@]}"; do
    wait "$pid" || fail=1
done
exit "$fail"
