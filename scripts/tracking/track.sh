#!/usr/bin/env bash
set -Eeuo pipefail

# Track a bag in this terminal, and return when it is done:
#   start OpenVINS (or the Mahony filter) on every active module
#   record its output locally once every module's filter is up
#   play the bag on every active module
#   when playback ends, stop the recorder and the filter, then re-time the
#   recording to the module sample stamps (sync_tracked.sh)
# Usage:
#   scripts/tracking/track.sh bag-name             # OpenVINS: /<module>/odomimu_throttled -> ~/bags/<bag>_tracked
#   scripts/tracking/track.sh --mahoney bag-name   # Mahony:   /<module>/mahoney_odomimu   -> ~/bags/<bag>_mahoney_tracked
# The Mahony filter averages the first 10 s of gyroscope as bias, so the bag
# should start with the modules still.
# Ctrl-C ends playback early; the recording is still closed and re-timed.
# Re-timing is needed because each module's playback starts at a slightly
# different moment. The arrival-time original is kept as *.unsynced beside it.
# Filter and recorder output go to /tmp/argus_track_<recording>/.

filter=openvins
if [[ "${1:-}" == "--mahoney" || "${1:-}" == "--mahony" ]]; then
    filter=mahoney
    shift
fi
if [[ $# -ne 1 ]]; then
    echo "Usage: $0 [--mahoney] bag-name" >&2
    exit 1
fi

bag_name="$1"
if [[ "$bag_name" == */* || "$bag_name" == .* || "$bag_name" == *..* ]]; then
    echo "Bag name must be a single folder name, not a path" >&2
    exit 1
fi

here=$(cd "$(dirname "$0")" && pwd)
# shellcheck disable=SC1091
source "${here}/../argus_common.sh"
if ! command -v ros2 >/dev/null 2>&1; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
fi

declare -A seen=()
modules=()
topics=()
ready_topics=()
while read -r name ip; do
    [[ -n "$name" ]] || continue
    [[ -n "${seen[$name]:-}" ]] && continue
    # Same rule as all_active.sh, so the recorded topics match the modules that play.
    if ping -c 3 -i 0.2 -W 1 "$ip" >/dev/null 2>&1; then
        seen[$name]=1
        modules+=("$name")
        if [[ "$filter" == mahoney ]]; then
            topics+=("/${name}/mahoney_odomimu")
            ready_topics+=("/${name}/mahoney_odomimu")
        else
            topics+=("/${name}/odomimu_throttled")
            # The estimator's own output; the relay's topic appears before the estimator is up.
            ready_topics+=("/${name}/odomimu")
        fi
    fi
done < <(argus_each_host)

if [[ ${#topics[@]} -eq 0 ]]; then
    echo "No active argus modules" >&2
    exit 1
fi

if [[ "$filter" == mahoney ]]; then
    tracked_name="${bag_name}_mahoney_tracked"
else
    tracked_name="${bag_name}_tracked"
fi
out="${HOME}/bags/${tracked_name}"
mkdir -p "${HOME}/bags"
if [[ -e "$out" ]]; then
    echo "${out} already exists" >&2
    exit 1
fi
logs="/tmp/argus_track_${tracked_name}"
mkdir -p "$logs"

interrupted=0
trap 'interrupted=1' INT

filter_pid=""
record_pid=""
play_pid=""

# Background jobs started under job control get their own process group and
# keep SIGINT, which the recorder needs to close the bag. Job control is off
# again afterwards so Ctrl-C reaches this script, not whatever runs in front.
start_job() {
    set -m
    "$@" </dev/null &
    set +m
}

# Signal a job's process group, wait for it, then kill whatever is left.
stop_job() {
    local pid=$1 signal=$2 seconds=$3 i
    [[ -n "$pid" ]] || return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill "-${signal}" -- "-${pid}" 2>/dev/null || true
        for ((i = 0; i < seconds * 2; i++)); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.5 || true
        done
        kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}

wait_for_topics() {
    local deadline=$((SECONDS + 90)) listed topic
    local -a missing=("${ready_topics[@]}")
    while (( SECONDS < deadline )) && [[ "$interrupted" -eq 0 ]]; do
        if ! kill -0 "$filter_pid" 2>/dev/null; then
            echo "${filter} stopped during startup; see ${logs}/${filter}.log" >&2
            return 1
        fi
        # The modules announce themselves every few seconds over unicast.
        listed=$(ros2 topic list --no-daemon --spin-time 4 2>/dev/null || true)
        missing=()
        for topic in "${ready_topics[@]}"; do
            grep -qxF -- "$topic" <<<"$listed" || missing+=("$topic")
        done
        [[ ${#missing[@]} -eq 0 ]] && return 0
        sleep 1 || true
    done
    [[ "$interrupted" -eq 0 ]] && echo "Timed out waiting for: ${missing[*]}" >&2
    return 1
}

echo "Tracking ${bag_name} with ${filter} on ${modules[*]}"
echo "Logs in ${logs}"
start_job "${ARGUS_SCRIPTS}/all_active.sh" "$filter" >"${logs}/${filter}.log" 2>&1
filter_pid=$!

status=0
if wait_for_topics; then
    start_job ros2 bag record -o "$out" "${topics[@]}" >"${logs}/record.log" 2>&1
    record_pid=$!
    # Let the recorder subscribe before the first sample arrives.
    sleep 3 || true
fi
if [[ -n "$record_pid" && "$interrupted" -eq 0 ]]; then
    echo "Playing ${bag_name}"
    start_job "${ARGUS_SCRIPTS}/all_active.sh" play "$bag_name"
    play_pid=$!
    while :; do
        rc=0
        wait "$play_pid" 2>/dev/null || rc=$?
        # wait also returns when Ctrl-C arrives while playback is still running.
        kill -0 "$play_pid" 2>/dev/null || break
        if [[ "$interrupted" -eq 1 ]]; then
            echo
            echo "Stopping playback"
            stop_job "$play_pid" TERM 20
            break
        fi
    done
    [[ "$rc" -eq 0 || "$interrupted" -eq 1 ]] || status=1
    # The filters are a few samples behind the playback.
    [[ "$interrupted" -eq 1 ]] || sleep 3 || true
elif [[ -z "$record_pid" ]]; then
    status=1
fi

# Ctrl-C from here on would cut the wrap-up short.
trap '' INT
if [[ -n "$record_pid" ]]; then
    echo "Closing ${tracked_name}"
    stop_job "$record_pid" INT 30
fi
echo "Stopping ${filter}"
stop_job "$filter_pid" TERM 30
trap - INT

if [[ -f "${out}/metadata.yaml" ]]; then
    echo "Re-timing ${tracked_name} to the module sample stamps..."
    if ! "${here}/sync_tracked.sh" "$out"; then
        echo "sync_tracked.sh failed; the recording is unchanged" >&2
        status=1
    fi
    empty=$(awk '$1 == "-" && $2 == "topic_metadata:" { t = "" }
                 $1 == "name:" { t = $2 }
                 $1 == "message_count:" && t != "" { if ($2 == 0) print t; t = "" }' "${out}/metadata.yaml")
    if [[ -n "$empty" ]]; then
        echo "No messages recorded from: $(tr '\n' ' ' <<<"$empty")" >&2
        echo "Check the play output above and ${logs}/${filter}.log" >&2
        status=1
    fi
elif [[ -n "$record_pid" ]]; then
    echo "No recording was written; see ${logs}/record.log" >&2
    status=1
fi

if [[ "$interrupted" -eq 1 ]]; then
    exit 130
fi
exit "$status"
