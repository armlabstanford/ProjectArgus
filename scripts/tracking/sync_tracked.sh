#!/usr/bin/env bash
set -Eeuo pipefail

# Point rosbag playback time at each message header stamp.
# Usage: sync_tracked.sh [bag-dir ...]
# With no arguments, every directory in ~/bags matching *_tracked.
# Playback follows the sqlite timestamp, so separately started module plays
# show up offset inside one tracked bag. The header stamp is the module clock.
# The sqlite time is rewritten so the modules share one timeline. Differences
# of a couple of minutes or less are kept, because those are real start-time
# offsets. A larger clock gap is removed, with a warning, so the bag stays the
# length of the recording instead of stretching across the bad clocks. The
# original db3 and metadata.yaml are kept beside them with a .unsynced suffix.

if [[ $# -eq 0 ]]; then
    shopt -s nullglob
    bags=()
    for dir in "${HOME}/bags"/*_tracked; do
        [[ -d "$dir" ]] || continue
        bags+=("$dir")
    done
    shopt -u nullglob
else
    bags=()
    for arg in "$@"; do
        if [[ -d "$arg" ]]; then
            bags+=("$(cd "$arg" && pwd)")
        elif [[ -d "${HOME}/bags/${arg}" ]]; then
            bags+=("${HOME}/bags/${arg}")
        else
            echo "No bag directory: ${arg}" >&2
            exit 1
        fi
    done
fi

if [[ ${#bags[@]} -eq 0 ]]; then
    echo "No tracked bags to correct" >&2
    exit 1
fi

python3 - "${bags[@]}" << 'PY'
import os
import re
import shutil
import sqlite3
import statistics
import subprocess
import sys

# A start-time offset is about a second. A gap beyond a couple of minutes is a
# different module clock and is removed so the streams still overlap.
MAX_CLOCK_GAP_NS = 120 * 1_000_000_000
HEADER_TYPES = {
    "nav_msgs/msg/Odometry",
    "nav_msgs/msg/Path",
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseWithCovarianceStamped",
    "geometry_msgs/msg/TwistStamped",
}


def die(message):
    print(message, file=sys.stderr)
    sys.exit(1)


def header_stamp(blob):
    if len(blob) < 12 or blob[0] != 0 or blob[1] not in (0, 1):
        return None
    endian = "little" if blob[1] == 1 else "big"
    sec = int.from_bytes(blob[4:8], endian, signed=True)
    nsec = int.from_bytes(blob[8:12], endian, signed=False)
    if sec <= 0 or nsec >= 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nsec


def metadata_files(text):
    files = []
    in_files = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped == "files:":
            in_files = True
            continue
        if in_files and "path:" in stripped:
            files.append(stripped.split("path:", 1)[1].strip())
    return files


def read_messages(db_path):
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        topics = {
            topic_id: (topic_name, topic_type)
            for topic_id, topic_name, topic_type in con.execute(
                "SELECT id, name, type FROM topics"
            )
        }
        rows = []
        for message_id, topic_id, stored, blob in con.execute(
            "SELECT id, topic_id, timestamp, data FROM messages"
        ):
            topic_name, topic_type = topics[topic_id]
            if topic_type not in HEADER_TYPES:
                return topic_type, []
            stamp = header_stamp(blob)
            if stamp is None:
                raise ValueError(f"{topic_name} message {message_id} has no header stamp")
            rows.append((message_id, topic_name, stored, stamp))
        return None, rows
    finally:
        con.close()


def aligned_times(rows):
    """Map message id to a playback time on one shared clock.

    Each topic keeps its own header spacing. A constant clock offset that is
    more than a couple of minutes from the group is subtracted so the topics
    overlap. Smaller offsets stay, which keeps a real staggered start.
    """
    by_topic = {}
    for message_id, topic_name, stored, stamp in rows:
        by_topic.setdefault(topic_name, []).append((stored, stamp))
    biases = {
        topic_name: statistics.median(stamp - stored for stored, stamp in items)
        for topic_name, items in by_topic.items()
    }
    bias_ref = statistics.median(biases.values())
    offsets = {}
    for topic_name, bias in biases.items():
        excess = bias - bias_ref
        offsets[topic_name] = int(round(excess)) if abs(excess) > MAX_CLOCK_GAP_NS else 0
    playback = {
        message_id: stamp - offsets[topic_name]
        for message_id, topic_name, _stored, stamp in rows
    }
    return playback, offsets, biases


def replace_metadata(text, file_stats, bag_start, bag_duration):
    lines = text.splitlines(keepends=True)
    in_files = False
    current = None
    expect = None
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped == "files:":
            in_files = True
            expect = None
        elif in_files and "path:" in stripped:
            current = stripped.split("path:", 1)[1].strip()
            expect = None
        elif stripped == "duration:":
            expect = "duration"
        elif stripped == "starting_time:":
            expect = "starting_time"
        elif expect == "duration" and re.match(r"^\s*nanoseconds:\s*\d+\s*$", line):
            value = bag_duration if not in_files else file_stats[current][1]
            ending = "\n" if line.endswith("\n") else ""
            prefix = line.split(":", 1)[0]
            line = f"{prefix}: {value}{ending}"
            expect = None
        elif expect == "starting_time" and re.match(r"^\s*nanoseconds_since_epoch:\s*\d+\s*$", line):
            value = bag_start if not in_files else file_stats[current][0]
            ending = "\n" if line.endswith("\n") else ""
            prefix = line.split(":", 1)[0]
            line = f"{prefix}: {value}{ending}"
            expect = None
        elif stripped and not stripped.startswith("nanoseconds"):
            expect = None
        out.append(line)
    return "".join(out)


def correct(bag_dir):
    name = os.path.basename(bag_dir)
    metadata_path = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.isfile(metadata_path):
        print(f"{name}: skipped, no metadata.yaml")
        return True
    text = open(metadata_path, encoding="utf-8").read()
    storage = re.search(r"^  storage_identifier:\s*(\S+)\s*$", text, re.M)
    if storage is None or storage.group(1) != "sqlite3":
        print(f"{name}: skipped, storage is not sqlite3")
        return True
    compressed = re.search(r"^  compression_format:\s*(.*?)\s*$", text, re.M)
    if compressed and compressed.group(1).strip('"') not in ("", "''"):
        print(f"{name}: skipped, bag is compressed")
        return True
    files = metadata_files(text)
    if not files:
        print(f"{name}: skipped, metadata lists no db3 files")
        return True

    per_file = {}
    total = 0
    for filename in files:
        db_path = os.path.join(bag_dir, filename)
        backup = db_path + ".unsynced"
        if not os.path.isfile(db_path):
            print(f"{name}: missing {filename}", file=sys.stderr)
            return False
        # Recompute from the original recording when a previous run saved one.
        source = backup if os.path.isfile(backup) else db_path
        try:
            skipped_type, rows = read_messages(source)
        except ValueError as exc:
            print(f"{name}: {exc}", file=sys.stderr)
            return False
        if skipped_type is not None:
            print(f"{name}: skipped, {skipped_type} has no header stamp to follow")
            return True
        current_con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
        try:
            current = dict(current_con.execute("SELECT id, timestamp FROM messages"))
        finally:
            current_con.close()
        per_file[filename] = (db_path, backup, source, rows, current)
        total += len(rows)

    if total == 0:
        print(f"{name}: skipped, no messages")
        return True

    playback = {}
    offsets = {}
    for filename, (_db_path, _backup, _source, rows, _current) in per_file.items():
        if rows:
            file_playback, file_offsets, _biases = aligned_times(rows)
        else:
            file_playback, file_offsets = {}, {}
        overlap = set(playback).intersection(file_playback)
        if overlap:
            print(f"{name}: message ids collide across db3 files", file=sys.stderr)
            return False
        playback.update(file_playback)
        for topic_name, offset in file_offsets.items():
            offsets[topic_name] = offset

    changed = 0
    for _filename, (_db_path, _backup, _source, rows, current) in per_file.items():
        for message_id, _topic_name, _stored, _stamp in rows:
            if message_id not in current:
                print(f"{name}: message {message_id} is missing from the bag", file=sys.stderr)
                return False
            if current[message_id] != playback[message_id]:
                changed += 1
    if changed == 0:
        print(f"{name}: already synced ({total} messages)")
        return True

    shifted = sorted((topic, offset) for topic, offset in offsets.items() if offset)
    if shifted:
        largest = max(abs(offset) for _topic, offset in shifted)
        print(
            f"{name}: warning: module clocks differ by up to {largest / 1e9:.3f} s; "
            "shifting them onto one timeline",
            file=sys.stderr,
        )
        for topic_name, offset in shifted:
            print(
                f"  {topic_name}: removed a {offset / 1e9:+.3f} s clock offset",
                file=sys.stderr,
            )

    file_stats = {}
    starts = []
    ends = []
    for filename, (db_path, backup, source, rows, _current) in per_file.items():
        if source == db_path and not os.path.exists(backup):
            shutil.copy2(db_path, backup)
            print(f"{name}: backed up {os.path.basename(backup)}")
        stamps = [playback[message_id] for message_id, _topic, _stored, _stamp in rows]
        busy = subprocess.run(
            ["fuser", db_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if busy.returncode == 0:
            print(
                f"{name}: {filename} is open in another program. "
                "Close that playback and run the script again.",
                file=sys.stderr,
            )
            return False
        con = sqlite3.connect(db_path, timeout=2)
        try:
            con.execute("BEGIN IMMEDIATE")
            con.executemany(
                "UPDATE messages SET timestamp=? WHERE id=?",
                [(playback[message_id], message_id) for message_id, _t, _s, _h in rows],
            )
            con.commit()
        except sqlite3.OperationalError as exc:
            con.close()
            print(
                f"{name}: could not write {filename} ({exc}). "
                "Close anything playing this bag and run the script again.",
                file=sys.stderr,
            )
            return False
        except Exception:
            con.rollback()
            con.close()
            raise
        con.close()
        start = min(stamps)
        end = max(stamps)
        file_stats[filename] = (start, end - start)
        starts.append(start)
        ends.append(end)

    bag_start = min(starts)
    bag_duration = max(ends) - bag_start
    metadata_backup = metadata_path + ".unsynced"
    template = text
    if os.path.isfile(metadata_backup):
        template = open(metadata_backup, encoding="utf-8").read()
    else:
        shutil.copy2(metadata_path, metadata_backup)
    updated = replace_metadata(template, file_stats, bag_start, bag_duration)
    tmp_metadata = metadata_path + ".tmp"
    with open(tmp_metadata, "w", encoding="utf-8") as handle:
        handle.write(updated)
    os.replace(tmp_metadata, metadata_path)

    print(f"{name}: aligned {changed} of {total} playback timestamps")
    for topic_name in sorted(offsets):
        if offsets[topic_name] == 0:
            print(f"  {topic_name}: header time kept")
        else:
            print(f"  {topic_name}: clock shifted by {-offsets[topic_name] / 1e9:+.3f} s")
    print(f"  span {bag_duration / 1e9:.3f} s")
    return True


failed = False
for bag_dir in sys.argv[1:]:
    try:
        if not correct(os.path.abspath(bag_dir)):
            failed = True
    except Exception as exc:
        print(f"{os.path.basename(bag_dir)}: {exc}", file=sys.stderr)
        failed = True
sys.exit(1 if failed else 0)
PY
