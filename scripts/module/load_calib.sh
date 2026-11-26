#!/usr/bin/env bash
set -Eeuo pipefail

# Merge a Kalibr directory into the OpenVINS calib on the matching module.
# Usage: load_calib.sh argus_wide_calib [argusN]
# Files are named like argus0-dynamic_calib-camchain-imucam.yaml and
# argus0-dynamic_calib-imu.yaml. Values shared with the module are updated.
# Matrices that exist only on the module, including Tw, Ta, Tg, R_IMUtoGYRO,
# and R_IMUtoACC, are left unchanged.

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 calib-dir [argusN]" >&2
    exit 1
fi

calib_dir=$(cd "$1" && pwd)
only="${2:-}"
if [[ -n "$only" && ! "$only" =~ ^argus[0-9]+$ ]]; then
    echo "Module name must look like argus0" >&2
    exit 1
fi

# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")/.." && pwd)/argus_common.sh"

declare -A cam_files=()
declare -A imu_files=()
shopt -s nullglob
for file in "$calib_dir"/*.yaml; do
    base=$(basename "$file")
    if [[ "$base" =~ ^(argus[0-9]+)-.+-camchain-imucam\.yaml$ ]]; then
        name="${BASH_REMATCH[1]}"
        if [[ -n "${cam_files[$name]:-}" ]]; then
            echo "More than one camera calib for ${name} in ${calib_dir}" >&2
            exit 1
        fi
        cam_files[$name]=$file
    elif [[ "$base" =~ ^(argus[0-9]+)-.+-imu\.yaml$ ]]; then
        name="${BASH_REMATCH[1]}"
        if [[ -n "${imu_files[$name]:-}" ]]; then
            echo "More than one IMU calib for ${name} in ${calib_dir}" >&2
            exit 1
        fi
        imu_files[$name]=$file
    else
        echo "Skipping ${base}" >&2
    fi
done
shopt -u nullglob

names=()
if [[ -n "$only" ]]; then
    if [[ -z "${cam_files[$only]:-}" || -z "${imu_files[$only]:-}" ]]; then
        echo "Need both an IMU file and a camchain-imucam file for ${only}" >&2
        exit 1
    fi
    names=("$only")
else
    for name in "${!cam_files[@]}"; do
        if [[ -z "${imu_files[$name]:-}" ]]; then
            echo "Missing IMU calib for ${name}" >&2
            exit 1
        fi
        names+=("$name")
    done
    for name in "${!imu_files[@]}"; do
        if [[ -z "${cam_files[$name]:-}" ]]; then
            echo "Missing camera calib for ${name}" >&2
            exit 1
        fi
    done
fi

if [[ ${#names[@]} -eq 0 ]]; then
    echo "No argus calib files in ${calib_dir}" >&2
    exit 1
fi

mapfile -t names < <(printf '%s\n' "${names[@]}" | sort -V)

# Merge one new Kalibr file into the module file. Device-only keys stay verbatim.
merge_calib() {
    python3 - "$@" << 'PY'
import re
import sys

module, kind, new_path, device_path, out_path = sys.argv[1:6]
if kind == "cam":
    section = "cam0"
    topic = f"/{module}/cam0/image_raw"
    required = (
        "T_cam_imu",
        "camera_model",
        "distortion_coeffs",
        "distortion_model",
        "intrinsics",
        "resolution",
        "rostopic",
        "timeshift_cam_imu",
    )
elif kind == "imu":
    section = "imu0"
    topic = f"/{module}/imu0"
    required = (
        "T_i_b",
        "accelerometer_noise_density",
        "accelerometer_random_walk",
        "gyroscope_noise_density",
        "gyroscope_random_walk",
        "rostopic",
        "update_rate",
    )
else:
    sys.exit("kind must be cam or imu")

key_re = re.compile(r"^  ([A-Za-z0-9_]+):(.*)$")
row_re = re.compile(r"^([ ]+)-\s*(\[.*\])\s*$")
section_re = re.compile(r"^(cam0|imu0):\s*$")


def die(message):
    print(message, file=sys.stderr)
    sys.exit(1)


def parse(path):
    lines = open(path, encoding="utf-8").read().splitlines()
    header = []
    index = 0
    while index < len(lines) and not section_re.match(lines[index]):
        header.append(lines[index])
        index += 1
    if index >= len(lines):
        die(f"{path} has no cam0 or imu0 section")
    found = lines[index].split(":", 1)[0]
    index += 1
    order = []
    blocks = {}
    while index < len(lines):
        if lines[index].strip() == "":
            index += 1
            continue
        match = key_re.match(lines[index])
        if not match:
            die(f"{path}: unexpected line {lines[index]!r}")
        name = match.group(1)
        block = [lines[index]]
        index += 1
        while index < len(lines) and lines[index].strip() != "" and not key_re.match(lines[index]):
            if not (lines[index].startswith("   ") or row_re.match(lines[index])):
                die(f"{path}: unexpected line under {name}: {lines[index]!r}")
            block.append(lines[index])
            index += 1
        if name in blocks:
            die(f"{path}: duplicate key {name}")
        order.append(name)
        blocks[name] = block
    return header, found, order, blocks


def inline_value(block):
    match = key_re.match(block[0])
    rest = match.group(2).strip()
    if rest:
        if len(block) != 1:
            die(f"{match.group(1)} has both an inline value and extra lines")
        return rest
    return None


def matrix_rows(block):
    if inline_value(block) is not None:
        return None
    rows = []
    indent = None
    for line in block[1:]:
        match = row_re.match(line)
        if not match:
            die(f"bad matrix row under {block[0]!r}: {line!r}")
        indent = match.group(1)
        rows.append(match.group(2))
    if not rows:
        die(f"empty matrix {block[0]!r}")
    return indent, rows


def check_matrix(name, rows, width):
    if len(rows) != width:
        die(f"{name} should have {width} rows, found {len(rows)}")
    for row in rows:
        cells = [cell.strip() for cell in row.strip()[1:-1].split(",")]
        if len(cells) != width or any(cell == "" for cell in cells):
            die(f"{name} row is not {width} numbers: {row}")


def check_flow(name, value, count):
    if not (value.startswith("[") and value.endswith("]")):
        die(f"{name} should be a list, found {value}")
    cells = [cell.strip() for cell in value[1:-1].split(",")] if value != "[]" else []
    if count is not None and len(cells) != count:
        die(f"{name} should have {count} values, found {value}")


new_header, new_section, new_order, new_blocks = parse(new_path)
device_header, device_section, device_order, device_blocks = parse(device_path)
if new_section != section:
    die(f"{new_path} is {new_section}, expected {section}")
if device_section != section:
    die(f"{device_path} is {device_section}, expected {section}")
for name in required:
    if name not in new_blocks:
        die(f"{new_path} is missing {name}")

topic_value = inline_value(new_blocks["rostopic"])
if topic_value != topic:
    die(f"{new_path} rostopic is {topic_value}, expected {topic}")

if kind == "cam":
    check_matrix("T_cam_imu", matrix_rows(new_blocks["T_cam_imu"])[1], 4)
    check_flow("distortion_coeffs", inline_value(new_blocks["distortion_coeffs"]), 4)
    check_flow("intrinsics", inline_value(new_blocks["intrinsics"]), 4)
    check_flow("resolution", inline_value(new_blocks["resolution"]), 2)
else:
    check_matrix("T_i_b", matrix_rows(new_blocks["T_i_b"])[1], 4)
    for name in ("Tw", "Ta", "Tg", "R_IMUtoGYRO", "R_IMUtoACC"):
        if name not in device_blocks:
            die(f"{device_path} is missing {name}")
        if name in new_blocks:
            die(f"{new_path} contains {name}; that matrix is owned by the module file")

dash_indent = "   "
for name in device_order:
    parsed = matrix_rows(device_blocks[name]) if inline_value(device_blocks[name]) is None else None
    if parsed is not None:
        dash_indent = parsed[0]
        break


def render(name, block):
    inline = inline_value(block)
    if inline is not None:
        return [f"  {name}: {inline}"]
    rows = matrix_rows(block)[1]
    rendered = [f"  {name}:"]
    rendered.extend(f"{dash_indent}- {row}" for row in rows)
    return rendered


order = list(device_order)
for name in new_order:
    if name not in device_blocks:
        order.append(name)

kept = [name for name in device_order if name not in new_blocks]
print(f"  {section}")
for name in order:
    if name not in new_blocks:
        print(f"    kept {name}")
        continue
    old = device_blocks.get(name)
    new = render(name, new_blocks[name])
    if old is None:
        print(f"    added {name}")
    elif old == new:
        print(f"    unchanged {name}")
    elif inline_value(new_blocks[name]) is not None:
        previous = inline_value(old)
        print(f"    {name}: {previous} -> {inline_value(new_blocks[name])}")
    else:
        print(f"    updated {name}")

body = [line for line in device_header if line != ""]
if not any(line.startswith("%YAML:1.0") for line in body):
    body.insert(0, "%YAML:1.0")
body.append(f"{section}:")
for name in order:
    if name in new_blocks:
        body.extend(render(name, new_blocks[name]))
    else:
        body.extend(device_blocks[name])
text = "\n".join(body) + "\n"
for name in kept:
    if f"  {name}:" not in text:
        die(f"merge dropped device-only key {name}")
open(out_path, "w", encoding="utf-8").write(text)
PY
}

update_one() {
    local name=$1
    local ip work stamp remote
    local status=0
    ip=$(argus_resolve "$name")
    work=$(mktemp -d)
    echo "Updating OpenVINS calib on ${name} (${ip})"
    echo "  camera ${cam_files[$name]}"
    echo "  imu    ${imu_files[$name]}"
    scp -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
        "${ARGUS_SSH_USER}@${ip}:openvins_ws/src/open_vins/config/argus/kalibr_imucam_chain.yaml" \
        "${work}/device-imucam.yaml"
    scp -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
        "${ARGUS_SSH_USER}@${ip}:openvins_ws/src/open_vins/config/argus/kalibr_imu_chain.yaml" \
        "${work}/device-imu.yaml"
    merge_calib "$name" cam "${cam_files[$name]}" "${work}/device-imucam.yaml" "${work}/kalibr_imucam_chain.yaml"
    merge_calib "$name" imu "${imu_files[$name]}" "${work}/device-imu.yaml" "${work}/kalibr_imu_chain.yaml"
    scp -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
        "${work}/kalibr_imucam_chain.yaml" \
        "${ARGUS_SSH_USER}@${ip}:/tmp/load_calib_${name}_imucam.yaml"
    scp -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
        "${work}/kalibr_imu_chain.yaml" \
        "${ARGUS_SSH_USER}@${ip}:/tmp/load_calib_${name}_imu.yaml"
    stamp=$(date +%Y%m%d_%H%M%S)
    remote=$(cat << EOF
set -euo pipefail
cfg="\$HOME/openvins_ws/src/open_vins/config/argus"
test -f "\$cfg/kalibr_imucam_chain.yaml"
test -f "\$cfg/kalibr_imu_chain.yaml"
cp -a "\$cfg/kalibr_imucam_chain.yaml" "\$cfg/kalibr_imucam_chain.yaml.bak.${stamp}"
cp -a "\$cfg/kalibr_imu_chain.yaml" "\$cfg/kalibr_imu_chain.yaml.bak.${stamp}"
mv "/tmp/load_calib_${name}_imucam.yaml" "\$cfg/kalibr_imucam_chain.yaml"
mv "/tmp/load_calib_${name}_imu.yaml" "\$cfg/kalibr_imu_chain.yaml"
grep -q '^  Tw:' "\$cfg/kalibr_imu_chain.yaml"
grep -q '^  Ta:' "\$cfg/kalibr_imu_chain.yaml"
grep -q '^  Tg:' "\$cfg/kalibr_imu_chain.yaml"
grep -q '^  R_IMUtoGYRO:' "\$cfg/kalibr_imu_chain.yaml"
grep -q '^  R_IMUtoACC:' "\$cfg/kalibr_imu_chain.yaml"
EOF
)
    "${ARGUS_SSH[@]}" -n -o BatchMode=yes "${ARGUS_SSH_USER}@${ip}" "bash -c $(printf '%q' "$remote")" || status=$?
    rm -rf "$work"
    [[ "$status" -eq 0 ]] || return "$status"
    echo "Installed on ${name}; previous files saved with .bak.${stamp}"
}

fail=0
for name in "${names[@]}"; do
    if ! update_one "$name"; then
        echo "Failed to update ${name}" >&2
        fail=1
    fi
done
exit "$fail"
