#!/usr/bin/env bash
set -Eeuo pipefail

# Add or remove a Wi-Fi network on every Argus module that answers ping.
# Ethernet stanzas are left unchanged.
#
#   wifi_net.sh add SSID PASSWORD
#   wifi_net.sh remove SSID
#
# add inserts the network first and marks it preferred in wpa_supplicant, and
# keeps the ones already configured so the Pi can fall back. Run remove later,
# from the new network, to delete the old SSID.

here=$(cd "$(dirname "$0")" && pwd)
# shellcheck disable=SC1091
source "${here}/argus_common.sh"

usage() {
    echo "Usage: $0 add SSID PASSWORD | $0 remove SSID" >&2
    exit 1
}

if [[ $# -lt 2 || $# -gt 3 ]]; then
    usage
fi

mode=$1
ssid=$2
password=${3:-}
case "$mode" in
    add)
        [[ $# -eq 3 ]] || usage
        ;;
    remove)
        [[ $# -eq 2 ]] || usage
        ;;
    *)
        usage
        ;;
esac

# Lease addresses belong to the network that issued them. On the new network,
# prefer an address that actually answers, including DNS for the hostname.
live_address() {
    local name=$1 lease_ip=$2 alt
    if [[ -n "$lease_ip" ]] && ping -c 1 -W 1 "$lease_ip" >/dev/null 2>&1; then
        printf '%s\n' "$lease_ip"
        return 0
    fi
    local candidate
    for candidate in "$name" "${name}.local"; do
        alt=$(getent ahostsv4 "$candidate" 2>/dev/null | awk '{print $1; exit}' || true)
        if [[ -n "$alt" ]] && ping -c 1 -W 1 "$alt" >/dev/null 2>&1; then
            printf '%s\n' "$alt"
            return 0
        fi
    done
    return 1
}

declare -A seen=()
targets=()
lease_ok=0
while read -r name ip; do
    [[ -n "$name" ]] || continue
    [[ -n "${seen[$name]:-}" ]] && continue
    seen[$name]=1
    lease_ok=1
    if addr=$(live_address "$name" "$ip"); then
        targets+=("${name} ${addr}")
    else
        echo "Skipping ${name}: no reachable address" >&2
    fi
done < <(argus_each_host || true)

if [[ "$lease_ok" -eq 0 ]]; then
    prefix=${ARGUS_HOST_PREFIX:-argus}
    n=1
    while [[ "$n" -le 8 ]]; do
        name="${prefix}${n}"
        if addr=$(live_address "$name" ""); then
            targets+=("${name} ${addr}")
        fi
        n=$((n + 1))
    done
fi

if [[ ${#targets[@]} -eq 0 ]]; then
    echo "No active argus modules" >&2
    exit 1
fi

editor=$(cat <<'PY'
import glob
import os
import re
import shutil
import subprocess
import sys

def unquote(text):
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "\"'":
        return text[1:-1]
    return text

def yaml_quote(text):
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'

def indent_of(line):
    return len(line) - len(line.lstrip(" "))

# Keys that belong inside one access point, not the name of a network.
PROPS = {
    "password", "priority", "mode", "band", "channel", "bssid", "hidden",
    "auth", "key-management", "macaddress", "nameservers", "dhcp4", "dhcp6",
    "optional", "wakeonwlan", "regulatory-domain", "renderer", "routes",
    "routing-policy", "addresses", "search", "gateway4", "gateway6",
    "networkmanager", "activation-mode", "match", "set-name", "mtu",
}

def indent_step(lines, index):
    here = indent_of(lines[index])
    for j in range(index - 1, -1, -1):
        line = lines[j]
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        parent = indent_of(line)
        if parent < here:
            return here - parent
    return 2

def edit_text(text, mode, ssid, password):
    lines = text.splitlines()
    changed = False
    i = 0
    while i < len(lines):
        match = re.match(r"^(\s*)access-points:\s*$", lines[i])
        if not match or not inside_wifis(lines, i):
            i += 1
            continue
        ap_indent = len(match.group(1))
        block_start = i + 1
        block_end = block_start
        while block_end < len(lines):
            line = lines[block_end]
            if line.strip() and not line.lstrip().startswith("#") and indent_of(line) <= ap_indent:
                break
            block_end += 1
        body = lines[block_start:block_end]
        step = indent_step(lines, i)
        new_body, body_changed = edit_access_points(body, ap_indent, step, mode, ssid, password)
        if body_changed:
            lines[block_start:block_end] = new_body
            changed = True
            i = block_start + len(new_body)
        else:
            i = block_end
    if mode == "add" and not re.search(r"(?m)^\s*wifis:\s*$", "\n".join(lines)):
        if lines and lines[-1] != "":
            lines.append("")
        lines.extend([
            "  wifis:",
            "    wlan0:",
            "      dhcp4: true",
            "      optional: true",
            "      access-points:",
            "        %s:" % yaml_quote(ssid),
            "          password: %s" % yaml_quote(password),
        ])
        changed = True
    return "\n".join(lines) + ("\n" if text.endswith("\n") or changed else ""), changed

def inside_wifis(lines, index):
    ap_indent = indent_of(lines[index])
    for j in range(index - 1, -1, -1):
        line = lines[j]
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if indent_of(line) < ap_indent and re.match(r"^\s*wifis:\s*$", line):
            return True
        if indent_of(line) < ap_indent and re.match(r"^\s*(ethernets|bridges|bonds|vlans|tunnels):\s*$", line):
            return False
    return False

def is_ap_key(line, ap_indent):
    stripped = line.strip()
    if not stripped or stripped.startswith("#") or not stripped.endswith(":"):
        return False, ""
    if indent_of(line) <= ap_indent:
        return False, ""
    name = unquote(stripped[:-1].strip())
    if name in PROPS:
        return False, ""
    return True, name

def edit_access_points(body, ap_indent, step, mode, ssid, password):
    key_indent = ap_indent + step
    blocks = []
    current = None
    for line in body:
        found, name = is_ap_key(line, ap_indent)
        if found:
            if current is not None:
                blocks.append(current)
            current = [name, [line]]
        elif current is None:
            if not blocks or blocks[0][0] != "":
                blocks.insert(0, ["", []])
            blocks[0][1].append(line)
        else:
            current[1].append(line)
    if current is not None:
        blocks.append(current)

    changed = False
    kept = []
    for name, chunk in blocks:
        if not name:
            kept.append((name, chunk))
            continue
        if name == ssid:
            changed = True
            continue
        cleaned = [line for line in chunk if not re.match(r"^\s*priority:\s*", line)]
        if cleaned != chunk:
            changed = True
        kept.append((name, cleaned))
    if mode == "add":
        entry = (ssid, network_block(key_indent, step, ssid, password))
        insert_at = 0
        for n, (name, _) in enumerate(kept):
            if name:
                insert_at = n
                break
            insert_at = n + 1
        kept.insert(insert_at, entry)
        changed = True

    out = []
    for _, chunk in kept:
        out.extend(chunk)
    return out, changed

def network_block(key_indent, step, ssid, password):
    pad = " " * key_indent
    child = " " * (key_indent + step)
    return [
        "%s%s:" % (pad, yaml_quote(ssid)),
        "%spassword: %s" % (child, yaml_quote(password)),
    ]

# This netplan has no access-point priority key. wpa_supplicant does, and a
# higher priority is what makes the new network win while the old one remains
# as a fallback. The helper runs before wpa_supplicant starts.
HELPER = "/usr/local/sbin/argus-wifi-prefer"
PREFER_PATH = "/etc/argus-wifi-prefer"
HELPER_SRC = r'''#!/usr/bin/env python3
import glob
import re
import sys

try:
    prefer = open("/etc/argus-wifi-prefer", encoding="utf-8").read().strip()
except OSError:
    sys.exit(0)
if not prefer:
    sys.exit(0)

def patch(text, prefer):
    out = []
    index = 0
    while True:
        start = text.find("network={", index)
        if start < 0:
            out.append(text[index:])
            break
        end = text.find("\n}", start)
        if end < 0:
            out.append(text[index:])
            break
        block = text[start:end]
        found = re.search(r'(?m)^\s*ssid=(?:"([^"]*)"|(\S+))\s*$', block)
        name = None
        if found:
            name = found.group(1) if found.group(1) is not None else found.group(2)
        lines = [ln for ln in block.splitlines() if not re.match(r"\s*priority=\d+\s*$", ln)]
        block = "\n".join(lines)
        if name == prefer:
            block += "\n  priority=100"
        out.append(text[index:start])
        out.append(block)
        index = end
    return "".join(out)

for conf in glob.glob("/run/netplan/wpa-*.conf"):
    try:
        original = open(conf, encoding="utf-8").read()
    except OSError:
        continue
    updated = patch(original, prefer)
    if updated != original:
        with open(conf, "w", encoding="utf-8") as handle:
            handle.write(updated)
sys.exit(0)
'''

def wifi_ifaces(text):
    skip = {
        "renderer", "dhcp4", "dhcp6", "optional", "version",
        "access-points", "nameservers",
    }
    ifaces = []
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if not re.match(r"^\s*wifis:\s*$", line):
            continue
        base = indent_of(line)
        step = indent_step(lines, i)
        for j in range(i + 1, len(lines)):
            cur = lines[j]
            if not cur.strip() or cur.lstrip().startswith("#"):
                continue
            ind = indent_of(cur)
            if ind <= base:
                break
            if ind != base + step:
                continue
            found = re.match(r"^\s*([A-Za-z0-9._-]+):\s*$", cur)
            if found and found.group(1) not in skip:
                ifaces.append(found.group(1))
    return ifaces or ["wlan0"]

def install_prefer(ssid, ifaces, enable):
    if enable:
        with open(PREFER_PATH, "w", encoding="utf-8") as handle:
            handle.write(ssid + "\n")
    else:
        try:
            current = open(PREFER_PATH, encoding="utf-8").read().strip()
        except OSError:
            current = ""
        if current == ssid and os.path.exists(PREFER_PATH):
            os.remove(PREFER_PATH)
    os.makedirs(os.path.dirname(HELPER), exist_ok=True)
    with open(HELPER, "w", encoding="utf-8") as handle:
        handle.write(HELPER_SRC)
    os.chmod(HELPER, 0o755)
    for iface in ifaces:
        directory = "/etc/systemd/system/netplan-wpa-%s.service.d" % iface
        os.makedirs(directory, exist_ok=True)
        dropin = directory + "/prefer.conf"
        with open(dropin, "w", encoding="utf-8") as handle:
            handle.write("[Service]\nExecStartPre=%s\n" % HELPER)
    subprocess.call(["systemctl", "daemon-reload"])

def main():
    mode, ssid = sys.argv[1], sys.argv[2]
    password = sys.argv[3] if len(sys.argv) > 3 else ""
    paths = sorted(glob.glob("/etc/netplan/*.yaml"))
    wifi_paths = []
    for path in paths:
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read()
        if re.search(r"(?m)^\s*wifis:\s*$", text) or (mode == "add" and not wifi_paths and path == paths[-1]):
            wifi_paths.append((path, text))
    if mode == "add" and not any(re.search(r"(?m)^\s*wifis:\s*$", text) for _, text in wifi_paths):
        path = "/etc/netplan/99-argus-wifi.yaml"
        wifi_paths = [(path, "network:\n  version: 2\n")]
    if not wifi_paths:
        print("No Wi-Fi netplan file to edit", file=sys.stderr)
        return 1
    wrote = []
    for path, text in wifi_paths:
        if mode == "remove" and not re.search(r"(?m)^\s*wifis:\s*$", text):
            continue
        updated, changed = edit_text(text, mode, ssid, password)
        if not changed:
            continue
        backup = path + ".argus-bak"
        if os.path.exists(path):
            shutil.copy2(path, backup)
        else:
            backup = ""
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(updated)
        os.chmod(path, 0o600)
        print("Updated %s" % path)
        wrote.append((path, backup))
    if not wrote:
        print("No change for %s" % ssid)
        return 0
    cloud = "/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg"
    os.makedirs(os.path.dirname(cloud), exist_ok=True)
    if not os.path.exists(cloud):
        with open(cloud, "w", encoding="utf-8") as handle:
            handle.write("network: {config: disabled}\n")
        print("Disabled cloud-init network rewrite")
    ifaces = []
    for path, _text in wrote:
        with open(path, encoding="utf-8") as handle:
            ifaces.extend(wifi_ifaces(handle.read()))
    install_prefer(ssid, ifaces or ["wlan0"], mode == "add")
    try:
        subprocess.check_call(["netplan", "generate"])
    except subprocess.CalledProcessError:
        for path, backup in wrote:
            if backup:
                shutil.copy2(backup, path)
                print("Restored %s" % path, file=sys.stderr)
            elif os.path.exists(path):
                os.remove(path)
        return 1
    for _, backup in wrote:
        if backup and os.path.exists(backup):
            os.remove(backup)
    # Apply after this SSH session returns, so a switch to the new network
    # does not cut off the command that wrote the config.
    reapply = "/tmp/argus-wifi-reapply.sh"
    with open(reapply, "w", encoding="utf-8") as handle:
        handle.write("#!/bin/sh\nsleep 2\nnetplan apply > /tmp/argus-wifi-apply.log 2>&1\n")
    os.chmod(reapply, 0o755)
    subprocess.Popen(
        [reapply],
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print("Saved. Wi-Fi will reload in a few seconds.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
PY
)

fail=0
payload=$(printf '%s' "$editor" | base64 -w0)
for entry in "${targets[@]}"; do
    name=${entry%% *}
    ip=${entry##* }
    echo "Updating Wi-Fi on ${name} (${ip})"
    if [[ "$mode" == "add" ]]; then
        remote_args=$(printf '%q ' "$mode" "$ssid" "$password")
    else
        remote_args=$(printf '%q ' "$mode" "$ssid")
    fi
    # A leftover Enter from the previous host would be an empty sudo password.
    if [[ -t 0 ]]; then
        while IFS= read -r -t 0.05 _; do
            :
        done || true
    fi
    if ! ssh -o ConnectTimeout=8 "${ARGUS_SSH_USER}@${ip}" \
        "base64 -d > /tmp/argus-wifi-net.py" <<<"${payload}"; then
        echo "Failed on ${name}" >&2
        fail=1
        continue
    fi
    # A separate ssh -t leaves sudo's stdin on the terminal.
    if ! ssh -t -o ConnectTimeout=8 "${ARGUS_SSH_USER}@${ip}" \
        "sudo python3 /tmp/argus-wifi-net.py ${remote_args}; status=\$?; rm -f /tmp/argus-wifi-net.py; exit \$status"; then
        echo "Failed on ${name}" >&2
        fail=1
    fi
done

exit "$fail"
