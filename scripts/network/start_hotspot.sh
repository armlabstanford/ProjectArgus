#!/usr/bin/env bash
set -Eeuo pipefail

# Host the Argus access point on the Archer T2U Plus (RTL8821AU) as plain WPA2.
# NetworkManager always adds WPA-PSK-SHA256, which this Pi rejects with
# association status 16 when PMF is off. wpa_supplicant is started directly
# so the beacon offers only WPA-PSK.
# The built-in radio, when it is a different interface, is the internet uplink.
# Interface names and the uplink SSID come from argus.conf / argus.local.conf.

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/argus_common.sh"

SSID="${ARGUS_SSID:-argus}"
PASSWORD="${ARGUS_PASSWORD:-ArmLab19}"
AP_ADDR="${ARGUS_AP_ADDR:-10.42.0.1/24}"
AP_IP="${AP_ADDR%%/*}"
AP_PREFIX="${AP_IP%.*}"
ETHERNET_ADDR="${ARGUS_ETHERNET_ADDR:-10.42.1.1/24}"
CON_NAME="${SSID}-Hotspot"
UPLINK_CON="${ARGUS_UPLINK_SSID:-}"
UPLINK_USB_CON="${UPLINK_CON:+${UPLINK_CON}-usb}"
WPA_CONF="/run/arm_lab_ap.conf"
WPA_PID="/run/arm_lab_wpa.pid"
DNS_CONF="/run/arm_lab_dnsmasq.conf"
DNS_PID="/run/arm_lab_dnsmasq.pid"
LEASE_FILE="${ARGUS_LEASE_FILE:-/var/lib/misc/dnsmasq.leases}"

argus_is_usb_netdev() {
    local path
    path=$(readlink -f "/sys/class/net/$1/device" 2>/dev/null || true)
    [[ "$path" == */usb* ]]
}

argus_wifi_ifaces() {
    local path
    shopt -s nullglob
    for path in /sys/class/net/*; do
        [[ -e "${path}/phy80211" || -d "${path}/wireless" ]] || continue
        printf '%s\n' "$(basename "$path")"
    done
    shopt -u nullglob
}

argus_netdev_usb_id() {
    local dev
    dev=$(readlink -f "/sys/class/net/$1/device" 2>/dev/null || true)
    [[ -n "$dev" ]] || return 1
    while [[ "$dev" != "/" && ! -f "${dev}/idVendor" ]]; do
        dev=$(dirname "$dev")
    done
    [[ -f "${dev}/idVendor" ]] || return 1
    printf '%s:%s\n' "$(<"${dev}/idVendor")" "$(<"${dev}/idProduct")"
}

# Archer T2U Plus (RTL8821AU). The Tenda name is a fallback.
ARCHER_USB_ID=2357:0120
TENDA_IFACE=wlxc83a35c807b4

argus_usb_device_present() {
    local id=$1
    local vendor=${id%%:*}
    local product=${id##*:}
    local d
    shopt -s nullglob
    for d in /sys/bus/usb/devices/*; do
        [[ -f "${d}/idVendor" && -f "${d}/idProduct" ]] || continue
        if [[ "$(<"${d}/idVendor")" == "$vendor" && "$(<"${d}/idProduct")" == "$product" ]]; then
            shopt -u nullglob
            return 0
        fi
    done
    shopt -u nullglob
    return 1
}

argus_find_archer_iface() {
    local iface
    while read -r iface; do
        [[ -n "$iface" ]] || continue
        if [[ "$(argus_netdev_usb_id "$iface" 2>/dev/null || true)" == "$ARCHER_USB_ID" ]]; then
            printf '%s\n' "$iface"
            return 0
        fi
    done < <(argus_wifi_ifaces)
    return 1
}

# Ubuntu 6.8.0-147+ backported cfg80211 set_monitor_channel(net_device) while
# LINUX_VERSION_CODE stays 6.8, so stock 8821au 5.12.5.2 fails to build.
argus_patch_8821au_source() {
    local src=${1:-/usr/src/rtl8821au-5.12.5.2}
    local cfg="${src}/os_dep/linux/ioctl_cfg80211.c"
    local mk="${src}/Makefile"
    [[ -f "$cfg" && -f "$mk" ]] || return 1
    if ! grep -q 'RTW_CFG80211_MONITOR_CHANNEL_HAS_NETDEV' "$cfg"; then
        sed -i 's/#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 101))/#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 101)) || defined(RTW_CFG80211_MONITOR_CHANNEL_HAS_NETDEV)/' "$cfg"
    fi
    if ! grep -q 'RTW_CFG80211_MONITOR_CHANNEL_HAS_NETDEV' "$mk"; then
        python3 - "$mk" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
text = p.read_text()
needle = "ifneq ($(KERNELRELEASE),)\n"
insert = needle + """
# Ubuntu 6.8.0-147+ backported this argument; do not trust LINUX_VERSION_CODE.
ifeq ($(shell awk '/\\*set_monitor_channel/,/;/ { if (/struct net_device/) { print "y"; exit } }' $(srctree)/include/net/cfg80211.h 2>/dev/null), y)
ccflags-y += -DRTW_CFG80211_MONITOR_CHANNEL_HAS_NETDEV
endif

"""
if "RTW_CFG80211_MONITOR_CHANNEL_HAS_NETDEV" in text:
    raise SystemExit(0)
if needle not in text:
    raise SystemExit(f"Makefile marker not found in {p}")
p.write_text(text.replace(needle, insert, 1))
PY
    fi
}

# After a kernel update DKMS often has 8821au only for the previous kernel.
# Without that module the Archer has no netdev and the old fallback stole wlp1s0.
# Do not `dkms autoinstall`: amdgpu DKMS is broken on this kernel and is unrelated.
argus_load_archer_driver() {
    local kver src
    kver=$(uname -r)
    src=/usr/src/rtl8821au-5.12.5.2
    if ! modinfo 8821au >/dev/null 2>&1; then
        echo "8821au is not built for kernel ${kver}; building rtl8821au DKMS..." >&2
        if [[ -d "$src" ]]; then
            argus_patch_8821au_source "$src" || true
        fi
        if ! dkms install "rtl8821au/5.12.5.2" -k "$kver"; then
            echo "rtl8821au DKMS build failed. See /var/lib/dkms/rtl8821au/5.12.5.2/build/make.log" >&2
            return 1
        fi
    fi
    if ! modprobe 8821au; then
        echo "Could not load 8821au (needed for the Archer T2U Plus AP)." >&2
        return 1
    fi
    return 0
}

argus_wait_for_archer() {
    local i iface last=
    # 8821au first registers as wlan0; udev then renames it to wlx<mac>.
    udevadm settle -t 8 >/dev/null 2>&1 || true
    for i in $(seq 1 80); do
        iface=$(argus_find_archer_iface || true)
        if [[ -n "$iface" && "$iface" != wlan0 && "$iface" != wlan1 ]]; then
            printf '%s\n' "$iface"
            return 0
        fi
        [[ -n "$iface" ]] && last=$iface
        sleep 0.25
    done
    if [[ -n "$last" ]]; then
        printf '%s\n' "$last"
        return 0
    fi
    return 1
}

if [[ $EUID -ne 0 ]]; then
    echo "Run this script with sudo: sudo $0${1:+ $1}" >&2
    exit 1
fi

if [[ "${1:-}" == "stop" ]]; then
    if [[ -f "$WPA_PID" ]]; then
        kill "$(cat "$WPA_PID")" 2>/dev/null || true
        rm -f "$WPA_PID"
    fi
    if [[ -f "$DNS_PID" ]]; then
        kill "$(cat "$DNS_PID")" 2>/dev/null || true
        rm -f "$DNS_PID"
    fi
    ap_if=$(ip -4 -o addr show | awk -v ip="$AP_IP" '$4 ~ "^" ip "/" {print $2; exit}')
    if [[ -n "$ap_if" ]]; then
        ip addr del "$AP_ADDR" dev "$ap_if" 2>/dev/null || true
        ip link set "$ap_if" down 2>/dev/null || true
        nmcli device set "$ap_if" managed yes || true
    fi
    rm -f "$WPA_CONF" "$DNS_CONF"
    echo "Hotspot stopped."
    exit 0
fi

argus_load_archer_driver || true
archer_iface=$(argus_wait_for_archer || true)

builtin_iface=
usb_iface=
while read -r iface; do
    [[ -n "$iface" ]] || continue
    if argus_is_usb_netdev "$iface"; then
        [[ -z "$usb_iface" ]] && usb_iface=$iface
    else
        [[ -z "$builtin_iface" ]] && builtin_iface=$iface
    fi
done < <(argus_wifi_ifaces)

if [[ -n "${ARGUS_AP_IFACE:-}" ]]; then
    AP_IFACE=$ARGUS_AP_IFACE
elif [[ -n "$archer_iface" ]]; then
    AP_IFACE=$archer_iface
elif ip link show "$TENDA_IFACE" >/dev/null 2>&1; then
    AP_IFACE=$TENDA_IFACE
elif [[ -n "$usb_iface" && "$usb_iface" != "$builtin_iface" ]]; then
    AP_IFACE=$usb_iface
else
    AP_IFACE=
fi
if [[ -n "${ARGUS_UPLINK_IFACE:-}" ]]; then
    UPLINK_IFACE=$ARGUS_UPLINK_IFACE
elif [[ -n "$builtin_iface" && "$builtin_iface" != "$AP_IFACE" ]]; then
    UPLINK_IFACE=$builtin_iface
elif [[ -n "$usb_iface" && "$usb_iface" != "$AP_IFACE" ]]; then
    UPLINK_IFACE=$usb_iface
else
    UPLINK_IFACE=
fi
if [[ -n "$AP_IFACE" && "$AP_IFACE" == "${UPLINK_IFACE:-}" ]]; then
    UPLINK_IFACE=
fi
if [[ -z "$AP_IFACE" ]]; then
    if argus_usb_device_present "$ARCHER_USB_ID"; then
        echo "Archer T2U Plus is plugged in but has no wireless interface." >&2
        echo "8821au is missing for kernel $(uname -r). After a reboot, rebuild it with:" >&2
        echo "  sudo dkms autoinstall -k $(uname -r)" >&2
        echo "  sudo modprobe 8821au" >&2
    else
        echo "No USB Wi-Fi adapter found for the hotspot (Archer T2U Plus 2357:0120)." >&2
    fi
    echo "Refusing to use the built-in radio (${builtin_iface:-none}); that would drop eduroam." >&2
    exit 1
fi
if ! ip link show "$AP_IFACE" >/dev/null 2>&1; then
    echo "Hotspot interface ${AP_IFACE} is not present." >&2
    exit 1
fi
if [[ -n "$builtin_iface" && "$AP_IFACE" == "$builtin_iface" && "${ARGUS_ALLOW_BUILTIN_AP:-}" != "1" ]]; then
    echo "Hotspot interface ${AP_IFACE} is the built-in radio. That would drop internet." >&2
    echo "Plug in the Archer T2U Plus, or set ARGUS_ALLOW_BUILTIN_AP=1 to override." >&2
    exit 1
fi

# Re-read the Archer name in case udev renamed wlan0 after we first saw it.
if [[ -z "${ARGUS_AP_IFACE:-}" ]]; then
    renamed=$(argus_find_archer_iface || true)
    if [[ -n "$renamed" ]]; then
        AP_IFACE=$renamed
    fi
fi
if ! ip link show "$AP_IFACE" >/dev/null 2>&1; then
    echo "Hotspot interface ${AP_IFACE} is not present." >&2
    exit 1
fi

if ! command -v nmcli >/dev/null 2>&1; then
    echo "NetworkManager (nmcli) is required." >&2
    exit 1
fi

nmcli radio wifi on

# The access point uses the Archer adapter. The other radio is only the internet
# uplink, so a missing uplink must not stop the hotspot from starting.
uplink_ready=0
if [[ -z "${UPLINK_IFACE}" ]] || ! ip link show "$UPLINK_IFACE" >/dev/null 2>&1; then
    echo "No USB uplink is connected. Hotspot will start without internet."
elif [[ -z "$UPLINK_CON" ]]; then
    uplink_state=$(nmcli -g GENERAL.STATE device show "$UPLINK_IFACE" 2>/dev/null || true)
    if [[ "$uplink_state" == *connected* ]]; then
        uplink_ready=1
        UPLINK_CON=$(nmcli -g GENERAL.CONNECTION device show "$UPLINK_IFACE" 2>/dev/null || true)
    else
        echo "USB adapter ${UPLINK_IFACE} has no uplink SSID in argus.local.conf. Hotspot will start without internet."
    fi
else
    # Keep the uplink if it is already associated. Reloading it drops internet.
    # After reboot the built-in radio is on UPLINK_CON (eduroam), not the cloned
    # *-usb profile — treat that as already up and do not disable autoconnect.
    uplink_state=$(nmcli -g GENERAL.STATE device show "$UPLINK_IFACE" 2>/dev/null || true)
    uplink_con=$(nmcli -g GENERAL.CONNECTION device show "$UPLINK_IFACE" 2>/dev/null || true)
    if [[ "$uplink_state" == *connected* && ( "$uplink_con" == "$UPLINK_USB_CON" || "$uplink_con" == "$UPLINK_CON" ) ]]; then
        uplink_ready=1
    else
        if nmcli -t -f NAME connection show | grep -Fxq "$CON_NAME"; then
            nmcli connection down "$CON_NAME" >/dev/null 2>&1 || true
        fi
        nmcli device set "$UPLINK_IFACE" managed yes || true
        uplink_driver=$(basename "$(readlink -f "/sys/class/net/${UPLINK_IFACE}/device/driver" 2>/dev/null)" 2>/dev/null || true)
        if [[ "$uplink_driver" == "88XXau" || -f /etc/modprobe.d/88XXau-ap.conf ]]; then
            rm -f /etc/modprobe.d/88XXau-ap.conf
            ip link set "$UPLINK_IFACE" down || true
            modprobe -r 88XXau || true
            modprobe 88XXau || true
        fi
        for _ in $(seq 1 50); do
            if ip link show "$UPLINK_IFACE" >/dev/null 2>&1; then
                break
            fi
            sleep 0.1
        done
        if ! ip link show "$UPLINK_IFACE" >/dev/null 2>&1; then
            echo "USB adapter ${UPLINK_IFACE} did not come back. Hotspot will start without internet."
        else
            nmcli device set "$UPLINK_IFACE" managed yes
            if ! nmcli -t -f NAME connection show | grep -Fxq "$UPLINK_USB_CON"; then
                nmcli connection clone "$UPLINK_CON" "$UPLINK_USB_CON" >/dev/null
            fi
            nmcli connection modify "$UPLINK_USB_CON" \
                connection.interface-name "$UPLINK_IFACE" \
                connection.autoconnect yes
            echo "Joining ${UPLINK_CON} on ${UPLINK_IFACE}..."
            if nmcli --wait 40 connection up "$UPLINK_USB_CON" ifname "$UPLINK_IFACE"; then
                nmcli connection modify "$UPLINK_CON" connection.autoconnect no
                uplink_ready=1
            else
                echo "USB adapter did not join ${UPLINK_CON}. Hotspot will start without internet."
            fi
        fi
    fi
fi

if nmcli -t -f NAME connection show | grep -Fxq "$CON_NAME"; then
    nmcli connection down "$CON_NAME" >/dev/null 2>&1 || true
fi
if [[ -z "${ARGUS_AP_IFACE:-}" ]]; then
    renamed=$(argus_find_archer_iface || true)
    if [[ -n "$renamed" ]]; then
        AP_IFACE=$renamed
    fi
fi
if ! ip link show "$AP_IFACE" >/dev/null 2>&1; then
    echo "Hotspot interface ${AP_IFACE} is not present (udev may have renamed it)." >&2
    exit 1
fi
nmcli device set "$AP_IFACE" managed no

if [[ -f "$WPA_PID" ]]; then
    kill "$(cat "$WPA_PID")" 2>/dev/null || true
    rm -f "$WPA_PID"
fi
if [[ -f "$DNS_PID" ]]; then
    kill "$(cat "$DNS_PID")" 2>/dev/null || true
    rm -f "$DNS_PID"
fi

ip addr flush dev "$AP_IFACE"
ip link set "$AP_IFACE" up
ip addr add "$AP_ADDR" dev "$AP_IFACE"

install -d /run/wpa_supplicant_ap
cat > "$WPA_CONF" <<EOF
ctrl_interface=/run/wpa_supplicant_ap
country=US
network={
    ssid="${SSID}"
    mode=2
    frequency=2437
    key_mgmt=WPA-PSK
    psk="${PASSWORD}"
    proto=RSN
    pairwise=CCMP
    group=CCMP
    ieee80211w=0
}
EOF
chmod 600 "$WPA_CONF"

wpa_supplicant -B -i "$AP_IFACE" -c "$WPA_CONF" -P "$WPA_PID"

{
    cat <<EOF
interface=${AP_IFACE}
bind-interfaces
dhcp-range=${AP_PREFIX}.10,${AP_PREFIX}.100,255.255.255.0,12h
dhcp-option=option:router,${AP_IP}
dhcp-option=option:dns-server,1.1.1.1
dhcp-leasefile=${LEASE_FILE}
pid-file=${DNS_PID}
EOF
    # Keep the addresses already logged for the modules.
    if [[ -n "${ARGUS_DHCP_HOSTS:-}" ]]; then
        while IFS= read -r row; do
            row=${row%%#*}
            row=${row//[[:space:]]/}
            [[ -n "$row" ]] || continue
            printf 'dhcp-host=%s,infinite\n' "$row"
        done <<< "${ARGUS_DHCP_HOSTS}"
    fi
} > "$DNS_CONF"
dnsmasq -C "$DNS_CONF"

# A NetworkManager "shared" ethernet profile takes the same address as the
# hotspot. Pin those profiles to the other subnet.
while IFS=: read -r cname ctype; do
    [[ "$ctype" == "802-3-ethernet" ]] || continue
    method=$(nmcli -g ipv4.method connection show "$cname" 2>/dev/null || true)
    addr=$(nmcli -g ipv4.addresses connection show "$cname" 2>/dev/null || true)
    if [[ "$method" == "shared" || "$addr" == "${AP_IP}/"* || "$addr" == "${AP_PREFIX}."* ]]; then
        nmcli connection modify "$cname" \
            ipv4.method shared \
            ipv4.addresses "$ETHERNET_ADDR"
        nmcli connection up "$cname" >/dev/null 2>&1 || true
    fi
done < <(nmcli -t -f NAME,TYPE connection show)

# Installed whenever the uplink interface exists, so internet works as soon as
# it connects, even if it was still joining when the hotspot started.
allow_forward() {
    local chain=$1
    shift
    iptables -C "$chain" "$@" -j ACCEPT 2>/dev/null \
        || iptables -I "$chain" 1 "$@" -j ACCEPT
}
if [[ -n "${UPLINK_IFACE}" ]] && ip link show "$UPLINK_IFACE" >/dev/null 2>&1; then
    AP_SUBNET="${AP_PREFIX}.0/24"
    sysctl -w net.ipv4.ip_forward=1 >/dev/null
    iptables -t nat -C POSTROUTING -s "$AP_SUBNET" -o "$UPLINK_IFACE" -j MASQUERADE 2>/dev/null \
        || iptables -t nat -A POSTROUTING -s "$AP_SUBNET" -o "$UPLINK_IFACE" -j MASQUERADE
    # Docker sets the FORWARD policy to DROP and evaluates DOCKER-USER first,
    # so the accepts go at the top of both chains.
    forward_chains=(FORWARD)
    if iptables -S DOCKER-USER >/dev/null 2>&1; then
        forward_chains+=(DOCKER-USER)
    fi
    for chain in "${forward_chains[@]}"; do
        allow_forward "$chain" -i "$UPLINK_IFACE" -o "$AP_IFACE" -m conntrack --ctstate RELATED,ESTABLISHED
        allow_forward "$chain" -i "$AP_IFACE" -o "$UPLINK_IFACE"
    done
fi

echo
echo "Hotspot is active on ${AP_IFACE} (WPA2-PSK only)."
echo "  SSID:      $SSID"
echo "  Password:  $PASSWORD"
echo "  Interface: $AP_IFACE"
echo "  Address:   ${AP_IP}"
if [[ "$uplink_ready" -eq 1 ]]; then
    echo "  Uplink:    $UPLINK_IFACE ($UPLINK_CON)"
else
    echo "  Uplink:    none"
fi
echo
echo "Stop it with:"
echo "  sudo kill \$(cat $WPA_PID) \$(cat $DNS_PID)"
echo "  sudo nmcli device set $AP_IFACE managed yes"
if [[ -n "$UPLINK_CON" ]]; then
    echo "  sudo nmcli connection modify '$UPLINK_CON' connection.autoconnect yes"
    echo "  sudo nmcli connection up '$UPLINK_CON'"
fi
