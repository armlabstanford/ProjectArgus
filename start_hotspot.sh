#!/usr/bin/env bash
set -Eeuo pipefail

# Host the Argus access point on the built-in Wi-Fi as plain WPA2.
# NetworkManager always adds WPA-PSK-SHA256, which this Pi rejects with
# association status 16 when PMF is off. wpa_supplicant is started directly
# so the beacon offers only WPA-PSK.
# A USB Wi-Fi adapter, when present, is the internet uplink.
# Interface names and the uplink SSID come from argus.conf / argus.local.conf.

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/argus_common.sh"

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

builtin_iface=
usb_iface=
while read -r iface; do
    [[ -n "$iface" ]] || continue
    if argus_is_usb_netdev "$iface"; then
        [[ -z "$usb_iface" ]] && usb_iface=$iface
    else
        [[ -z "$builtin_iface" ]] && builtin_iface=$iface
    fi
done < <(iw dev 2>/dev/null | awk '$1=="Interface"{print $2}')

AP_IFACE="${ARGUS_AP_IFACE:-$builtin_iface}"
UPLINK_IFACE="${ARGUS_UPLINK_IFACE:-$usb_iface}"
if [[ -z "$AP_IFACE" && -n "$UPLINK_IFACE" ]]; then
    AP_IFACE=$UPLINK_IFACE
    UPLINK_IFACE=
fi
if [[ -n "$AP_IFACE" && "$AP_IFACE" == "${UPLINK_IFACE:-}" ]]; then
    UPLINK_IFACE=
fi
if [[ -z "$AP_IFACE" ]]; then
    echo "No wireless interface found for the hotspot." >&2
    exit 1
fi

if [[ $EUID -ne 0 ]]; then
    echo "Run this script with sudo: sudo $0" >&2
    exit 1
fi

if ! command -v nmcli >/dev/null 2>&1; then
    echo "NetworkManager (nmcli) is required." >&2
    exit 1
fi

nmcli radio wifi on

# The access point uses the built-in radio. The USB stick is only the internet
# uplink, so a missing stick must not stop the hotspot from starting.
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
    # Keep the USB uplink if it is already associated. Reloading it drops internet.
    uplink_state=$(nmcli -g GENERAL.STATE device show "$UPLINK_IFACE" 2>/dev/null || true)
    uplink_con=$(nmcli -g GENERAL.CONNECTION device show "$UPLINK_IFACE" 2>/dev/null || true)
    if [[ "$uplink_state" == *connected* && "$uplink_con" == "$UPLINK_USB_CON" ]]; then
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

if [[ "$uplink_ready" -eq 1 ]]; then
    sysctl -w net.ipv4.ip_forward=1 >/dev/null
    iptables -t nat -C POSTROUTING -s 10.42.0.0/24 -o "$UPLINK_IFACE" -j MASQUERADE 2>/dev/null \
        || iptables -t nat -A POSTROUTING -s 10.42.0.0/24 -o "$UPLINK_IFACE" -j MASQUERADE
    iptables -C FORWARD -i "$AP_IFACE" -o "$UPLINK_IFACE" -j ACCEPT 2>/dev/null \
        || iptables -A FORWARD -i "$AP_IFACE" -o "$UPLINK_IFACE" -j ACCEPT
    iptables -C FORWARD -i "$UPLINK_IFACE" -o "$AP_IFACE" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null \
        || iptables -A FORWARD -i "$UPLINK_IFACE" -o "$AP_IFACE" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
fi

echo
echo "Hotspot is active on the laptop Wi-Fi (WPA2-PSK only)."
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
