#!/usr/bin/env bash
set -Eeuo pipefail

# Ping each argus module.
# The Network line is the network those modules are on (the hotspot when
# their addresses are in the access-point subnet), not this computer's uplink.
# Usage: argus_status.sh

# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")/.." && pwd)/argus_common.sh"

green=$'\033[32m'
red=$'\033[31m'
reset=$'\033[0m'

# Name of the subnet the modules were found on. The access-point subnet is
# the hotspot SSID. Any other subnet uses the connection on that interface.
network_for_prefix() {
    local prefix=$1 iface addr ip conn
    local ap_ip=${ARGUS_AP_ADDR%%/*}
    if [[ "$prefix" == "${ap_ip%.*}" ]]; then
        printf '%s\n' "${ARGUS_SSID:-argus}"
        return 0
    fi
    while read -r _ iface _ addr _; do
        [[ -n "$iface" && -n "$addr" ]] || continue
        ip=${addr%%/*}
        [[ "${ip%.*}" == "$prefix" ]] || continue
        conn=$(nmcli -g GENERAL.CONNECTION device show "$iface" 2>/dev/null || true)
        if [[ -n "$conn" && "$conn" != "--" ]]; then
            printf '%s\n' "$conn"
            return 0
        fi
    done < <(ip -4 -o addr show scope global 2>/dev/null || true)
    return 1
}

mapfile -t hosts < <(argus_each_host)
if [[ ${#hosts[@]} -eq 0 ]]; then
    exit 1
fi

declare -A up_count=()
declare -A any_count=()
rows=()
for entry in "${hosts[@]}"; do
    name=${entry%% *}
    ip=${entry##* }
    prefix=${ip%.*}
    any_count[$prefix]=$(( ${any_count[$prefix]:-0} + 1 ))
    if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
        up_count[$prefix]=$(( ${up_count[$prefix]:-0} + 1 ))
        rows+=("$(printf '%-8s %-15s %s' "$name" "$ip" "${green}up${reset}")")
    else
        rows+=("$(printf '%-8s %-15s %s' "$name" "$ip" "${red}down${reset}")")
    fi
done

# Prefer the subnet where modules answer. If none do, use the addresses listed.
best_prefix=
best_n=0
if [[ ${#up_count[@]} -gt 0 ]]; then
    for prefix in "${!up_count[@]}"; do
        if (( up_count[$prefix] > best_n )); then
            best_n=${up_count[$prefix]}
            best_prefix=$prefix
        fi
    done
else
    for prefix in "${!any_count[@]}"; do
        if (( any_count[$prefix] > best_n )); then
            best_n=${any_count[$prefix]}
            best_prefix=$prefix
        fi
    done
fi
if [[ -n "$best_prefix" ]] && network=$(network_for_prefix "$best_prefix"); then
    printf 'Network: %s\n' "$network"
fi

printf '%s\n' "${rows[@]}"
