#!/usr/bin/env bash
set -Eeuo pipefail

# Ping each argus host known to the hotspot DHCP server.
# Usage: argus_status.sh

# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/argus_common.sh"

green=$'\033[32m'
red=$'\033[31m'
reset=$'\033[0m'

mapfile -t hosts < <(argus_each_host)
if [[ ${#hosts[@]} -eq 0 ]]; then
    exit 1
fi

for entry in "${hosts[@]}"; do
    name=${entry%% *}
    ip=${entry##* }
    if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
        printf '%-8s %-15s %s\n' "$name" "$ip" "${green}up${reset}"
    else
        printf '%-8s %-15s %s\n' "$name" "$ip" "${red}down${reset}"
    fi
done
