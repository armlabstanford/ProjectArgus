# Shared lookup for the Argus scripts. Source this file; do not execute it.

# This file is in scripts/; argus.conf is at the repo root.
ARGUS_SCRIPTS=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ARGUS_ROOT=$(cd "${ARGUS_SCRIPTS}/.." && pwd)
_argus_root="$ARGUS_ROOT"
if [[ -f "${_argus_root}/argus.conf" ]]; then
    # shellcheck disable=SC1091
    source "${_argus_root}/argus.conf"
fi
if [[ -f "${_argus_root}/argus.local.conf" ]]; then
    # shellcheck disable=SC1091
    source "${_argus_root}/argus.local.conf"
fi

argus_lease_files() {
    local f
    if [[ -n "${ARGUS_LEASE_FILE:-}" && -r "${ARGUS_LEASE_FILE}" ]]; then
        printf '%s\n' "${ARGUS_LEASE_FILE}"
    fi
    if [[ -r /var/lib/misc/dnsmasq.leases && "${ARGUS_LEASE_FILE:-}" != /var/lib/misc/dnsmasq.leases ]]; then
        printf '%s\n' /var/lib/misc/dnsmasq.leases
    fi
    shopt -s nullglob
    for f in /var/lib/NetworkManager/dnsmasq*.leases /run/dnsmasq*.leases; do
        [[ -r "$f" ]] && printf '%s\n' "$f"
    done
    shopt -u nullglob
}

# True when the address is on a subnet attached to this computer.
# A hotspot lease such as 10.42.0.36 is not on-link while we are on ARMLab-5G.
argus_addr_on_link() {
    local route
    route=$(ip route get "$1" 2>/dev/null) || return 1
    [[ "$route" != *" via "* ]]
}

# /24 bases (a.b.c) of networks this computer is connected to.
argus_scan_bases() {
    local iface addr ip
    while read -r _ iface _ addr _; do
        [[ -n "$iface" && -n "$addr" ]] || continue
        case "$iface" in
            lo|docker*|br-*|veth*|virbr*|tun*|tailscale*|zt*) continue ;;
        esac
        ip=${addr%%/*}
        [[ "$ip" == *.*.*.* ]] || continue
        printf '%s\n' "${ip%.*}"
    done < <(ip -4 -o addr show scope global 2>/dev/null || true)
}

# Ping a /24 so the neighbor table learns the devices on this network.
argus_probe_prefix() {
    local base=$1
    (
        i=
        for i in $(seq 1 254); do
            ping -c 1 -W 1 -n "${base}.${i}" >/dev/null 2>&1 &
        done
        wait || true
    )
}

# Print "mac ip" for neighbors on this /24.
# mawk rejects a for-loop here, so the fields are split in bash.
argus_neigh_macs() {
    local base=$1 ip rest mac
    while read -r ip rest; do
        [[ "$ip" == "${base}."* ]] || continue
        [[ "$rest" == *" lladdr "* ]] || continue
        [[ "$rest" == *" FAILED" || "$rest" == *" INCOMPLETE" ]] && continue
        mac=${rest#*lladdr }
        mac=${mac%% *}
        printf '%s %s\n' "${mac,,}" "$ip"
    done < <(ip -4 neigh show 2>/dev/null || true)
}

# Print "name ip" for prefix-matching reverse DNS names on this /24.
# Used when a module has no MAC in the hotspot leases.
argus_dns_names() {
    local base=$1
    local prefix=${ARGUS_HOST_PREFIX:-argus}
    local found
    found=$(mktemp)
    (
        ip=
        while read -r _ ip; do
            [[ -n "$ip" ]] || continue
            (
                line=$(timeout -k 0.2 0.8 getent hosts "$ip" 2>/dev/null | head -n 1 || true)
                [[ -n "$line" ]] || exit 0
                # shellcheck disable=SC2086
                set -- $line
                shift
                host=
                short=
                for host in "$@"; do
                    short=${host%%.*}
                    if [[ "$short" == "$prefix"* ]]; then
                        printf '%s %s\n' "$short" "$ip"
                        exit 0
                    fi
                done
            ) >>"$found" &
        done < <(argus_neigh_macs "$base")
        wait || true
    )
    cat "$found"
    rm -f "$found"
}

# Lease addresses when they still answer, otherwise a scan of the attached
# network. Prints "name ip" lines. Runs in a subshell so its waits cannot
# collect SSH jobs started by the caller.
_argus_discover() (
    local prefix=${ARGUS_HOST_PREFIX:-argus}
    local -A lease_ip=()
    local -A lease_mac=()
    local -A live_ip=()
    local -a files=()
    mapfile -t files < <(argus_lease_files)
    local f name mac ip
    for f in "${files[@]}"; do
        while read -r name mac ip; do
            [[ -n "$name" && -n "$ip" ]] || continue
            lease_ip[$name]=$ip
            [[ -n "$mac" ]] && lease_mac[$name]=$mac
        done < <(awk -v p="$prefix" '$4 ~ ("^" p) {print $4, tolower($2), $3}' "$f")
    done

    local all_up=1
    [[ ${#lease_ip[@]} -eq 0 ]] && all_up=0
    for name in "${!lease_ip[@]}"; do
        ip=${lease_ip[$name]}
        if argus_addr_on_link "$ip" && ping -c 1 -W 1 -n "$ip" >/dev/null 2>&1; then
            live_ip[$name]=$ip
        else
            all_up=0
        fi
    done

    if [[ "$all_up" -eq 0 ]]; then
        local base mac
        local -A seen_base=()
        local -A mac_ip=()
        while read -r base; do
            [[ -n "$base" ]] || continue
            [[ -n "${seen_base[$base]:-}" ]] && continue
            seen_base[$base]=1
            argus_probe_prefix "$base"
            while read -r mac ip; do
                [[ -n "$mac" && -n "$ip" ]] || continue
                mac_ip[$mac]=$ip
            done < <(argus_neigh_macs "$base")
        done < <(argus_scan_bases)
        local missing=0
        [[ ${#lease_ip[@]} -eq 0 ]] && missing=1
        for name in "${!lease_ip[@]}"; do
            mac=${lease_mac[$name]:-}
            if [[ -n "$mac" && -n "${mac_ip[$mac]:-}" ]]; then
                live_ip[$name]=${mac_ip[$mac]}
            else
                missing=1
            fi
        done
        # A module that never joined the hotspot is found by reverse DNS.
        if [[ "$missing" -eq 1 ]]; then
            for base in "${!seen_base[@]}"; do
                while read -r name ip; do
                    [[ -n "$name" && -n "$ip" ]] || continue
                    live_ip[$name]=$ip
                done < <(argus_dns_names "$base")
            done
        fi
    fi

    local -A emitted=()
    for name in "${!live_ip[@]}"; do
        printf '%s %s\n' "$name" "${live_ip[$name]}"
        emitted[$name]=1
    done
    for name in "${!lease_ip[@]}"; do
        [[ -n "${emitted[$name]:-}" ]] && continue
        printf '%s %s\n' "$name" "${lease_ip[$name]}"
    done
)

# How long a scan can be reused while several module scripts start together.
_ARGUS_HOST_TTL=30

# Print one "name ip" line per host.
argus_each_host() {
    local prefix=${ARGUS_HOST_PREFIX:-argus}
    local gw cache now mtime out
    gw=$(ip -4 route show default 2>/dev/null | awk '{print $3; exit}' || true)
    gw=${gw:-none}
    cache="/tmp/argus-hosts.$(id -u).${gw}"
    if [[ -s "$cache" ]]; then
        now=$(date +%s)
        mtime=$(stat -c %Y "$cache")
        if (( now - mtime < _ARGUS_HOST_TTL )); then
            cat "$cache"
            return 0
        fi
    fi
    out=$(mktemp)
    _argus_discover | sort -V >"$out" || true
    if [[ ! -s "$out" ]]; then
        rm -f "$out" "$cache"
        echo "No ${prefix} hosts on this network or in the DHCP leases" >&2
        return 1
    fi
    cp "$out" "$cache"
    chmod 600 "$cache" || true
    cat "$out"
    rm -f "$out"
}

# Direct ssh, so a backgrounded session is the process the caller waits on.
# A new address on the shared network is accepted once.
ARGUS_SSH=(
    ssh
    -o StrictHostKeyChecking=accept-new
    -o "ConnectTimeout=${ARGUS_SSH_CONNECT_TIMEOUT:-8}"
)

# Print the IPv4 address for a module name. A literal address is returned as-is.
argus_resolve() {
    local name=$1
    if [[ "$name" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
        printf '%s\n' "$name"
        return 0
    fi
    local ip
    ip=$(argus_each_host 2>/dev/null | awk -v n="$name" '$1 == n { print $2; exit }') || true
    if [[ -n "${ip:-}" ]]; then
        printf '%s\n' "$ip"
        return 0
    fi
    ip=$(getent ahostsv4 "$name" 2>/dev/null | awk '{print $1; exit}' || true)
    if [[ -n "${ip:-}" ]]; then
        printf '%s\n' "$ip"
        return 0
    fi
    echo "Could not resolve ${name} on this network" >&2
    return 1
}
