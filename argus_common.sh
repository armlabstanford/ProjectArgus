# Shared lookup for the Argus scripts. Source this file; do not execute it.

_argus_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
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

# Print one "name ip" line per host. A name in several lease files keeps the
# address that answers ping.
argus_each_host() {
    local prefix=${ARGUS_HOST_PREFIX:-argus}
    local -a files=()
    mapfile -t files < <(argus_lease_files)
    if [[ ${#files[@]} -eq 0 ]]; then
        echo "No DHCP lease file found" >&2
        return 1
    fi
    local -A chosen=()
    local f name ip
    for f in "${files[@]}"; do
        while read -r name ip; do
            [[ -n "$name" && -n "$ip" ]] || continue
            if [[ -z "${chosen[$name]:-}" ]]; then
                chosen[$name]=$ip
            elif ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
                chosen[$name]=$ip
            fi
        done < <(awk -v p="$prefix" '$4 ~ ("^" p) {print $4, $3}' "$f")
    done
    if [[ ${#chosen[@]} -eq 0 ]]; then
        echo "No ${prefix} hosts in the DHCP leases" >&2
        return 1
    fi
    local key
    for key in "${!chosen[@]}"; do
        printf '%s %s\n' "$key" "${chosen[$key]}"
    done | sort -V
}

# Print the IPv4 address for a module name. A literal address is returned as-is.
argus_resolve() {
    local name=$1
    if [[ "$name" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
        printf '%s\n' "$name"
        return 0
    fi
    local -a files=()
    mapfile -t files < <(argus_lease_files)
    local -a cands=()
    local f ip
    for f in "${files[@]}"; do
        while read -r ip; do
            [[ -n "$ip" ]] && cands+=("$ip")
        done < <(awk -v n="$name" '$4 == n {print $3}' "$f")
    done
    local c
    for c in "${cands[@]}"; do
        if ping -c 1 -W 1 "$c" >/dev/null 2>&1; then
            printf '%s\n' "$c"
            return 0
        fi
    done
    if [[ ${#cands[@]} -gt 0 ]]; then
        printf '%s\n' "${cands[-1]}"
        return 0
    fi
    ip=$(getent ahostsv4 "$name" 2>/dev/null | awk '{print $1; exit}' || true)
    if [[ -n "$ip" ]]; then
        printf '%s\n' "$ip"
        return 0
    fi
    echo "Could not resolve ${name} from DHCP leases" >&2
    return 1
}
