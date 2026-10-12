#!/usr/bin/env bash
set -Eeuo pipefail

# Delete everything in ~/bags/ on a Pi.
# Usage: clear_bags.sh argus2

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 argusX" >&2
    exit 1
fi

name="$1"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/argus_common.sh"
ip=$(argus_resolve "$name")

echo "Clearing ~/bags on ${name} (${ip})"

remote=$(cat <<'EOF'
set -Eeo pipefail
if [[ ! -d "$HOME/bags" ]]; then
    echo "~/bags does not exist"
    exit 0
fi
shopt -s nullglob
entries=("$HOME/bags"/*)
if [[ ${#entries[@]} -eq 0 ]]; then
    echo "~/bags is already empty"
    exit 0
fi
rm -rf -- "${entries[@]}"
echo "Removed ${#entries[@]} entries from ~/bags"
EOF
)

ssh "${ARGUS_SSH_USER}@${ip}" "bash -lc $(printf '%q' "$remote")"
