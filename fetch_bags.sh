#!/usr/bin/env bash
set -Eeuo pipefail

# Copy ~/bags/ from a Pi into ~/bags/argusX/ on this machine.
# Usage: fetch_bags.sh argus2

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 argusX" >&2
    exit 1
fi

name="$1"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/argus_common.sh"
ip=$(argus_resolve "$name")

dest="${HOME}/bags/${name}"
mkdir -p "$dest"
echo "Copying bags from ${name} (${ip}) to ${dest}"
rsync -avh --progress "${ARGUS_SSH_USER}@${ip}:bags/" "${dest}/"
