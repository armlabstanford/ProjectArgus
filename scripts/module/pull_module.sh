#!/usr/bin/env bash
set -Eeuo pipefail

# git pull ~/ArgusModule on a Pi, then colcon build it in Release.
# Fast-forward only, so local edits or diverged history stop the pull
# instead of creating a merge. A failed pull skips the build.
# Usage: pull_module.sh argus2

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 argusX" >&2
    exit 1
fi

name="$1"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")/.." && pwd)/argus_common.sh"
ip=$(argus_resolve "$name")

echo "[${name}] Pulling and building ~/ArgusModule (${ip})"

remote=$(cat <<'EOF'
set -Eeo pipefail
if [[ ! -d "$HOME/ArgusModule/.git" ]]; then
    echo "~/ArgusModule is not a git repo" >&2
    exit 1
fi
cd "$HOME/ArgusModule"
git pull --ff-only
echo "Now at $(git rev-parse --abbrev-ref HEAD) $(git log -1 --format='%h %s')"
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Build finished"
EOF
)

# Prefix every line so output from several modules can be told apart.
"${ARGUS_SSH[@]}" -o BatchMode=yes -o ConnectTimeout=5 "${ARGUS_SSH_USER}@${ip}" \
    "bash -lc $(printf '%q' "$remote")" 2>&1 | sed -u "s/^/[${name}] /"
