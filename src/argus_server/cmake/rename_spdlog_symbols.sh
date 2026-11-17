#!/bin/bash
# Rename spdlog/fmt symbols in an object file so they bind to OpenSim's
# private copy (prefixed osrt_) instead of ROS Humble's libspdlog 1.9.
set -euo pipefail

obj="${1:?object file required}"
names="$(mktemp)"
map="$(mktemp)"
trap 'rm -f "$names" "$map"' EXIT

nm "$obj" | awk '{print $NF}' | grep -E '(6spdlog|3fmt)' | grep -v '^osrt_' | sort -u > "$names" || true
if [ ! -s "$names" ]; then
  exit 0
fi

while read -r sym; do
  printf '%s osrt_%s\n' "$sym" "$sym"
done < "$names" > "$map"

objcopy --redefine-syms="$map" "$obj"
