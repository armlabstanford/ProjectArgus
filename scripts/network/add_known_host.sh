#!/usr/bin/env bash
set -Eeuo pipefail

# Install this machine's SSH key on another device.
# Usage: add_known_host.sh <ip> <password> [remote-user]
# remote-user defaults to the current user.

if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "Usage: $0 <ip> <password> [remote-user]" >&2
    exit 1
fi

IP="$1"
PASSWORD="$2"
REMOTE_USER="${3:-$USER}"

install -d -m 700 "$HOME/.ssh"
if [[ ! -f "$HOME/.ssh/id_ed25519.pub" && ! -f "$HOME/.ssh/id_rsa.pub" ]]; then
    ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519"
fi

ssh-keygen -R "$IP" >/dev/null 2>&1 || true

askpass=$(mktemp)
chmod 700 "$askpass"
{
    printf '%s\n' '#!/bin/sh'
    printf "printf '%%s\\n' %s\n" "$(printf '%q' "$PASSWORD")"
} > "$askpass"
trap 'rm -f "$askpass"' EXIT

SSH_ASKPASS="$askpass" SSH_ASKPASS_REQUIRE=force \
    ssh-copy-id \
        -o StrictHostKeyChecking=accept-new \
        -o PreferredAuthentications=password \
        -o PubkeyAuthentication=no \
        "${REMOTE_USER}@${IP}"

echo "Key installed for ${REMOTE_USER}@${IP}"
echo "Connect with: ssh ${REMOTE_USER}@${IP}"
