#!/usr/bin/env bash
# Clear a persistent firewall on an Ubuntu Pi root filesystem so SSH answers
# again. Pass the mount point of the ext4 "writable" partition.
set -Eeuo pipefail

ROOT="${1:-}"
if [[ -z "$ROOT" || ! -d "$ROOT/etc/systemd/system" || ! -d "$ROOT/etc/ssh" ]]; then
    echo "Usage: sudo $0 /mnt/pi" >&2
    echo "Mount the SD card's ext4 root (label writable) first." >&2
    exit 1
fi

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo so the card's root-owned files can be changed." >&2
    exit 1
fi

mask_unit() {
    local unit="$1"
    ln -sfn /dev/null "$ROOT/etc/systemd/system/${unit}"
    echo "masked ${unit}"
}

for unit in ufw.service nftables.service netfilter-persistent.service iptables.service ip6tables.service fail2ban.service tailscaled.service; do
    if [[ -e "$ROOT/lib/systemd/system/${unit}" || -e "$ROOT/etc/systemd/system/${unit}" || -L "$ROOT/etc/systemd/system/${unit}" ]]; then
        mask_unit "$unit"
    fi
done

if [[ -f "$ROOT/etc/ufw/ufw.conf" ]]; then
    sed -i 's/^ENABLED=.*/ENABLED=no/' "$ROOT/etc/ufw/ufw.conf"
    echo "set ufw ENABLED=no"
fi

for rules in "$ROOT/etc/iptables/rules.v4" "$ROOT/etc/iptables/rules.v6"; do
    if [[ -f "$rules" ]]; then
        mv "$rules" "${rules}.bak"
        echo "moved aside ${rules#"$ROOT"}"
    fi
done

install -d "$ROOT/etc/systemd/system/multi-user.target.wants"
cat > "$ROOT/etc/systemd/system/accept-ssh.service" <<'EOF'
[Unit]
Description=Accept traffic before sshd starts
DefaultDependencies=no
After=local-fs.target
Before=ssh.service sshd.service network-pre.target
ConditionPathExists=/usr/sbin/iptables

[Service]
Type=oneshot
ExecStart=/bin/sh -c '/usr/sbin/iptables -P INPUT ACCEPT; /usr/sbin/iptables -P FORWARD ACCEPT; /usr/sbin/iptables -P OUTPUT ACCEPT; /usr/sbin/iptables -F; /usr/sbin/iptables -t nat -F; /usr/sbin/iptables -t mangle -F; /usr/sbin/ip6tables -P INPUT ACCEPT; /usr/sbin/ip6tables -F; if [ -x /usr/sbin/nft ]; then /usr/sbin/nft flush ruleset; fi'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
ln -sfn ../accept-ssh.service "$ROOT/etc/systemd/system/multi-user.target.wants/accept-ssh.service"
echo "installed accept-ssh.service"

if [[ ! -e "$ROOT/etc/systemd/system/multi-user.target.wants/ssh.service" && -e "$ROOT/lib/systemd/system/ssh.service" ]]; then
    ln -sfn /lib/systemd/system/ssh.service "$ROOT/etc/systemd/system/multi-user.target.wants/ssh.service"
    echo "enabled ssh.service"
fi

install -d "$ROOT/etc/cloud/cloud.cfg.d"
printf 'network: {config: disabled}\n' > "$ROOT/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg"

shopt -s nullglob
for netplan in "$ROOT/etc/netplan/"*.yaml; do
    if grep -q 'ARM_LAB' "$netplan"; then
        mv "$netplan" "${netplan}.bak"
        echo "moved aside ${netplan#"$ROOT"}"
    fi
done
cat > "$ROOT/etc/netplan/99-ethernet-dhcp.yaml" <<'EOF'
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      optional: true
EOF
echo "wrote ethernet-only netplan"

echo
echo "Unmount the card, put it back in the Pi, and boot."
echo "Then: ssh argus@10.42.1.89"
