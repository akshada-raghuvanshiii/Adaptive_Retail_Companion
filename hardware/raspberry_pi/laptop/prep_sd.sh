#!/usr/bin/env bash
# OPTIONAL, before the Pi's first boot: give it the static cable address straight on the SD card,
# so you can ssh to it over the Ethernet cable at first boot (no router, screen or keyboard needed).
#
# 1. Flash "Ubuntu Server 24.04 LTS (64-bit)" with Raspberry Pi Imager. In its settings set:
#    hostname arc-pi, user (PI_USER in hil.conf), password, Wi-Fi (for internet), and ENABLE SSH.
# 2. Take the card out and put it back in: the laptop mounts the "system-boot" partition.
# 3. ./prep_sd.sh            (finds /media/$USER/system-boot; or ./prep_sd.sh /path/to/system-boot)
#
# It adds eth0 with PI_IP/PREFIX to the card's cloud-init "network-config" (DHCP kept on, Wi-Fi
# untouched) and keeps a backup next to it. Nothing on the laptop changes. No packages installed.
ARC_PI_DIR=$(cd "$(dirname "$0")/.." && pwd)
source "$ARC_PI_DIR/lib.sh"
if [ "$LINK" = hotspot ]; then
  say "LINK=hotspot: nothing to add to the card. In Raspberry Pi Imager, set the Wi-Fi to your phone"
  say "hotspot's name and password (plus hostname arc-pi, user $PI_USER, and enable SSH). Then boot the Pi,"
  say "keep the phone hotspot on with the laptop joined, and:  ~/arc_pi/laptop/net_laptop.sh up"
  exit 0
fi
BOOT=${1:-/media/$USER/system-boot}
NC=$BOOT/network-config
[ -f "$NC" ] || die "no $NC. Is the SD card mounted? (lsblk -f; the partition is labelled system-boot)"
grep -q "$PI_IP" "$NC" && { ok "network-config already has $PI_IP"; exit 0; }
say "This edits $NC: eth0 gets $PI_IP/$PREFIX (DHCP stays on, Wi-Fi settings unchanged)."
ask "Edit it (a backup is kept as network-config.bak_arc)?" || die "not changed"
cp "$NC" "$NC.bak_arc"
python3 - "$NC" "$PI_IP/$PREFIX" <<'PY' || { cp "$NC.bak_arc" "$NC"; die "could not edit; restored the backup"; }
import sys, yaml
path, addr = sys.argv[1], sys.argv[2]
with open(path) as f:
    d = yaml.safe_load(f) or {}
net = d.setdefault("network", {"version": 2})
net.setdefault("version", 2)
eth = net.setdefault("ethernets", {})
e0 = eth.setdefault("eth0", {})
e0["dhcp4"] = True
e0["optional"] = True
addrs = e0.setdefault("addresses", [])
if addr not in addrs:
    addrs.append(addr)
with open(path, "w") as f:
    yaml.safe_dump(d, f, default_flow_style=False, sort_keys=False)
print(open(path).read())
PY
sync
ok "done. Eject the card, boot the Pi with the Ethernet cable to the laptop, then:"
echo "    ~/arc_pi/laptop/net_laptop.sh up      # laptop gets $LAPTOP_IP"
echo "    ssh $PI_USER@$PI_IP                    # first boot takes 2-5 min (cloud-init)"
