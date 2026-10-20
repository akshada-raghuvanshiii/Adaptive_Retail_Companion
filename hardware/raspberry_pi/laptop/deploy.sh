#!/usr/bin/env bash
# Copy everything the Pi runs from the laptop to the Pi (rsync over ssh). Run ON THE LAPTOP:
#   ~/arc_pi/laptop/deploy.sh            code + arc_pi kit + weights
#   ~/arc_pi/laptop/deploy.sh --dry-run  show what would be copied
#
# On the Pi:  ~/arc_pi            this kit (scripts, hil.conf)
#             ~/arc_pi_run/arc-perception   ros/, arc_assist/, tools/live_lock.py, configs/, yolov8n.pt
#             ~/arc_pi_run/arc_hw           bridge_node.py, bench.py, arc_hw/, plot_hil.py, tests/
# Logs on the Pi (~/arc_pi_run/logs, hil_logs, perf) are never deleted by this. Nothing in ~/slam_ws is
# copied: the ARC navigation stack and arc_person_follower stay on the laptop only.
ARC_PI_DIR=$(cd "$(dirname "$0")/.." && pwd)
source "$ARC_PI_DIR/lib.sh"
DRY=(); [ "${1:-}" = --dry-run ] && DRY=(--dry-run)
DEST=$PI_USER@$PI_HOST
command -v rsync >/dev/null || die "rsync missing on the laptop (sudo apt install rsync)"
[ -d "$PERCEPTION_DIR/ros" ] || die "PERCEPTION_DIR=$PERCEPTION_DIR has no ros/ (edit hil.conf)"
[ -f "$HW_DIR/bridge_node.py" ] || die "HW_DIR=$HW_DIR has no bridge_node.py (edit hil.conf)"
[ -f "$PERCEPTION_DIR/ros/perception_bridge.py" ] && grep -q "arc/ui/command" "$PERCEPTION_DIR/ros/perception_bridge.py" \
  || die "$PERCEPTION_DIR/ros/perception_bridge.py is the old version (no /arc/ui/command): install the update first (install_updates.sh)"
say "checking ssh to $DEST ..."
ssh -o ConnectTimeout=5 -o BatchMode=yes "$DEST" true 2>/dev/null || {
  warn "ssh without a password failed. Either type the password at each prompt below, or set up a key once:"
  echo "      ssh-keygen -t ed25519   (Enter for all)   then   ssh-copy-id $DEST"
  ssh -o ConnectTimeout=5 "$DEST" true || die "cannot ssh to $DEST (net_laptop.sh status; Pi booted? on the same hotspot? SSH enabled in Imager?)"
}
X=(--exclude __pycache__ --exclude "*.pyc" --exclude .git --exclude .venv)
ssh "$DEST" "mkdir -p ~/arc_pi ~/arc_pi_run/arc-perception ~/arc_pi_run/arc_hw ~/arc_pi_run/logs" || die "mkdir on the Pi failed"
rsync -a "${DRY[@]}" "${X[@]}" --exclude updates --exclude hil.peer "$ARC_PI_DIR/" "$DEST:arc_pi/" || die "rsync arc_pi failed"
# Tell the Pi how to find this laptop on the hotspot: its mDNS name, plus the address it has right now
LH="$(hostname).local"; LI=$(own_ip laptop)
if [ ${#DRY[@]} = 0 ]; then
  ssh "$DEST" "printf 'LAPTOP_HOST=%s\nLAPTOP_IP_SEEN=%s\n' '$LH' '$LI' > ~/arc_pi/hil.peer" || die "could not write hil.peer on the Pi"
  ok "Pi will look for this laptop as $LH (now at ${LI:-?})"
fi
rsync -a "${DRY[@]}" "${X[@]}" --relative \
  "$PERCEPTION_DIR/./ros/perception_bridge.py" "$PERCEPTION_DIR/./arc_assist" "$PERCEPTION_DIR/./tools/live_lock.py" \
  "$PERCEPTION_DIR/./configs" "$DEST:arc_pi_run/arc-perception/" || die "rsync perception failed"
if [ -f "$WEIGHTS" ]; then
  rsync -a "${DRY[@]}" "$WEIGHTS" "$DEST:arc_pi_run/arc-perception/yolov8n.pt" || die "rsync weights failed"
else
  warn "weights not found at $WEIGHTS: the Pi will download yolov8n.pt on first run (needs internet there)"
fi
rsync -a "${DRY[@]}" "${X[@]}" --exclude hil_logs "$HW_DIR/" "$DEST:arc_pi_run/arc_hw/" || die "rsync arc_hw failed"
ok "deployed to $DEST${DRY:+ (dry run)}. On the Pi:  ~/arc_pi/pi/pi_check.sh"
