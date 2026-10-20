#!/usr/bin/env bash
# Put the Pi-related updates into the laptop's existing copies, keeping a backup of every file it
# replaces (<file>.bak_<time>), then run each project's offline tests. Run ON THE LAPTOP:
#   ~/arc_pi/install_updates.sh            (asks before copying)
#
# What changes and why:
#   ~/arc-perception/ros/perception_bridge.py  headless on the Pi: /arc/ui/command buttons, debug JPEG,
#                                             camera size/MJPG, frames from a ROS topic, SIGTERM stop
#   ~/arc-perception/ros/README.md, tests/test_bridge_pi.py
#   ~/arc_hw/bridge_node.py, README.md        SIGTERM = Ctrl+C (sends the Uno a stop), --log-dir; Pi note
#   ~/arc_test_kit/env.sh                     reads ~/.arc_net; WITHOUT that file it is exactly as before
#   ~/arc_test_kit/bringup.sh                 HIL link check, PERSON_INPUT=camera (skip B6)
#   ~/arc_test_kit/README.md, VERSION         v19 note
# Nothing in ~/slam_ws is touched. No packages are installed.
ARC_PI_DIR=$(cd "$(dirname "$0")" && pwd)
source "$ARC_PI_DIR/lib.sh"
U=$ARC_PI_DIR/updates
TS=$(date +%Y%m%d_%H%M%S)
declare -A DEST=([arc-perception]=${PERCEPTION_DIR:-$HOME/arc-perception} [arc_hw]=${HW_DIR:-$HOME/arc_hw}
                 [arc_test_kit]=${ARC_KIT:-$HOME/arc_test_kit})

if pgrep -f 'mission_runner.py' >/dev/null; then
  warn "a test run (mission_runner.py) is going. env.sh/bringup.sh only change NEW terminals, and without"
  warn "~/.arc_net they behave exactly as before, so the run is not affected. A chained next run will use"
  warn "the new bringup.sh (same behaviour). Safer still: install after the run."
  ask "Install now anyway?" || exit 0
fi
for p in "${!DEST[@]}"; do
  [ -d "${DEST[$p]}" ] || die "${DEST[$p]} not found (set PERCEPTION_DIR / HW_DIR / ARC_KIT or edit hil.conf)"
done
say "Files to update:"
CHANGED=()
while IFS= read -r f; do
  rel=${f#"$U"/}; proj=${rel%%/*}; sub=${rel#*/}; dst=${DEST[$proj]}/$sub
  if [ -f "$dst" ] && cmp -s "$f" "$dst"; then echo "  same      $dst"; continue; fi
  echo "  update    $dst"; CHANGED+=("$f|$dst")
done < <(find "$U" -type f | sort)
[ ${#CHANGED[@]} -gt 0 ] || { ok "everything already up to date"; exit 0; }
ask "Copy these (backups kept as .bak_$TS)?" || die "nothing changed"
for c in "${CHANGED[@]}"; do
  src=${c%%|*}; dst=${c#*|}
  mkdir -p "$(dirname "$dst")"
  [ -f "$dst" ] && cp -p "$dst" "$dst.bak_$TS"
  cp "$src" "$dst"
  case "$dst" in *.sh) chmod +x "$dst" ;; esac
done
ok "copied ${#CHANGED[@]} file(s)"

say "offline tests:"
PY=python3; [ -x "$HOME/arc_perception/.venv/bin/python" ] && PY=$HOME/arc_perception/.venv/bin/python
( cd "${DEST[arc-perception]}" && $PY -m unittest discover -s tests 2>&1 | tail -1 ) | sed 's/^/  arc-perception: /'
( cd "${DEST[arc_hw]}" && python3 -m unittest discover -s tests 2>&1 | tail -1 ) | sed 's/^/  arc_hw:         /'
( cd "${DEST[arc_test_kit]}" && python3 -m unittest discover -s tests 2>&1 | tail -1 ) | sed 's/^/  arc_test_kit:   /'
( cd "$ARC_PI_DIR" && ARC_TEST_KIT=${DEST[arc_test_kit]} python3 -m unittest discover -s tests 2>&1 | tail -1 ) | sed 's/^/  arc_pi:         /'
say "Undo: for each file, copy its .bak_$TS back."
