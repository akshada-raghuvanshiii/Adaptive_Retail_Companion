#!/usr/bin/env bash
# Laptop side of a HIL session (after the stack is up with bringup.sh in HIL mode).
#   ~/arc_pi/laptop/hil_laptop.sh check    link, mode, and whether the Pi nodes are visible
#   ~/arc_pi/laptop/hil_laptop.sh up       start flag_watchdog.py and hil_logger.py (own windows/logs)
#   ~/arc_pi/laptop/hil_laptop.sh down     stop them (the logger saves its CSV)
# Interactive tools, in their own terminals (source ~/arc_test_kit/env.sh first):
#   python3 ~/arc_pi/laptop/arc_ui.py       enrol / lead me / stop keys for the Pi webcam bridge
#   python3 ~/arc_pi/laptop/view_debug.py   the Pi's annotated camera view
ARC_PI_DIR=$(cd "$(dirname "$0")/.." && pwd)
source "$ARC_PI_DIR/lib.sh"
KIT=${ARC_KIT:-$HOME/arc_test_kit}
source "$KIT/env.sh" 2>/dev/null
RUN=$HOME/arc_hil_logs; mkdir -p "$RUN/pids"

start_bg() {   # start_bg NAME command...
  local name=$1; shift
  if [ -f "$RUN/pids/$name.pid" ] && kill -0 "$(cat "$RUN/pids/$name.pid")" 2>/dev/null; then
    warn "$name already running"; return; fi
  ( exec nohup "$@" ) > "$RUN/$name.log" 2>&1 < /dev/null &
  echo $! > "$RUN/pids/$name.pid"
  sleep 2
  kill -0 $! 2>/dev/null && ok "$name started (log $RUN/$name.log)" || { bad "$name exited:"; tail -n 10 "$RUN/$name.log"; }
}

stop_bg() {
  local f=$RUN/pids/$1.pid
  [ -f "$f" ] || return 0
  kill -TERM "$(cat "$f")" 2>/dev/null && ok "$1 stopped"
  rm -f "$f"
}

case "${1:-check}" in
  check)
    [ "${ARC_NET_MODE:-local}" = hil ] && ok "env.sh: HIL mode" || bad "env.sh is in '${ARC_NET_MODE:-?}' mode: ~/arc_pi/laptop/hil_mode.sh on, then restart the stack"
    P=$(profile_own_ip)
    has_ip "$P" && ok "laptop has $P (the address in the HIL profile)" \
      || bad "laptop has no ${P:-address}: ~/arc_pi/laptop/net_laptop.sh up, then hil_mode.sh on and restart the stack"
    PI=$(peer_ip laptop)
    [ -n "$PI" ] && ping -c 1 -W 1 "$PI" >/dev/null 2>&1 && ok "Pi answers at $PI" || bad "no answer from the Pi (${PI:-$PI_HOST not found}): net_laptop.sh status"
    Q=$(profile_peer_ip); [ -n "$Q" ] && [ -n "$PI" ] && [ "$Q" != "$PI" ] && warn "Pi moved to $PI (profile has $Q): hil_mode.sh on, restart the stack"
    lo_multicast_ok && ok "lo has MULTICAST" || bad "lo has no MULTICAST: sudo ip link set lo multicast on && ros2 daemon stop"
    ros2 daemon stop >/dev/null 2>&1
    T=$(timeout 15 ros2 topic list 2>/dev/null)
    for t in /cmd_vel /arc/behavior_mode; do
      echo "$T" | grep -qx "$t" && ok "laptop stack: $t" || warn "laptop stack: no $t (bringup.sh nav done?)"; done
    for t in /arc_hw/wheels /arc/target_lock /arc/camera/image/compressed; do
      echo "$T" | grep -qx "$t" && ok "from the Pi: $t" || echo "  from the Pi: $t not seen (that node not started?)"; done
    ;;
  up)
    [ "${ARC_NET_MODE:-local}" = hil ] || warn "not in HIL mode: the Pi will not be seen"
    start_bg watchdog python3 "$ARC_PI_DIR/laptop/flag_watchdog.py" --timeout "${WATCHDOG_S:-3}"
    start_bg logger python3 "$ARC_PI_DIR/laptop/hil_logger.py" ${TWIST:+--twist} --out-dir "$RUN"
    ;;
  down) stop_bg logger; stop_bg watchdog
    L=$(ls -t "$RUN"/chain_*.csv 2>/dev/null | head -1)
    [ -n "$L" ] && say "latest log: $L   plot: python3 $ARC_PI_DIR/laptop/plot_hil_timeline.py $L" ;;
  *) echo "usage: $0 check|up|down"; exit 2 ;;
esac
