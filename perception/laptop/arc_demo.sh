#!/usr/bin/env bash
# ARC everything together, from YOUR LAPTOP: webcam on the Pi -> BT -> /cmd_vel -> real motors.
#
#   ~/arc_pi_udp/laptop/arc_demo.sh up              bring-up (~15 min) + all windows + the Pi bridge over ssh
#   ~/arc_pi_udp/laptop/arc_demo.sh up --no-bringup the stack is already up: only the windows
#   ~/arc_pi_udp/laptop/arc_demo.sh stop            motors stop first, then everything else
#
# Windows it opens (gnome-terminal), each with its own log in ~/arc_logs/demo_<time>/:
#   ARC_relay     Pi UDP -> /person_moving, /person_confused (holds the BT in WAIT if the Pi goes quiet)
#   ARC_motors    /cmd_vel -> Uno -> wheels (arc_hw_bridge, your arc_hw.yaml)
#   ARC_chain     one live line: webcam | flags | BT | cmd_vel | PWM | wheels
#   ARC_recorder  hil_logger.py -> ~/arc_hil_logs/chain_<time>.csv (for plot_hil_timeline.py)
#   ARC_buttons   shopper keys: e enrol, l lead me, r reset   (click into it before pressing keys)
#   ARC_view      the Pi's annotated camera view
#   ARC_pi        ssh to the Pi running run_on_pi.sh (type the Pi's password there)
#   ARC_shopping  Shopping Manager (enter the list here)
# Settings (environment): PI=manav@172.20.10.2  CAM=0  IMGSZ=320
#   PERC=<IP>  perception runs on ANOTHER COMPUTER at that address (not the Pi): no ARC_pi window,
#              the relay accepts data only from PERC, and any shared network works (not only the
#              iPhone hotspot). Start the bridge on that computer with run_perception.sh / .bat.
# Nothing in ~/slam_ws is changed. B6 (person_state_input) is not used: the webcam replaces it.
HERE=$(cd "$(dirname "$0")" && pwd)
PI=${PI:-manav@172.20.10.2}
CAM=${CAM:-0}
IMGSZ=${IMGSZ:-320}
YAML=$HOME/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml
VENV_PY=$HOME/arc_perception/.venv/bin/python
ok()   { echo -e "\033[1;32m[  OK  ]\033[0m $*"; }
warn() { echo -e "\033[1;33m[ WARN ]\033[0m $*"; }
bad()  { echo -e "\033[1;31m[ FAIL ]\033[0m $*"; }
OURS="perception_relay.py|chain_monitor.py|hil_logger.py|arc_ui.py|view_debug.py|arc_hw.launch.py|motor_bridge|shopping_manager|run_on_pi.sh"

stop_all() {
  echo "stopping: motors first (they send STOP to the Uno), then the rest"
  pkill -INT -f "arc_hw.launch.py|motor_bridge" 2>/dev/null; sleep 2
  pkill -INT -f "hil_logger.py" 2>/dev/null; sleep 1          # saves the chain CSV
  pkill -INT -f "$OURS" 2>/dev/null; sleep 1
  pkill -f "$OURS" 2>/dev/null
  ~/arc_test_kit/bringup.sh stop
  L=$(ls -t ~/arc_hil_logs/chain_*.csv 2>/dev/null | head -1)
  [ -n "$L" ] && echo "chain log: $L   plot:  python3 $HERE/plot_hil_timeline.py $L"
  M=$(ls -t ~/arc_hw_logs/hil_*.csv 2>/dev/null | head -1)
  [ -n "$M" ] && echo "motor log: $M   plot:  python3 ~/arc_hw/plot_hil.py $M"
}

win() {   # win TITLE "command" [wait s] [tty]   sources env.sh; window stays open on exit so errors stay readable
  local title=$1 cmd=$2 log=$LOGDIR/$1.log run
  if [ "${4:-}" = tty ]; then run="$cmd"                       # interactive (keys, password, typed list): no tee
  else run="( $cmd ) 2>&1 | tee '$log'"; fi
  gnome-terminal --title="$title" -- bash -c "source ~/arc_test_kit/env.sh; export PYTHONUNBUFFERED=1; echo '== $title'; $run; echo; echo '[$title exited - press Enter]'; read" &
  sleep "${3:-2}"
}

case "${1:-}" in
  stop) stop_all; exit 0 ;;
  up) ;;
  *) sed -n '2,20p' "$0"; exit 2 ;;
esac

# ---------- checks (nothing is changed)
command -v gnome-terminal >/dev/null || { bad "gnome-terminal not found"; exit 1; }
ip link show lo | grep -q MULTICAST || { bad "loopback multicast off: sudo ip link set lo multicast on && ros2 daemon stop"; exit 1; }
if [ -n "${PERC:-}" ]; then
  PI_HOST=$PERC
  LAPTOP_IP=$(hostname -I | tr ' ' '\n' | grep -m1 "^${PERC%.*}\.")
  if [ -n "$LAPTOP_IP" ]; then ok "same network as the perception laptop: this laptop is $LAPTOP_IP"
  else bad "no address on ${PERC%.*}.x here: join the same Wi-Fi/hotspot as the perception laptop (addresses now: $(hostname -I))"; exit 1; fi
else
LAPTOP_IP=$(hostname -I | tr ' ' '\n' | grep -m1 '^172\.20\.10\.')
if [ -n "$LAPTOP_IP" ]; then ok "on the iPhone hotspot as $LAPTOP_IP"
else
  bad "this laptop has no 172.20.10.x address: join Manav's iPhone hotspot first (addresses now: $(hostname -I))"; exit 1
fi
PI_HOST=${PI#*@}
fi
ping -c 1 -W 2 "$PI_HOST" >/dev/null 2>&1 && ok "perception computer answers at $PI_HOST" || warn "no ping reply from $PI_HOST (on? same network? Windows blocks ping by default, which is harmless)"
PORT=$(awk '/port:/{print $2; exit}' "$YAML" 2>/dev/null)
[ -n "$PORT" ] && [ -e "$PORT" ] && ok "Uno at $PORT" || { bad "Uno port '${PORT:-?}' from $YAML not found (ls /dev/ttyACM*; fix 'port:' in the YAML)"; exit 1; }
source ~/arc_test_kit/env.sh 2>/dev/null
ros2 pkg prefix arc_hw_bridge >/dev/null 2>&1 && ok "arc_hw_bridge installed" || { bad "arc_hw_bridge not built (cd ~/slam_ws && colcon build --packages-select arc_hw_bridge)"; exit 1; }
[ -x "$VENV_PY" ] || warn "no $VENV_PY: the camera view window will fail (the rest works)"
if pgrep -f "$OURS" >/dev/null; then warn "demo windows from an earlier run are still open: run '$0 stop' first"; exit 1; fi

# ---------- stack
if [ "${2:-}" != "--no-bringup" ]; then
  PERSON_INPUT=camera ~/arc_test_kit/bringup.sh all || { bad "bring-up failed (see above)"; exit 1; }
fi
if pgrep -f person_state_input >/dev/null; then
  pkill -INT -f person_state_input; sleep 1; pkill -f person_state_input
  ok "closed B6 person_state_input (the webcam drives the BT; your bringup.sh version still starts B6)"
fi

LOGDIR=$HOME/arc_logs/demo_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOGDIR"
win ARC_relay    "python3 $HERE/perception_relay.py --pi-ip $PI_HOST" 3     # first: holds WAIT from now on
win ARC_motors   "ros2 launch arc_hw_bridge arc_hw.launch.py params_file:=$YAML" 4
win ARC_chain    "python3 $HERE/chain_monitor.py"
win ARC_recorder "python3 $HERE/hil_logger.py"
win ARC_view     "$VENV_PY $HERE/view_debug.py"
win ARC_buttons  "python3 $HERE/arc_ui.py" 2 tty
if [ -z "${PERC:-}" ]; then
win ARC_pi       "ssh -t $PI 'export LC_ALL=C.UTF-8; IMGSZ=$IMGSZ ~/arc_pi_udp/pi/run_on_pi.sh $LAPTOP_IP $CAM'" 3 tty
fi
win ARC_shopping "ros2 run arc_planner shopping_manager" 1 tty
echo
ok "all windows started (logs: $LOGDIR)"
if [ -n "${PERC:-}" ]; then
echo "  1. On the perception laptop run:  run_perception.sh $LAPTOP_IP   (Windows: run_perception.bat $LAPTOP_IP)"
echo "     ARC_relay should then say 'receiving from the Pi at $PI_HOST' (the relay calls any sender 'the Pi')."
else
echo "  1. ARC_pi: type the Pi's password. ARC_relay should then say 'receiving from the Pi at $PI_HOST'."
fi
echo "  2. Stand alone in front of the webcam. Click ARC_buttons, press e (enrol), then l (lead me)."
echo "  3. ARC_shopping: enter an item. Gazebo cart and real wheels drive; ARC_chain shows every stage."
echo "  4. Step out of view: within a few seconds BT -> WAIT, cmd_vel 0, PWM 0, wheels stop."
echo "  Stop everything:  $0 stop"
