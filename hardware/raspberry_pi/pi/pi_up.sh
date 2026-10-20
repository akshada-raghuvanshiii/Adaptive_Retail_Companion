#!/usr/bin/env bash
# Start/stop the ARC nodes ON THE PI (each in the background with its own log).
#
#   ~/arc_pi/pi/pi_up.sh motors       /cmd_vel from the laptop -> Uno -> wheels   (arc_hw/bridge_node.py)
#   ~/arc_pi/pi/pi_up.sh perception   webcam -> YOLO on the Pi -> /person_moving, /person_confused
#   ~/arc_pi/pi/pi_up.sh stream       webcam -> JPEG frames to the laptop only (YOLO runs on the laptop:
#                                     perception_bridge.py --camera ros:/arc/camera/image/compressed)
#   ~/arc_pi/pi/pi_up.sh all          motors + perception
#   ~/arc_pi/pi/pi_up.sh status | logs [name] | down [name]
#
# Stop = SIGTERM: the webcam bridge publishes "wait" (false/false) and the motor bridge sends the Uno
# a stop before exiting. Settings come from ~/arc_pi/hil.conf. Wheels OFF THE GROUND for HIL tests.
ARC_PI_DIR=$(cd "$(dirname "$0")/.." && pwd)
source "$ARC_PI_DIR/pi/pi_env.sh"
PIDS=$ARC_RUN/pids
LOGS=$ARC_RUN/logs
mkdir -p "$PIDS" "$LOGS" "$ARC_RUN/hil_logs"

running() { [ -f "$PIDS/$1.pid" ] && kill -0 "$(cat "$PIDS/$1.pid")" 2>/dev/null; }

start() {   # start NAME command...
  local name=$1; shift
  if running "$name"; then warn "$name already running (pid $(cat "$PIDS/$name.pid"))"; return 0; fi
  local log; log=$LOGS/${name}_$(date +%Y%m%d_%H%M%S).log
  ( cd "$ARC_RUN" && exec nohup "$@" ) > "$log" 2>&1 < /dev/null &   # same PID, survives ssh logout
  echo $! > "$PIDS/$name.pid"
  ln -sf "$log" "$LOGS/$name.latest.log"
  sleep 3
  if running "$name"; then ok "$name started (pid $!), log: $log"
  else bad "$name exited at once. Last lines of $log:"; tail -n 15 "$log"; return 1; fi
}

stop() {
  local name=$1
  running "$name" || { rm -f "$PIDS/$name.pid"; return 0; }
  local pid; pid=$(cat "$PIDS/$name.pid")
  kill -TERM "$pid" 2>/dev/null
  for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
  if kill -0 "$pid" 2>/dev/null; then warn "$name did not stop in 6 s, killing"; kill -KILL "$pid"; fi
  rm -f "$PIDS/$name.pid"; ok "$name stopped"
}

prechecks() {
  lo_multicast_ok || warn "lo has no MULTICAST: the Pi nodes may not see each other. Fix: sudo ip link set lo multicast on"
  local me; me=$(own_ip pi)
  has_ip "$me" || warn "no ${LINK} address on this Pi${me:+ ($me)}: the laptop will not see these nodes (pi_check.sh net)"
  [ -n "$(profile_peer_ip)" ] || warn "laptop address unknown: discovery by multicast only (run deploy.sh on the laptop; pi_check.sh net)"
  ros2 daemon stop >/dev/null 2>&1
}

camera_free() {
  for n in perception stream; do
    [ "$n" != "$1" ] && running "$n" && die "'$n' already uses the camera. Stop it first: pi_up.sh down $n"
  done
  return 0
}

up_motors() {
  [ -e "$SERIAL_PORT" ] || die "$SERIAL_PORT not found (Uno plugged into the Pi?)"
  pgrep -f 'bench.py' >/dev/null && die "bench.py is using the Uno; close it first"
  start motors python3 "$ARC_RUN/arc_hw/bridge_node.py" --port "$SERIAL_PORT" \
    --wheel "$WHEEL_D" --track "$TRACK" --ppr "$PPR" --pwm-min "$PWM_MIN" --ff "$FF" \
    --max-wheel "$MAX_WHEEL" --log-dir "$ARC_RUN/hil_logs"
}

up_perception() {
  camera_free perception
  [ -x "$ARC_VENV/bin/python" ] || die "no venv: pi_setup.sh venv"
  local model=${PI_MODEL:-$ARC_RUN/arc-perception/yolov8n.pt}
  [ -e "$model" ] || die "model not found: $model (deploy.sh copies yolov8n.pt)"
  start perception "$ARC_VENV/bin/python" "$ARC_RUN/arc-perception/ros/perception_bridge.py" --headless \
    --camera "$CAMERA" --width "$CAM_W" --height "$CAM_H" --mjpg --imgsz "$IMGSZ" --model "$model" \
    --max-lost-s 4 --debug-image-hz 2
  say "Enrol the shopper from the laptop: ~/arc_pi/laptop/arc_ui.py (press e). See it: ~/arc_pi/laptop/view_debug.py"
}

up_stream() {
  camera_free stream
  start stream "$ARC_VENV/bin/python" "$ARC_PI_DIR/pi/camera_pub.py" --camera "$CAMERA" \
    --width "$CAM_W" --height "$CAM_H" --fps 10
  say "On the laptop: perception_bridge.py --camera ros:/arc/camera/image/compressed (do not also run 'perception' here)"
}

case "${1:-status}" in
  motors|perception|stream) prechecks; "up_$1" ;;
  all) prechecks; up_motors; up_perception ;;
  down)
    if [ -n "${2:-}" ]; then stop "$2"; else for n in perception stream motors; do stop "$n"; done; fi ;;
  status)
    for n in motors perception stream; do
      if running "$n"; then ok "$n running (pid $(cat "$PIDS/$n.pid")), log: $(readlink "$LOGS/$n.latest.log")"
      else echo "  $n: stopped"; fi
    done ;;
  logs)
    n=${2:-perception}; [ -e "$LOGS/$n.latest.log" ] || die "no log for $n"
    tail -n 40 -f "$LOGS/$n.latest.log" ;;
  *) echo "usage: $0 motors|perception|stream|all|status|logs [name]|down [name]"; exit 2 ;;
esac
