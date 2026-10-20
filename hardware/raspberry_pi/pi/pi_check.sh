#!/usr/bin/env bash
# Health check ON THE PI. Prints PASS/WARN/FAIL per item; nothing is installed or changed.
#   ~/arc_pi/pi/pi_check.sh              everything
#   ~/arc_pi/pi/pi_check.sh <part>...    env net laptop serial camera venv thermal
ARC_PI_DIR=$(cd "$(dirname "$0")/.." && pwd)
source "$ARC_PI_DIR/pi/pi_env.sh" >/dev/null 2>&1
source "$ARC_PI_DIR/lib.sh"
FAILS=0
f() { bad "$*"; FAILS=$((FAILS + 1)); }

part_env() {
  say "-- env"
  [ -f /opt/ros/jazzy/setup.bash ] && ok "ROS 2 Jazzy" || f "ROS 2 Jazzy missing (pi_setup.sh ros)"
  python3 -c "import rclpy" 2>/dev/null && ok "rclpy imports" || f "rclpy does not import"
  [ -f "$HOME/.arc_hil/fastdds_hil.xml" ] && ok "DDS profile: this Pi $(profile_own_ip), laptop $(profile_peer_ip | grep . || echo 'not in profile')" \
    || f "no ~/.arc_hil/fastdds_hil.xml"
  [ -d "$ARC_RUN/arc-perception/ros" ] && [ -f "$ARC_RUN/arc_hw/bridge_node.py" ] && ok "code deployed in $ARC_RUN" \
    || f "code not deployed: on the LAPTOP run ~/arc_pi/laptop/deploy.sh"
}

part_net() {
  say "-- net"
  local me lap; me=$(own_ip pi); lap=$(peer_ip pi)
  has_ip "$me" && ok "Pi has $me ($LINK)" || f "Pi has no ${LINK} address (hotspot: is the Pi on it? pi_setup.sh network)"
  if [ -n "$lap" ]; then
    ping -c 2 -W 1 "$lap" >/dev/null 2>&1 && ok "laptop answers at $lap" \
      || f "no answer from the laptop at $lap (same hotspot? phone 'client isolation' off?)"
  else
    f "cannot find the $(peer_name pi) (on the laptop: ~/arc_pi/laptop/deploy.sh; Pi: pi_setup.sh mdns)"
  fi
  lo_multicast_ok && ok "lo has MULTICAST" || f "lo has no MULTICAST: sudo ip link set lo multicast on  (permanent: pi_setup.sh lo)"
}

part_laptop() {
  say "-- laptop ROS (needs the laptop stack up in HIL mode)"
  command -v ros2 >/dev/null || { f "ros2 not on PATH"; return; }
  ros2 daemon stop >/dev/null 2>&1
  local topics; topics=$(timeout 15 ros2 topic list 2>/dev/null)
  for t in /cmd_vel /arc/behavior_mode /clock; do
    echo "$topics" | grep -qx "$t" && ok "sees $t from the laptop" || f "does not see $t"
  done
  if echo "$topics" | grep -qx /arc/behavior_mode; then
    local m; m=$(timeout 6 ros2 topic echo --once /arc/behavior_mode --field data 2>/dev/null | head -1)
    [ -n "$m" ] && ok "BT mode received: $m" || warn "/arc/behavior_mode listed but no message in 6 s"
  fi
}

part_serial() {
  say "-- serial (Uno on $SERIAL_PORT)"
  [ -e "$SERIAL_PORT" ] || { f "$SERIAL_PORT not present (USB cable? ls /dev/ttyACM* /dev/ttyUSB*)"; return; }
  [ -w "$SERIAL_PORT" ] || { f "no write access to $SERIAL_PORT (pi_setup.sh groups, then log in again)"; return; }
  if pgrep -f bridge_node.py >/dev/null; then warn "motor bridge running: it owns the port, skipping the handshake"; return; fi
  python3 - "$SERIAL_PORT" <<'PY'
import sys, time
try:
    import serial
except ImportError:
    print("\033[1;31m[ FAIL ]\033[0m python3-serial missing (pi_setup.sh apt)"); sys.exit(1)
s = serial.Serial(sys.argv[1], 115200, timeout=0.2)
time.sleep(2.0)                       # Uno resets when the port opens
end, lines = time.time() + 2.0, []
while time.time() < end:
    l = s.readline().decode(errors="ignore").strip()
    if l:
        lines.append(l)
s.write(b"S\n"); s.close()
reports = [l for l in lines if l.startswith("E ")]
if reports:
    print(f"\033[1;32m[  OK  ]\033[0m Uno answers: {len(reports)} reports in 2 s, last: {reports[-1]}")
elif lines:
    print(f"\033[1;33m[ WARN ]\033[0m Uno sends text but no 'E ...' reports (other sketch?): {lines[-1][:60]}")
    sys.exit(1)
else:
    print("\033[1;31m[ FAIL ]\033[0m nothing from the Uno in 2 s (sketch flashed? baud 115200?)"); sys.exit(1)
PY
  [ $? -eq 0 ] || FAILS=$((FAILS + 1))
}

part_camera() {
  say "-- camera ($CAMERA, ${CAM_W}x${CAM_H} MJPG)"
  if pgrep -f 'perception_bridge.py|camera_pub.py' >/dev/null; then warn "a camera node is running, skipping the grab test"; return; fi
  local py=$ARC_VENV/bin/python; [ -x "$py" ] || py=python3
  "$py" - "$CAMERA" "$CAM_W" "$CAM_H" <<'PY'
import sys, time
try:
    import cv2
except ImportError:
    print("\033[1;31m[ FAIL ]\033[0m OpenCV not importable (pi_setup.sh venv)"); sys.exit(1)
cam, w, h = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
cap = cv2.VideoCapture(int(cam) if cam.isdigit() else cam)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
ok, f = cap.read()
if not ok:
    print(f"\033[1;31m[ FAIL ]\033[0m cannot read camera {cam}"); sys.exit(1)
t0, n = time.time(), 0
while time.time() - t0 < 3.0:
    ok, f = cap.read(); n += ok
fps = n / (time.time() - t0)
cap.release()
print(f"\033[1;32m[  OK  ]\033[0m camera {cam}: {f.shape[1]}x{f.shape[0]}, {fps:.1f} fps capture only (this Pi, no YOLO)")
PY
  [ $? -eq 0 ] || FAILS=$((FAILS + 1))
}

part_venv() {
  say "-- venv ($ARC_VENV)"
  [ -x "$ARC_VENV/bin/python" ] || { f "no venv (pi_setup.sh venv)"; return; }
  "$ARC_VENV/bin/python" -c "import ultralytics, lap, cv2, rclpy; print('  ultralytics', ultralytics.__version__, '| opencv', cv2.__version__)" 2>/dev/null \
    && ok "ultralytics, lap, cv2 and rclpy import in the venv" || f "imports fail in the venv (pi_setup.sh venv; source pi_env.sh first)"
  [ -f "$ARC_RUN/arc-perception/yolov8n.pt" ] && ok "weights present" || warn "no yolov8n.pt in $ARC_RUN/arc-perception (deploy.sh copies it)"
}

part_thermal() {
  say "-- thermal / power"
  local t; t=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
  [ -n "$t" ] && echo "  CPU temperature: $((t / 1000)) C"
  if command -v vcgencmd >/dev/null; then
    local th; th=$(vcgencmd get_throttled 2>/dev/null)
    echo "  $th  (0x0 = never throttled or under-volted since boot)"
    [ "$th" = "throttled=0x0" ] || warn "throttling/under-voltage seen: use the official PSU and a heatsink/fan"
  else
    echo "  vcgencmd not available (Ubuntu); temperature above is the guide. Under 80 C is fine."
  fi
}

PARTS=(env net laptop serial camera venv thermal)
[ $# -gt 0 ] && PARTS=("$@")
for p in "${PARTS[@]}"; do
  declare -F "part_$p" >/dev/null || die "unknown part '$p'"
  "part_$p"
done
echo
[ $FAILS -eq 0 ] && ok "pi_check: no failures" || bad "pi_check: $FAILS failure(s)"
exit $((FAILS > 0))
