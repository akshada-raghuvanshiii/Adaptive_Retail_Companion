#!/usr/bin/env bash
# Run ON THE PI:  ~/arc_pi_udp/pi/run_on_pi.sh <laptop IP> [camera index, default 0]
# Webcam -> YOLOv8n + ByteTrack + target lock (headless) -> UDP to perception_relay.py on the laptop.
# Ctrl+C (or SIGTERM) sends "wait" to the laptop before exiting.
LAPTOP=${1:?usage: run_on_pi.sh <laptop IP on the hotspot, e.g. 172.20.10.3> [camera index]}
CAM=${2:-0}
DST=$HOME/arc-perception
ping -c 1 -W 1 "$LAPTOP" >/dev/null 2>&1 && echo "[ OK ] laptop answers at $LAPTOP" \
  || echo "[WARN] no ping reply from $LAPTOP (same hotspot? laptop firewall?). Starting anyway."
cd "$DST" && exec "$DST/.venv/bin/python" ros/perception_bridge.py --headless --udp "$LAPTOP:5600" \
  --camera "$CAM" --width 640 --height 480 --mjpg --imgsz "${IMGSZ:-320}" --max-lost-s 4 --debug-image-hz 2
