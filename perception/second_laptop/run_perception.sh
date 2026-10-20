#!/usr/bin/env bash
# Run on the perception laptop (Linux):  ./run_perception.sh <ARC laptop IP> [camera index, default 0] [extra bridge options]
# No webcam? Replay a clip of one person walking instead:  ./run_perception.sh <ARC laptop IP> ~/clip.mp4 --loop
# Webcam -> YOLOv8n + ByteTrack + target lock -> UDP to perception_relay.py on the ARC laptop.
# Same bridge and settings as the Pi version. Ctrl+C sends "wait" before exiting.
ARC=${1:?usage: run_perception.sh <ARC laptop IP, printed by arc_demo.sh> [camera index]}
CAM=${2:-0}
HERE=$(cd "$(dirname "$0")" && pwd)
ping -c 1 -W 1 "$ARC" >/dev/null 2>&1 && echo "[ OK ] ARC laptop answers at $ARC" || echo "[WARN] no ping reply from $ARC (same network?). Starting anyway."
cd "$HERE/arc-perception" && exec "$HERE/.venv/bin/python" ros/perception_bridge.py --headless --udp "$ARC:5600" \
  --camera "$CAM" --width 640 --height 480 --mjpg --imgsz "${IMGSZ:-416}" --max-lost-s 4 --debug-image-hz 2 "${@:3}"
