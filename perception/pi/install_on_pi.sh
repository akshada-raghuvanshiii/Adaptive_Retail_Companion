#!/usr/bin/env bash
# Run ON THE PI from the unpacked kit:  ~/arc_pi_udp/pi/install_on_pi.sh
# Copies the perception code into ~/arc-perception (your existing .venv is kept and used).
# Anything it replaces is first copied to ~/backups/arc-perception_<time>/. Nothing is deleted.
# No packages are installed: if the venv lacks something, the pip command is printed for you to run.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
DST=$HOME/arc-perception
B=$HOME/backups/arc-perception_$(date +%Y%m%d_%H%M%S)
mkdir -p "$DST" "$B"
for p in ros arc_assist configs tools/live_lock.py yolov8n.pt; do
  if [ -e "$DST/$p" ]; then mkdir -p "$B/$(dirname "$p")"; cp -a "$DST/$p" "$B/$p"; fi
done
cp -a "$HERE/arc-perception/." "$DST/"
echo "[ OK ] code copied to $DST (previous files backed up in $B)"
PY=$DST/.venv/bin/python
if [ ! -x "$PY" ]; then
  echo "[TODO] no venv at $DST/.venv. Create it once:"
  echo "       python3 -m venv $DST/.venv && $DST/.venv/bin/pip install ultralytics lap"
  exit 1
fi
miss=()
for m in ultralytics cv2 lap numpy; do "$PY" -c "import $m" 2>/dev/null || miss+=("$m"); done
if [ ${#miss[@]} -gt 0 ]; then
  pk=$(printf '%s\n' "${miss[@]}" | sed 's/^cv2$/opencv-python-headless/' | tr '\n' ' ')
  echo "[TODO] the venv is missing: ${miss[*]}  (ByteTrack needs lap; YOLO needs ultralytics)"
  echo "       $DST/.venv/bin/pip install $pk"
  exit 1
fi
echo "[ OK ] venv has ultralytics, opencv, lap, numpy"
"$PY" -c "import sys; sys.path[:0]=['$DST','$DST/tools','$DST/ros']; import netshim, live_lock, arc_assist.target_lock; print('[ OK ] perception code imports')"
echo "Next:  $HERE/find_camera.sh   then   $HERE/run_on_pi.sh <laptop IP>"
