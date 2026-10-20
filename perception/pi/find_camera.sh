#!/usr/bin/env bash
# Which /dev/videoN is the USB webcam? The Pi itself adds many video nodes (codec, ISP: video19+).
# Prints each device's name, then grabs one frame from the first USB camera it finds. Changes nothing.
PY=$HOME/arc-perception/.venv/bin/python
for d in /sys/class/video4linux/video*; do
  n=${d##*/}; echo "  /dev/$n  $(cat "$d/name" 2>/dev/null)"
done | sort -V
for d in /sys/class/video4linux/video*; do
  n=${d##*/}; nm=$(cat "$d/name" 2>/dev/null)
  case "$nm" in *pispbe*|*rpi-hevc*|*bcm2835*|*rp1-cfe*|*codec*|*isp*) continue ;; esac
  idx=${n#video}
  if "$PY" - "$idx" <<'PY' 2>/dev/null; then echo "CAMERA=$idx  ($nm)  <- use this index"; exit 0; fi
import sys, cv2
c = cv2.VideoCapture(int(sys.argv[1]))
ok, f = c.read()
c.release()
print(f"  index {sys.argv[1]}: frame {f.shape[1]}x{f.shape[0]}" if ok else f"  index {sys.argv[1]}: no frame")
sys.exit(0 if ok else 1)
PY
done
echo "No camera gave a frame. Is the webcam plugged in? (lsusb)"; exit 1
