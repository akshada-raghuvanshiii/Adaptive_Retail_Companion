#!/usr/bin/env bash
# Run ONCE on the perception laptop (Linux), with internet:  ./setup.sh
# Makes a venv next to this script with CPU-only PyTorch, Ultralytics (YOLOv8) and lap (ByteTrack).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
python3 -m venv "$HERE/.venv"
"$HERE/.venv/bin/pip" install --upgrade pip
"$HERE/.venv/bin/pip" install torch torchvision --index-url https://download.pytorch.org/whl/cpu \
  || "$HERE/.venv/bin/pip" install torch torchvision   # fallback: the normal PyPI build (bigger, also works)
"$HERE/.venv/bin/pip" install ultralytics lap
"$HERE/.venv/bin/python" -c "import ultralytics, cv2, lap; print('[ OK ] perception venv ready')"
"$HERE/.venv/bin/python" "$HERE/find_camera.py"
