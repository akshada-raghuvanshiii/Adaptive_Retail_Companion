#!/usr/bin/env python3
"""Measure perception speed ON THE RASPBERRY PI (no ROS needed). These are real Pi numbers,
for the report, unlike the laptop timings elsewhere in the project.

For each model x input size it runs the same call the bridge makes (YOLO track, persons only, CPU,
ByteTrack) on live webcam frames (or a video file) and records per-frame time, plus CPU temperature
and throttling before and after.

    ~/arc_pi_venv/bin/python ~/arc_pi/pi/pi_perf.py                         # yolov8n.pt at 320 and 416
    ~/arc_pi_venv/bin/python ~/arc_pi/pi/pi_perf.py --export-ncnn --imgsz 320 # also ncnn (needs pip ncnn)
    ~/arc_pi_venv/bin/python ~/arc_pi/pi/pi_perf.py --source clip.mp4 --frames 100

Output: ~/arc_pi_run/perf/<time>/frames.csv, summary.json, and a table on screen.
Close the perception bridge first (it holds the camera and the CPU).
"""
import argparse
import csv
import json
import os
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path


def cpu_temp_c():
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
    except (OSError, ValueError):
        return None


def throttled():
    try:
        return subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def board():
    try:
        return Path("/proc/device-tree/model").read_text().strip("\x00\n")
    except OSError:
        return platform.machine()


def pct(v, q):
    s = sorted(v)
    if not s:
        return float("nan")
    k = (len(s) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def summarise(ms):
    return {"n": len(ms), "median_ms": round(statistics.median(ms), 1), "p95_ms": round(pct(ms, 0.95), 1),
            "max_ms": round(max(ms), 1), "fps_median": round(1000.0 / statistics.median(ms), 2)}


def open_source(src, w, h):
    import cv2
    cap = cv2.VideoCapture(int(src) if str(src).isdigit() else src)
    if str(src).isdigit():
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def grab(cap, src):
    ok, f = cap.read()
    if not ok and not str(src).isdigit():          # video file: loop
        import cv2
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ok, f = cap.read()
    return ok, f


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    run = Path(os.environ.get("ARC_RUN", Path.home() / "arc_pi_run"))
    ap.add_argument("--source", default="0", help="camera index or video file")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--model", default=str(run / "arc-perception" / "yolov8n.pt"))
    ap.add_argument("--tracker", default=str(run / "arc-perception" / "configs" / "bytetrack_buf60.yaml"))
    ap.add_argument("--imgsz", type=int, nargs="+", default=[320, 416])
    ap.add_argument("--frames", type=int, default=60, help="timed frames per configuration")
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--capture-s", type=float, default=5.0, help="capture-only test length")
    ap.add_argument("--export-ncnn", action="store_true", help="also export and time an ncnn model per imgsz")
    ap.add_argument("--out", default=str(run / "perf" / time.strftime("%Y%m%d_%H%M%S")))
    a = ap.parse_args(argv)

    import cv2
    from ultralytics import YOLO
    try:
        import torch
        torch_info = {"torch": torch.__version__, "threads": torch.get_num_threads()}
    except ImportError:
        torch_info = {}
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    info = {"board": board(), "machine": platform.machine(), "python": platform.python_version(),
            "opencv": cv2.__version__, **torch_info, "source": a.source, "frame_size": [a.width, a.height],
            "temp_start_c": cpu_temp_c(), "throttled_start": throttled(), "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    print(f"board: {info['board']} | temp {info['temp_start_c']} C | {info['throttled_start']}")

    cap = open_source(a.source, a.width, a.height)
    ok, frame = grab(cap, a.source)
    if not ok:
        sys.exit(f"cannot read source {a.source}")
    info["frame_size_actual"] = [frame.shape[1], frame.shape[0]]
    t0, n = time.monotonic(), 0
    while time.monotonic() - t0 < a.capture_s:
        ok, _ = grab(cap, a.source)
        n += ok
    info["capture_only_fps"] = round(n / (time.monotonic() - t0), 2)
    print(f"capture only: {info['capture_only_fps']} fps at {info['frame_size_actual']}")

    configs = [(a.model, s) for s in a.imgsz]
    if a.export_ncnn:
        for s in a.imgsz:
            try:
                path = YOLO(a.model).export(format="ncnn", imgsz=s)
                configs.append((str(path), s))
                print(f"exported ncnn at imgsz {s}: {path}")
            except Exception as e:                     # missing ncnn/pnnx, no internet, ...
                print(f"ncnn export at {s} failed: {e}")

    rows, results = [], []
    for model_path, s in configs:
        model = YOLO(model_path, task="detect")
        times = []
        for i in range(a.warmup + a.frames):
            ok, f = grab(cap, a.source)
            if not ok:
                break
            tic = time.perf_counter()
            model.track(f, persist=True, tracker=a.tracker, classes=[0], imgsz=s, conf=0.25, iou=0.5,
                        device="cpu", verbose=False)
            ms = (time.perf_counter() - tic) * 1000.0
            if i >= a.warmup:
                times.append(ms)
                rows.append({"model": Path(model_path).name, "imgsz": s, "frame": i - a.warmup,
                             "track_ms": round(ms, 2), "temp_c": cpu_temp_c()})
        if not times:
            continue
        r = {"model": Path(model_path).name, "imgsz": s, **summarise(times), "temp_end_c": cpu_temp_c()}
        results.append(r)
        print(f"{r['model']:<24} imgsz {s:<4} median {r['median_ms']:7.1f} ms  p95 {r['p95_ms']:7.1f} ms  "
              f"-> {r['fps_median']:5.2f} fps   ({r['temp_end_c']} C)")
    cap.release()
    info["temp_end_c"], info["throttled_end"] = cpu_temp_c(), throttled()
    with open(out / "frames.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["model", "imgsz", "frame", "track_ms", "temp_c"])
        w.writeheader()
        w.writerows(rows)
    (out / "summary.json").write_text(json.dumps({"info": info, "results": results}, indent=2))
    print(f"saved {out}/summary.json and frames.csv")
    print("Note: YOLO + tracking time only. The whole bridge loop adds capture, lock logic and ROS "
          "publishing; its live rate is 'fps' in /arc/target_lock.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
