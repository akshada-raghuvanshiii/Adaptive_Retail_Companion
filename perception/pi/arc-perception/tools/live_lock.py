"""Live target-lock test on a webcam: YOLOv8n + ByteTrack -> appearance features -> TargetLock -> AssistFSM.

    python tools/live_lock.py                    # laptop camera 0
    python tools/live_lock.py --camera 1 --imgsz 480 --record lock_test.mp4

Controls (in the video window)
  left-click a person   enrol that person (start following)
  e                     enrol the largest person in view
  r                     reset the lock (back to IDLE)
  s / y / l / n         shopper buttons (simulated): STOP, YES (resume / accept), LEAD ME, NO
  p                     pause / resume
  q or Esc              quit

What you see
  grey boxes   every tracked person, with tracker ID and appearance similarity to the enrolled
               shopper (sim; 1.00 = identical colours)
  green box    the locked target          amber caption   LOST, looking to re-acquire
  red caption  NEEDS_ENROL: lost too long / left the view, the cart would wait for re-enrolment
  top bar      lock state | assist mode | reason | FPS

Everything is logged per frame to outputs/live_<timestamp>/frames.csv, and state changes to events.csv.

This uses the SAME TargetLock and AssistFSM code as the offline evaluation (arc_assist/), with
appearance (torso colour histogram) ON, so re-acquisition after an occlusion is possible.
Laptop-camera note: if you stand close, your box touches the image border, and when the lock is
lost there it treats that as "left the camera view" (no automatic re-acquisition, by design).
Stand back 2-3 m so your whole body is in view, or use --no-exit-check to test without that rule.
Laptop CPU numbers are not Raspberry Pi numbers.
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arc_assist.appearance import torso_histogram  # noqa: E402
from arc_assist.assist_fsm import AssistFSM  # noqa: E402
from arc_assist.target_lock import Detection, LockConfig, LockState, TargetLock, similarity  # noqa: E402

GREEN, AMBER, RED, GREY, WHITE, BLACK = (60, 200, 60), (0, 170, 255), (60, 60, 230), (180, 180, 180), (255, 255, 255), (0, 0, 0)
STATE_COLOUR = {LockState.IDLE: GREY, LockState.LOCKED: GREEN, LockState.LOST: AMBER, LockState.NEEDS_ENROL: RED}


# ----------------------------------------------------------------- pure helpers (unit-tested)

def detections_from_arrays(xyxy, confs, ids, feature_fn=None):
    """Build Detection objects from tracker arrays; boxes without a track ID are skipped."""
    out = []
    if ids is None:
        return out
    for box, c, tid in zip(xyxy, confs, ids):
        b = tuple(float(v) for v in box)
        feat = feature_fn(b) if feature_fn else None
        out.append(Detection(int(tid), b, float(c), feat))
    return out


def person_at(detections, x, y):
    """The detection whose box contains (x, y); the smallest box wins if several do."""
    hits = [d for d in detections if d.box[0] <= x <= d.box[2] and d.box[1] <= y <= d.box[3]]
    if not hits:
        return None
    return min(hits, key=lambda d: (d.box[2] - d.box[0]) * (d.box[3] - d.box[1]))


def largest(detections):
    if not detections:
        return None
    return max(detections, key=lambda d: (d.box[2] - d.box[0]) * (d.box[3] - d.box[1]))


# ----------------------------------------------------------------- drawing

def put(img, text, org, colour=WHITE, scale=0.5, thick=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, BLACK, thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thick, cv2.LINE_AA)


def draw(frame, dets, lock, out, mode, mode_reason, fps, paused):
    for d in dets:
        x1, y1, x2, y2 = (int(v) for v in d.box)
        is_target = out.track_id is not None and d.track_id == out.track_id
        sim = similarity(d.feature, lock.signature, lock.config.similarity_kind) if lock.signature else None
        label = f"ID {d.track_id}" + (f"  sim {sim:.2f}" if sim is not None else "")
        if d.track_id in lock.blacklist:
            label += "  [blacklisted]"
        colour = GREEN if is_target else GREY
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 3 if is_target else 1)
        put(frame, ("TARGET  " if is_target else "") + label, (x1 + 3, max(14, y1 - 6)), colour, 0.5, 1)
    h, w = frame.shape[:2]
    bar = frame.copy()
    cv2.rectangle(bar, (0, 0), (w, 58), BLACK, -1)
    cv2.addWeighted(bar, 0.6, frame, 0.4, 0, frame)
    col = STATE_COLOUR[out.state]
    put(frame, f"LOCK: {out.state.value}", (8, 22), col, 0.65, 2)
    put(frame, f"MODE: {mode}", (230, 22), WHITE, 0.65, 2)
    put(frame, f"{fps:4.1f} FPS", (w - 95, 22), WHITE, 0.55, 1)
    put(frame, f"{out.reason}  |  {mode_reason}"[:110], (8, 46), col, 0.5, 1)
    hint = "click to enrol | e largest  r reset | s stop  y yes  l lead  n no | p pause  q quit"
    if paused:
        hint = "PAUSED (p to resume)"
    put(frame, hint, (8, h - 10), WHITE, 0.45, 1)


# ----------------------------------------------------------------- main loop

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", default="0", help="camera index (0) or a video file path")
    ap.add_argument("--model", default=str(ROOT / "yolov8n.pt"))
    ap.add_argument("--tracker", default=str(ROOT / "configs" / "bytetrack_buf60.yaml"))
    ap.add_argument("--imgsz", type=int, default=640, help="YOLO input size; 480 or 320 is faster")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.5, help="NMS IoU (0.5 was best in experiment E3)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--mirror", action="store_true", help="flip horizontally (selfie view)")
    ap.add_argument("--no-appearance", action="store_true", help="geometry-only lock (original-ID recovery only)")
    ap.add_argument("--no-exit-check", action="store_true", help="disable the 'left the camera view' rule")
    ap.add_argument("--max-lost-s", type=float, default=LockConfig.max_lost_s)
    ap.add_argument("--record", help="also save the annotated video to this .mp4")
    args = ap.parse_args(argv)

    from ultralytics import YOLO  # venv only; imported here so the helpers stay testable without it

    src = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(src)
    if isinstance(src, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    ok, frame = cap.read()
    if not ok:
        sys.exit(f"Could not read from camera {args.camera}. Try --camera 1, close other apps using the "
                 "camera, and check `ls /dev/video*`.")
    H, W = frame.shape[:2]

    out_dir = ROOT / "outputs" / f"live_{datetime.now():%Y%m%d_%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)
    fcsv = open(out_dir / "frames.csv", "w", newline="")
    ecsv = open(out_dir / "events.csv", "w", newline="")
    fw, ew = csv.writer(fcsv), csv.writer(ecsv)
    fw.writerow(["t_s", "frame", "fps", "people", "lock_state", "target_id", "mode", "reason", "target_sim"])
    ew.writerow(["t_s", "frame", "event", "detail"])
    writer = None
    if args.record:
        writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"), 15, (W, H))

    model = YOLO(args.model)
    cfg = LockConfig(require_appearance=not args.no_appearance,
                     image_size=None if args.no_exit_check else (W, H),
                     max_lost_s=args.max_lost_s)
    lock, fsm = TargetLock(cfg), AssistFSM()
    state = {"click": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (x, y)

    win = "ARC live target lock"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    t0 = time.monotonic()
    frame_no, fps, paused, pending_user = 0, 0.0, False, None
    last_state, last_mode, last_target = None, None, None
    dets, out, mode, mode_reason = [], None, "IDLE", ""
    print(f"Camera {args.camera}: {W}x{H}. Logging to {out_dir}")
    print("Click yourself in the window to enrol. q to quit.")

    def event(t, kind, detail):
        ew.writerow([f"{t:.2f}", frame_no, kind, detail])
        print(f"[{t:7.2f}s] {kind:12s} {detail}")

    while True:
        if not paused:
            tic = time.monotonic()
            ok, frame = cap.read()
            if not ok:
                print("Camera / video ended.")
                break
            if args.mirror:
                frame = cv2.flip(frame, 1)
            frame_no += 1
            t = time.monotonic() - t0
            r = model.track(frame, persist=True, tracker=args.tracker, classes=[0], imgsz=args.imgsz,
                            conf=args.conf, iou=args.iou, device="cpu", verbose=False)[0]
            b = r.boxes
            ids = b.id.int().tolist() if b is not None and b.id is not None else None
            feat_fn = None if args.no_appearance else (lambda box, f=frame: torso_histogram(f, box))
            dets = detections_from_arrays(b.xyxy.tolist() if b is not None else [],
                                          b.conf.tolist() if b is not None else [], ids, feat_fn)

            # enrolment by click or key
            if state["click"] is not None:
                who = person_at(dets, *state["click"])
                state["click"] = None
                if who is None:
                    event(t, "click", "no tracked person under the cursor")
                else:
                    lock.enrol(who, t)
                    fsm = AssistFSM()
                    event(t, "ENROL", f"ID {who.track_id} (click)")

            out = lock.update(dets, t)
            f_out = fsm.step(t, out.state, user=pending_user)
            if pending_user:
                event(t, "BUTTON", f"{pending_user} -> {f_out.mode.value}: {f_out.reason}")
            pending_user = None
            mode, mode_reason = f_out.mode.value, f_out.reason
            dt = time.monotonic() - tic
            fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

            tsim = None
            if out.track_id is not None and lock.signature:
                tgt = next((d for d in dets if d.track_id == out.track_id), None)
                tsim = similarity(tgt.feature, lock.signature, cfg.similarity_kind) if tgt else None
            fw.writerow([f"{t:.3f}", frame_no, f"{fps:.1f}", len(dets), out.state.value, out.track_id or "",
                         mode, out.reason, f"{tsim:.3f}" if tsim is not None else ""])
            if out.state != last_state:
                event(t, out.state.value, out.reason)
                last_state = out.state
            if out.track_id is not None and last_target is not None and out.track_id != last_target:
                event(t, "ID_CHANGE", f"target now ID {out.track_id} (was {last_target}): {out.reason}")
            if out.track_id is not None:
                last_target = out.track_id
            if mode != last_mode:
                event(t, "MODE", f"{mode}: {mode_reason}")
                last_mode = mode

        shown = frame.copy()
        if out is not None:
            draw(shown, dets, lock, out, mode, mode_reason, fps, paused)
        cv2.imshow(win, shown)
        if writer is not None and not paused:
            writer.write(shown)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("p"):
            paused = not paused
        elif key == ord("r"):
            lock.reset()
            fsm = AssistFSM()
            last_target = None
            event(time.monotonic() - t0, "RESET", "lock reset to IDLE")
        elif key == ord("e"):
            who = largest(dets)
            if who is not None:
                tt = time.monotonic() - t0
                lock.enrol(who, tt)
                fsm = AssistFSM()
                event(tt, "ENROL", f"ID {who.track_id} (largest)")
        elif key in (ord("s"), ord("y"), ord("l"), ord("n")):
            pending_user = {"s": "stop", "y": "yes", "l": "lead_me", "n": "no"}[chr(key)]

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    fcsv.close()
    ecsv.close()
    print(f"\nFrames: {frame_no}. Logs: {out_dir}/frames.csv and events.csv")
    print(f"Summary: python tools/live_lock_summary.py {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
