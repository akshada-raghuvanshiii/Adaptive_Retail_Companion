#!/usr/bin/env python3
"""ARC perception -> behaviour tree bridge (ROS 2 node, runs with the perception venv).

Camera -> YOLOv8n + ByteTrack -> TargetLock -> AssistFSM -> the SAME inputs the behaviour tree
already reads from person_state_input:

    /person_moving    std_msgs/Bool   true while the enrolled shopper is locked, the mode is FOLLOW,
                                      AND the shopper is actually walking (box motion in the image);
                                      standing still -> false -> the BT waits
    /person_confused  std_msgs/Bool   true while the mode is LEAD (shopper asked to be led)
    /arc/target_lock  std_msgs/String JSON diagnostics: lock state, target ID, mode, shopper motion, FPS
    /arc/perception_markers  visualization_msgs/MarkerArray  RViz: status text + coloured ring on the robot
    /arc/perception/debug/compressed  sensor_msgs/CompressedImage  annotated frame, only with --debug-image-hz

Subscribes (so it can run headless on the Raspberry Pi and be driven from the laptop):
    /arc/ui/command   std_msgs/String  enrol | reset | stop | yes | lead_me | no   (tools: arc_pi/laptop/arc_ui.py)

Run it INSTEAD of person_state_input (both publish the same topics). Nothing in arc_behavior_tree,
arc_motion_gate, the planner or the controller is changed.

Safety choices
  * It never publishes /emergency_stop or /person_state, so perception can never raise OR clear an
    emergency; that stays with the operator (person_state_input option 4 / a physical button).
  * Anything other than LOCKED + FOLLOW/LEAD publishes moving=false, confused=false -> the BT
    falls through to WaitForPerson and the motion gate stops the robot.
  * On exit (q, Esc, Ctrl+C) it publishes false/false a few times before shutting down.
  * KNOWN GAP: if this process is killed or crashes, the BT keeps the last values it received
    (it has no staleness check). Closing that gap needs a timeout in person_assistant_bt, which is
    not changed here without your approval.

    source /opt/ros/jazzy/setup.bash
    ~/arc_perception/.venv/bin/python ros/perception_bridge.py            # laptop camera
    ~/arc_perception/.venv/bin/python ros/perception_bridge.py --camera 1 --max-lost-s 4
    ~/arc_perception/.venv/bin/python ros/perception_bridge.py --camera videos/my_walk.mp4 --loop   # replay a clip
    # Raspberry Pi, no screen: USB webcam at 640x480 MJPG, debug image for the laptop at 2 Hz
    python ros/perception_bridge.py --headless --width 640 --height 480 --mjpg --debug-image-hz 2
    # Raspberry Pi WITHOUT ROS (Raspberry Pi OS): same node, topics sent over UDP to ros/perception_relay.py
    python ros/perception_bridge.py --headless --udp 172.20.10.3:5600 --width 640 --height 480 --mjpg --imgsz 320 --debug-image-hz 2
    # Laptop, frames streamed from the Pi by arc_pi/pi/camera_pub.py
    python ros/perception_bridge.py --camera ros:/arc/camera/image/compressed

Replaying a recorded clip (--camera <file>): time comes from the clip (frame / fps), and frames are
skipped to keep pace with the wall clock, so the lock and motion timings behave as they would live
even when YOLO runs slower than the clip. --no-realtime processes every frame (slow motion) instead.

Window keys: click/e enrol, r reset, s/y/l/n shopper buttons (stop/yes/lead me/no), p pause, q quit.
Timings (fps, proc_ms in /arc/target_lock) describe the machine the node runs on and nothing else.
"""

import argparse
import json
import signal
import socket
import sys
import threading
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from arc_assist.appearance import torso_histogram  # noqa: E402
from arc_assist.assist_fsm import AssistFSM, Mode  # noqa: E402
from arc_assist.motion_state import MotionState  # noqa: E402
from arc_assist.target_lock import LockConfig, LockState, TargetLock  # noqa: E402
from live_lock import detections_from_arrays, draw, largest, person_at  # noqa: E402


def bt_flags(lock_state, mode, shopper_moving=True):
    """(person_moving, person_confused) for the behaviour tree. Fail-safe: anything uncertain -> wait.
    FOLLOW only while the shopper is walking; a shopper standing still -> the cart waits."""
    if lock_state != LockState.LOCKED:
        return False, False
    if mode == Mode.LEAD:
        return False, True
    if mode == Mode.FOLLOW:
        return bool(shopper_moving), False
    return False, False          # IDLE / WAIT / OFFER_HELP -> wait


# RViz status colours (r, g, b)
COLOURS = {"FOLLOW": (0.2, 0.8, 0.2), "LEAD": (0.2, 0.5, 1.0), "WAIT": (1.0, 0.65, 0.0), "LOST": (1.0, 0.25, 0.2)}


def status(lock_state, moving, confused, shopper_moving):
    """(behaviour the BT will choose, one-line label) for the RViz marker."""
    if lock_state != LockState.LOCKED:
        beh = "LOST" if lock_state in (LockState.LOST, LockState.NEEDS_ENROL) else "WAIT"
    else:
        beh = "LEAD" if confused else ("FOLLOW" if moving else "WAIT")
    shopper = "WALKING" if shopper_moving else "STILL"
    return beh, f"SHOPPER: {shopper if lock_state == LockState.LOCKED else '-'} | LOCK: {lock_state.value} | -> {beh}"


def build_markers(Marker, MarkerArray, lock_state, moving, confused, shopper_moving, frame="base_footprint"):
    beh, text = status(lock_state, moving, confused, shopper_moving)
    r, g, b = COLOURS[beh]
    ring = Marker()
    ring.header.frame_id = frame
    ring.ns, ring.id, ring.type, ring.action = "arc_perception", 0, Marker.CYLINDER, Marker.ADD
    ring.pose.position.z = 0.01
    ring.pose.orientation.w = 1.0
    ring.scale.x = ring.scale.y = 0.45
    ring.scale.z = 0.02
    ring.color.r, ring.color.g, ring.color.b, ring.color.a = r, g, b, 0.6
    label = Marker()
    label.header.frame_id = frame
    label.ns, label.id, label.type, label.action = "arc_perception", 1, Marker.TEXT_VIEW_FACING, Marker.ADD
    label.pose.position.z = 0.55
    label.pose.orientation.w = 1.0
    label.scale.z = 0.14
    label.color.r, label.color.g, label.color.b, label.color.a = r, g, b, 1.0
    label.text = text
    arr = MarkerArray()
    arr.markers = [ring, label]
    return arr


UI_COMMANDS = ("enrol", "reset", "stop", "yes", "lead_me", "no")


class RosFrames:
    """Frames from a sensor_msgs/CompressedImage topic (camera on another machine).
    read() waits up to `timeout` s for a frame newer than the last one returned."""

    def __init__(self, node, topic, timeout=1.0):
        from sensor_msgs.msg import CompressedImage
        from rclpy.qos import qos_profile_sensor_data
        import numpy as np
        self._np, self._timeout = np, timeout
        self._cv = threading.Condition()
        self._data, self._seq, self._seen = None, 0, 0
        self.node = node
        node.create_subscription(CompressedImage, topic, self._on_img, qos_profile_sensor_data)

    def _on_img(self, msg):
        with self._cv:
            self._data, self._seq = bytes(msg.data), self._seq + 1
            self._cv.notify_all()

    def read(self, spin):
        end = time.monotonic() + self._timeout
        while time.monotonic() < end:
            with self._cv:
                if self._seq != self._seen:
                    self._seen, data = self._seq, self._data
                    break
            spin(0.02)
        else:
            return False, None
        img = cv2.imdecode(self._np.frombuffer(data, self._np.uint8), cv2.IMREAD_COLOR)
        return img is not None, img

    def release(self):
        pass


def open_camera(src, args):
    """OpenCV capture for a device index or a file; size/MJPG requests only apply to devices."""
    cap = cv2.VideoCapture(src)
    if isinstance(src, int):
        if args.mjpg:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if args.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        if args.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # newest frame, not a queue of old ones
    return cap


def ros_init(rclpy):
    """rclpy.init without rclpy's own SIGINT/SIGTERM handlers, so Ctrl+C and SIGTERM reach our
    cleanup (publish wait / send stop) while the ROS context is still alive."""
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", default="0")
    ap.add_argument("--model", default=str(ROOT / "yolov8n.pt"))
    ap.add_argument("--tracker", default=str(ROOT / "configs" / "bytetrack_buf60.yaml"))
    ap.add_argument("--imgsz", type=int, default=416, help="YOLO input size (416 is faster than 640 on a busy laptop)")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--max-lost-s", type=float, default=LockConfig.max_lost_s)
    ap.add_argument("--no-exit-check", action="store_true")
    ap.add_argument("--no-motion", action="store_true",
                    help="FOLLOW whenever locked (old behaviour), without the walking/still check")
    ap.add_argument("--no-realtime", action="store_true",
                    help="video file: process every frame instead of skipping frames to keep real time")
    ap.add_argument("--loop", action="store_true", help="video file: start again at the end (lock is reset)")
    ap.add_argument("--headless", action="store_true", help="no window (enrol with --auto-enrol)")
    ap.add_argument("--auto-enrol", action="store_true", help="enrol the largest person automatically when IDLE")
    ap.add_argument("--width", type=int, default=0, help="camera device: requested frame width")
    ap.add_argument("--height", type=int, default=0, help="camera device: requested frame height")
    ap.add_argument("--mjpg", action="store_true", help="camera device: request MJPG (less USB and CPU load)")
    ap.add_argument("--debug-image-hz", type=float, default=0.0,
                    help="publish the annotated frame as JPEG at this rate (0 = off)")
    ap.add_argument("--debug-quality", type=int, default=60)
    ap.add_argument("--ros-timeout-s", type=float, default=1.0,
                    help="ros:<topic> camera: publish wait if no frame arrives for this long")
    ap.add_argument("--udp", default="", metavar="LAPTOP_IP:PORT",
                    help="no ROS on this machine (e.g. a Pi on Raspberry Pi OS): send the same topics over UDP to "
                         "ros/perception_relay.py on the laptop, which republishes them (see ros/netshim.py)")
    ap.add_argument("--udp-port", type=int, default=5601, help="with --udp: port for shopper commands from the laptop")
    args = ap.parse_args(argv)
    if args.udp:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        import netshim
        netshim.install(args.udp, args.udp_port)
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, _sigterm)     # pi_down.sh / systemd stop -> same clean exit as Ctrl+C

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from visualization_msgs.msg import Marker, MarkerArray
    from ultralytics import YOLO

    ros_init(rclpy)
    node = Node("arc_perception_bridge")
    pub_moving = node.create_publisher(Bool, "/person_moving", 10)
    pub_confused = node.create_publisher(Bool, "/person_confused", 10)
    pub_diag = node.create_publisher(String, "/arc/target_lock", 10)
    pub_markers = node.create_publisher(MarkerArray, "/arc/perception_markers", 10)
    pub_debug = None
    if args.debug_image_hz > 0:
        from sensor_msgs.msg import CompressedImage
        pub_debug = node.create_publisher(CompressedImage, "/arc/perception/debug/compressed", 1)
    ui = {"cmd": None}

    def on_ui(msg):
        c = str(msg.data).strip().lower()
        if c in UI_COMMANDS:
            ui["cmd"] = c
        else:
            node.get_logger().info(f"ignored /arc/ui/command '{c}' (use one of {', '.join(UI_COMMANDS)})")
    if hasattr(node, "create_subscription"):
        node.create_subscription(String, "/arc/ui/command", on_ui, 10)
    host = socket.gethostname()

    def publish(moving, confused, diag=None):
        pub_moving.publish(Bool(data=bool(moving)))
        pub_confused.publish(Bool(data=bool(confused)))
        if diag is not None:
            pub_diag.publish(String(data=json.dumps(diag)))

    from_ros = args.camera.startswith("ros:")
    src = int(args.camera) if args.camera.isdigit() else args.camera
    if from_ros:
        cap = RosFrames(node, args.camera[4:], args.ros_timeout_s)
        node.get_logger().info(f"waiting for frames on {args.camera[4:]} ...")
        ok, frame = False, None
        for _ in range(30):                          # up to ~30 x timeout for the first frame
            ok, frame = cap.read(lambda s: rclpy.spin_once(node, timeout_sec=s))
            if ok:
                break
    else:
        cap = open_camera(src, args)
        ok, frame = cap.read()
    if not ok:
        publish(False, False)
        node.get_logger().error(f"cannot read camera {args.camera}")
        rclpy.shutdown()
        return 2
    H, W = frame.shape[:2]
    is_file = not isinstance(src, int) and not from_ros
    vfps = (cap.get(cv2.CAP_PROP_FPS) or 0.0) if is_file else 0.0
    if is_file and vfps <= 0:
        vfps = 30.0
    pos, t_base = 1, 0.0          # frames consumed so far (the probe read above), time offset for --loop
    model = YOLO(args.model)
    cfg = LockConfig(image_size=None if args.no_exit_check else (W, H), max_lost_s=args.max_lost_s)
    lock, fsm, motion = TargetLock(cfg), AssistFSM(), MotionState()
    click = {"xy": None}
    win = "ARC perception bridge"
    if not args.headless:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win, lambda e, x, y, f, p: click.update(xy=(x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)

    t0, fps, pending, last_flags = time.monotonic(), 0.0, None, None
    last_debug = 0.0
    node.get_logger().info(f"camera {W}x{H}; publishing /person_moving, /person_confused, /arc/target_lock. "
                           "Do not run person_state_input at the same time.")
    try:
        while rclpy.ok():
            tic = time.monotonic()
            if is_file and not args.no_realtime:      # skip frames to stay in step with the wall clock
                target = int((tic - t0 - t_base) * vfps)
                while pos < target and cap.grab():
                    pos += 1
            if from_ros:
                ok, frame = cap.read(lambda s: rclpy.spin_once(node, timeout_sec=s))
                if not ok:                            # stream stopped: hold the BT in WAIT and keep listening
                    if lock.state != LockState.IDLE:
                        lock.reset()               # shopper must be enrolled again once frames return
                    motion.reset()
                    publish(False, False, {"lock": "NO_CAMERA", "moving": False, "confused": False,
                                           "host": host, "t": round(time.monotonic() - t0, 2)})
                    if last_flags != (False, False):
                        node.get_logger().info("no camera frames: BT flags moving=False confused=False")
                        last_flags = (False, False)
                    continue
            else:
                ok, frame = cap.read()
            if not ok:
                if is_file and args.loop:
                    t_base += pos / vfps
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    pos = 0
                    lock.reset()
                    fsm, pending = AssistFSM(), None
                    motion.reset()
                    node.get_logger().info("clip restarted: lock reset, click to enrol again")
                    continue
                break
            pos += 1
            t = (t_base + pos / vfps) if is_file else tic - t0
            r = model.track(frame, persist=True, tracker=args.tracker, classes=[0], imgsz=args.imgsz,
                            conf=args.conf, iou=args.iou, device="cpu", verbose=False)[0]
            b = r.boxes
            ids = b.id.int().tolist() if b is not None and b.id is not None else None
            dets = detections_from_arrays(b.xyxy.tolist() if b is not None else [],
                                          b.conf.tolist() if b is not None else [], ids,
                                          lambda box, f=frame: torso_histogram(f, box))
            who = None
            cmd, ui["cmd"] = ui["cmd"], None
            if cmd == "enrol":
                who = largest(dets)
            elif cmd == "reset":
                lock.reset()
                fsm = AssistFSM()
                motion.reset()
            elif cmd in ("stop", "yes", "lead_me", "no"):
                pending = cmd
            if who is None and click["xy"] is not None:
                who = person_at(dets, *click["xy"])
                click["xy"] = None
            elif who is None and args.auto_enrol and lock.state == LockState.IDLE:
                who = largest(dets)
            if who is not None:
                lock.enrol(who, t)
                fsm = AssistFSM()
                motion.reset()
                node.get_logger().info(f"enrolled ID {who.track_id}")

            out = lock.update(dets, t)
            f_out = fsm.step(t, out.state, user=pending)
            if pending is not None:
                node.get_logger().info(f"shopper button '{pending}' -> mode {f_out.mode.value}")
            pending = None
            if out.state == LockState.LOCKED and out.box is not None:
                shopper_moving = motion.update(t, out.box)
            else:
                motion.reset()
                shopper_moving = False
            moving, confused = bt_flags(out.state, f_out.mode, True if args.no_motion else shopper_moving)
            pub_markers.publish(build_markers(Marker, MarkerArray, out.state, moving, confused, shopper_moving))
            dt = max(time.monotonic() - tic, 1e-6)
            fps = 0.9 * fps + 0.1 / dt if fps else 1.0 / dt
            publish(moving, confused, {"lock": out.state.value, "target_id": out.track_id, "mode": f_out.mode.value,
                                       "reason": out.reason, "shopper_walking": shopper_moving,
                                       "motion_speed": round(motion.speed, 3), "moving": moving,
                                       "confused": confused, "fps": round(fps, 1), "proc_ms": round(dt * 1000, 1),
                                       "host": host, "t": round(t, 2)})
            if (moving, confused) != last_flags:
                node.get_logger().info(f"BT flags moving={moving} confused={confused} "
                                       f"({out.state.value}, {f_out.mode.value}: {out.reason})")
                last_flags = (moving, confused)
            rclpy.spin_once(node, timeout_sec=0.0)

            want_debug = pub_debug is not None and time.monotonic() - last_debug >= 1.0 / args.debug_image_hz
            if not args.headless or want_debug:
                shown = frame.copy()
                draw(shown, dets, lock, out, f_out.mode.value,
                     f"shopper {'WALKING' if shopper_moving else 'STILL'} ({motion.speed:.2f} h/s) -> "
                     f"BT {status(out.state, moving, confused, shopper_moving)[0]}", fps, False)
            if want_debug:
                last_debug = time.monotonic()
                ok_jpg, jpg = cv2.imencode(".jpg", shown, [int(cv2.IMWRITE_JPEG_QUALITY), args.debug_quality])
                if ok_jpg:
                    from sensor_msgs.msg import CompressedImage
                    m = CompressedImage()
                    m.format = "jpeg"
                    m.data = jpg.tobytes()
                    pub_debug.publish(m)
            if not args.headless:
                cv2.imshow(win, shown)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("r"):
                    lock.reset()
                    fsm = AssistFSM()
                    motion.reset()
                elif key == ord("e"):
                    w = largest(dets)
                    if w is not None:
                        lock.enrol(w, t)
                        fsm = AssistFSM()
                        motion.reset()
                elif key in (ord("s"), ord("y"), ord("l"), ord("n")):
                    pending = {"s": "stop", "y": "yes", "l": "lead_me", "n": "no"}[chr(key)]
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):                      # leave the BT in WAIT
            publish(False, False, {"lock": "SHUTDOWN", "moving": False, "confused": False})
            pub_markers.publish(build_markers(Marker, MarkerArray, LockState.IDLE, False, False, False))
            time.sleep(0.05)
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
