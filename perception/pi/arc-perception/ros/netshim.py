"""Stand-in for the few rclpy / message APIs that perception_bridge.py uses, sending over plain UDP.

For a Raspberry Pi that has no ROS 2 (e.g. Raspberry Pi OS / Debian trixie). The bridge code runs
unchanged; only its "ROS" layer is swapped:

    python ros/perception_bridge.py --headless --udp <laptop-ip>:5600 ...

  publish /person_moving, /person_confused, /arc/target_lock  -> one JSON datagram each to the laptop
  publish /arc/perception/debug/compressed                    -> one datagram: b"ARCJ" + JPEG (kept < 60 kB)
  /arc/perception_markers                                     -> not sent (the laptop relay draws them)
  subscription /arc/ui/command                                <- JSON {"arc":1,"cmd":"enrol"} from the laptop

On the laptop, ros/perception_relay.py (a normal ROS 2 node) turns the datagrams back into the same ROS
topics and holds the BT in WAIT if they stop arriving. Datagram format (JSON, UTF-8):
    {"arc": 1, "topic": "/person_moving", "type": "Bool", "data": true, "seq": 17, "host": "manav"}

install(target, listen_port) puts fake rclpy / std_msgs / sensor_msgs / visualization_msgs modules into
sys.modules. Call it only when real ROS is NOT wanted in this process.
"""
import json
import select
import socket
import sys
import time
import types

MAGIC_JPEG = b"ARCJ"
MAX_DGRAM = 60000
SENT_TOPICS = ("/person_moving", "/person_confused", "/arc/target_lock")


class _Auto:
    """Object whose attributes spring into existence (msg.pose.position.z = 0.1 works)."""

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        v = _Auto()
        object.__setattr__(self, name, v)
        return v


class Bool(_Auto):
    def __init__(self, data=False):
        super().__init__(data=data)


class String(_Auto):
    def __init__(self, data=""):
        super().__init__(data=data)


class CompressedImage(_Auto):
    def __init__(self):
        super().__init__(format="jpeg", data=b"")


class Marker(_Auto):
    CYLINDER, TEXT_VIEW_FACING, ADD = 3, 9, 0


class MarkerArray(_Auto):
    def __init__(self):
        super().__init__(markers=[])


class Link:
    """One UDP socket: sends to the laptop, receives commands on listen_port."""

    def __init__(self, target, listen_port):
        host, _, port = target.rpartition(":")
        if not host or not port.isdigit():
            raise SystemExit(f"--udp needs HOST:PORT, got '{target}'")
        self.dest = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if host.endswith(".255") or host == "255.255.255.255":
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", listen_port))
        self.sock.setblocking(False)
        self.seq = 0
        self.host = socket.gethostname()
        self.send_errors = 0

    def send(self, payload: bytes):
        try:
            self.sock.sendto(payload, self.dest)
        except OSError:
            self.send_errors += 1      # laptop not reachable yet (Wi-Fi down etc.): keep running

    def send_json(self, topic, typ, data):
        self.seq += 1
        self.send(json.dumps({"arc": 1, "topic": topic, "type": typ, "data": data, "seq": self.seq,
                              "host": self.host}).encode())

    def poll(self, timeout):
        """Commands received within `timeout` s -> list of strings."""
        out = []
        r, _, _ = select.select([self.sock], [], [], max(0.0, timeout))
        while r:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                m = json.loads(data.decode())
                if m.get("arc") == 1 and isinstance(m.get("cmd"), str):
                    out.append(m["cmd"])
            except (ValueError, UnicodeDecodeError):
                pass
            r, _, _ = select.select([self.sock], [], [], 0)
        return out


def _shrink_jpeg(data):
    """Keep a debug JPEG under one datagram: halve the size / lower quality until it fits."""
    if len(data) <= MAX_DGRAM:
        return data
    try:
        import cv2
        import numpy as np
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        for scale, q in ((0.5, 60), (0.5, 40), (0.35, 40)):
            small = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode(".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if ok and len(jpg) <= MAX_DGRAM:
                return jpg.tobytes()
    except Exception:
        pass
    return None


class _Pub:
    def __init__(self, link, msg_type, topic):
        self.link, self.type, self.topic = link, msg_type, topic

    def publish(self, msg):
        if self.topic in SENT_TOPICS:
            self.link.send_json(self.topic, self.type.__name__, msg.data)
        elif self.type is CompressedImage:
            jpg = _shrink_jpeg(bytes(msg.data))
            if jpg:
                self.link.send(MAGIC_JPEG + jpg)
        # MarkerArray and anything else: not sent


class _Logger:
    def __init__(self, name):
        self.name = name

    def _p(self, level, text):
        print(f"[{level}] [{time.time():.3f}] [{self.name}]: {text}", flush=True)

    def info(self, t):
        self._p("INFO", t)

    def warn(self, t):
        self._p("WARN", t)

    warning = warn

    def error(self, t):
        self._p("ERROR", t)


def install(target, listen_port=5601):
    """Make `import rclpy` etc. resolve to the UDP stand-ins. Returns the Link (for tests)."""
    link = Link(target, listen_port)
    state = {"ok": True, "subs": {}}

    class Node:
        def __init__(self, name):
            self._log = _Logger(name)

        def create_publisher(self, msg_type, topic, qos):
            return _Pub(link, msg_type, topic)

        def create_subscription(self, msg_type, topic, cb, qos):
            state["subs"][topic] = cb

        def get_logger(self):
            return self._log

        def destroy_node(self):
            pass

    def spin_once(node, timeout_sec=0.0):
        for c in link.poll(timeout_sec):
            cb = state["subs"].get("/arc/ui/command")
            if cb:
                cb(String(data=c))

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: state.update(ok=True)
    rclpy.ok = lambda: state["ok"]
    rclpy.shutdown = lambda: state.update(ok=False)
    rclpy.spin_once = spin_once
    rclpy.__arc_netshim__ = True
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node
    signals = types.ModuleType("rclpy.signals")
    signals.SignalHandlerOptions = types.SimpleNamespace(NO="NO")
    std = types.ModuleType("std_msgs.msg")
    std.Bool, std.String = Bool, String
    sens = types.ModuleType("sensor_msgs.msg")
    sens.CompressedImage = CompressedImage
    vis = types.ModuleType("visualization_msgs.msg")
    vis.Marker, vis.MarkerArray = Marker, MarkerArray
    mods = {"rclpy": rclpy, "rclpy.node": node_mod, "rclpy.signals": signals,
            "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": std,
            "sensor_msgs": types.ModuleType("sensor_msgs"), "sensor_msgs.msg": sens,
            "visualization_msgs": types.ModuleType("visualization_msgs"), "visualization_msgs.msg": vis}
    sys.modules.update(mods)
    print(f"[INFO] netshim: no ROS in this process; sending to {link.dest[0]}:{link.dest[1]} over UDP, "
          f"commands on port {listen_port}", flush=True)
    return link
