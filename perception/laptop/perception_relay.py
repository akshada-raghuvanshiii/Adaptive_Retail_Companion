#!/usr/bin/env python3
"""Laptop side of the no-ROS Pi link: UDP datagrams from perception_bridge.py --udp -> ROS 2 topics.

    source ~/arc_test_kit/env.sh
    python3 ~/arc-perception/ros/perception_relay.py                # listens on UDP 5600

Publishes (same topics and types the bridge publishes when it runs with real ROS):
    /person_moving   std_msgs/Bool    \\ what the behaviour tree reads
    /person_confused std_msgs/Bool    /
    /arc/target_lock std_msgs/String  JSON diagnostics from the Pi (lock, mode, fps, ...)
    /arc/perception_markers  visualization_msgs/MarkerArray  RViz ring + text, drawn here from the diagnostics
    /arc/perception/debug/compressed  sensor_msgs/CompressedImage  the Pi's annotated view (if --debug-image-hz)
Subscribes:
    /arc/ui/command  std_msgs/String  enrol | reset | stop | yes | lead_me | no  -> forwarded to the Pi
                                      (keyboard tool: ~/arc_pi_udp/laptop/arc_ui.py)

Safety: if no datagram arrives from the Pi for --timeout s (Pi off, Wi-Fi drop, bridge crashed), it
publishes moving=false, confused=false at 5 Hz until the Pi is back, so the BT falls to WaitForPerson and
the motion gate stops the cart. Flags from the Pi are only passed on while it is fresh.
Uses plain sockets next to ROS, so env.sh's localhost-only DDS setting is unaffected.
Needs only ROS 2 Python (rclpy, std_msgs, sensor_msgs, visualization_msgs): no YOLO, no venv.
"""
import argparse
import json
import socket
import time

MAGIC_JPEG = b"ARCJ"
UI_COMMANDS = ("enrol", "reset", "stop", "yes", "lead_me", "no")
COLOURS = {"FOLLOW": (0.2, 0.8, 0.2), "LEAD": (0.2, 0.5, 1.0), "WAIT": (1.0, 0.65, 0.0), "LOST": (1.0, 0.25, 0.2),
           "NO PI": (0.6, 0.6, 0.6)}


def parse_datagram(data):
    """-> ('jpeg', bytes) | ('topic', topic, value) | None."""
    if data.startswith(MAGIC_JPEG):
        return ("jpeg", data[len(MAGIC_JPEG):])
    try:
        m = json.loads(data.decode())
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(m, dict) or m.get("arc") != 1 or "topic" not in m:
        return None
    t = m["topic"]
    if t in ("/person_moving", "/person_confused"):
        return ("topic", t, bool(m.get("data")))
    if t == "/arc/target_lock":
        return ("topic", t, str(m.get("data", "")))
    return None


def behaviour(diag, fresh):
    """(label for RViz, colour key) from the Pi's /arc/target_lock JSON."""
    if not fresh:
        return "NO PI: holding WAIT", "NO PI"
    try:
        d = json.loads(diag) if diag else {}
    except ValueError:
        d = {}
    lock = d.get("lock", "?")
    if lock != "LOCKED":
        beh = "LOST" if lock in ("LOST", "NEEDS_ENROL") else "WAIT"
    else:
        beh = "LEAD" if d.get("confused") else ("FOLLOW" if d.get("moving") else "WAIT")
    walking = "WALKING" if d.get("shopper_walking") else "STILL"
    fps = d.get("fps")
    return (f"SHOPPER: {walking if lock == 'LOCKED' else '-'} | LOCK: {lock} | -> {beh}"
            + (f" | Pi {fps} fps" if fps is not None else "")), beh


class Relay:
    """Socket logic without ROS (unit-tested). on_* callbacks are set by the ROS node."""

    def __init__(self, port, timeout, pi_ip="", clock=time.monotonic, sock=None):
        self.timeout, self.pi_ip, self.clock = timeout, pi_ip, clock
        if sock is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", port))
        sock.setblocking(False)
        self.sock = sock
        self.pi_addr = None
        self.last_rx = None
        self.flags = {"/person_moving": False, "/person_confused": False}
        self.diag = ""
        self.n = 0
        self.ignored_from = set()

    def fresh(self):
        return self.last_rx is not None and self.clock() - self.last_rx <= self.timeout

    def drain(self):
        """Read everything waiting -> list of parsed messages (also updates state)."""
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            if self.pi_ip and addr[0] != self.pi_ip:
                self.ignored_from.add(addr[0])
                continue
            m = parse_datagram(data)
            if m is None:
                continue
            self.pi_addr, self.last_rx = addr, self.clock()
            self.n += 1
            if m[0] == "topic":
                if m[1] in self.flags:
                    self.flags[m[1]] = m[2]
                else:
                    self.diag = m[2]
            out.append(m)
        return out

    def send_cmd(self, cmd):
        if self.pi_addr is None:
            return False
        self.sock.sendto(json.dumps({"arc": 1, "cmd": cmd}).encode(), self.pi_addr)
        return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=5600, help="UDP port the Pi sends to")
    ap.add_argument("--timeout", type=float, default=1.5,
                    help="s without data from the Pi before the BT is held in WAIT (Pi YOLO at 1 fps still fits)")
    ap.add_argument("--pi-ip", default="", help="accept data only from this address (default: anyone)")
    a = ap.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = Node("arc_perception_relay")
    log = node.get_logger()
    pub = {"/person_moving": node.create_publisher(Bool, "/person_moving", 10),
           "/person_confused": node.create_publisher(Bool, "/person_confused", 10)}
    pub_diag = node.create_publisher(String, "/arc/target_lock", 10)
    pub_markers = node.create_publisher(MarkerArray, "/arc/perception_markers", 10)
    pub_img = node.create_publisher(CompressedImage, "/arc/perception/debug/compressed", 1)
    relay = Relay(a.port, a.timeout, a.pi_ip)
    st = {"was_fresh": None, "last_hold": 0.0, "last_flags": None, "last_marker": 0.0}

    def markers():
        text, key = behaviour(relay.diag, relay.fresh())
        r, g, b = COLOURS[key]
        ring, label = Marker(), Marker()
        for m, i in ((ring, 0), (label, 1)):
            m.header.frame_id = "base_footprint"
            m.ns, m.id, m.action = "arc_perception", i, Marker.ADD
            m.pose.orientation.w = 1.0
        ring.type = Marker.CYLINDER
        ring.pose.position.z = 0.01
        ring.scale.x = ring.scale.y = 0.45
        ring.scale.z = 0.02
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = r, g, b, 0.6
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.z = 0.55
        label.scale.z = 0.14
        label.color.r, label.color.g, label.color.b, label.color.a = r, g, b, 1.0
        label.text = text
        arr = MarkerArray()
        arr.markers = [ring, label]
        pub_markers.publish(arr)

    def tick():
        for m in relay.drain():
            if m[0] == "jpeg":
                img = CompressedImage()
                img.format = "jpeg"
                img.data = m[1]
                pub_img.publish(img)
            elif m[1] in pub:
                pub[m[1]].publish(Bool(data=m[2]))
            else:
                pub_diag.publish(String(data=m[2]))
        fresh = relay.fresh()
        now = time.monotonic()
        if fresh != st["was_fresh"]:
            if fresh:
                log.info(f"receiving from the Pi at {relay.pi_addr[0]} (UDP {a.port})")
            elif st["was_fresh"] is None:
                log.info(f"waiting for the Pi on UDP {a.port}: BT held in WAIT until it sends")
            else:
                log.warn(f"nothing from the Pi for {a.timeout} s: holding the BT in WAIT (moving=false, confused=false)")
            st["was_fresh"] = fresh
        if not fresh and now - st["last_hold"] >= 0.2:
            st["last_hold"] = now
            for p in pub.values():
                p.publish(Bool(data=False))
        flags = (relay.flags["/person_moving"], relay.flags["/person_confused"]) if fresh else (False, False)
        if flags != st["last_flags"]:
            log.info(f"BT flags moving={flags[0]} confused={flags[1]}")
            st["last_flags"] = flags
        if now - st["last_marker"] >= 0.5:
            st["last_marker"] = now
            markers()

    def on_ui(msg):
        c = str(msg.data).strip().lower()
        if c not in UI_COMMANDS:
            log.info(f"ignored /arc/ui/command '{c}' (use one of {', '.join(UI_COMMANDS)})")
        elif relay.send_cmd(c):
            log.info(f"sent '{c}' to the Pi at {relay.pi_addr[0]}")
        else:
            log.warn(f"'{c}' not sent: no data from the Pi yet, so its address is unknown")

    node.create_subscription(String, "/arc/ui/command", on_ui, 10)
    node.create_timer(0.02, tick)
    log.info(f"listening on UDP {a.port}; on the Pi run perception_bridge.py --udp <this laptop's IP>:{a.port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            for p in pub.values():
                p.publish(Bool(data=False))
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
