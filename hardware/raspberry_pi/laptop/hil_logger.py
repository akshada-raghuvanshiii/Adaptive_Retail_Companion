#!/usr/bin/env python3
"""Record the whole HIL chain on ONE clock (the laptop's), for plot_hil_timeline.py.

    webcam (Pi) -> /person_moving, /person_confused, /arc/target_lock
    BT (laptop) -> /arc/behavior_mode
    motion gate -> /cmd_vel
    motor bridge (Pi) -> /arc_hw/cmd_wheels (sent to the Uno), /arc_hw/wheels (measured by the Hall sensors)
    shopper buttons -> /arc/ui/command

Every message is stamped with the laptop's receive time, so no clock sync between Pi and laptop is
needed. Times include the network hop and, for the BT and motion gate, whatever the simulation load
does to them (they run in sim time; this log is wall time).

    source ~/arc_test_kit/env.sh
    python3 ~/arc_pi/laptop/hil_logger.py                 # -> ~/arc_hil_logs/chain_<time>.csv
    python3 ~/arc_pi/laptop/hil_logger.py --twist         # if /cmd_vel is plain Twist
CSV columns: t (s since start), topic, a, b, c, text.
"""
import argparse
import csv
import json
import os
import signal
import sys
import time

FIELDS = ["t", "topic", "a", "b", "c", "text"]


def row_for(topic, msg):
    """Flatten one message to (a, b, c, text). Pure function, unit-tested."""
    if topic in ("/person_moving", "/person_confused"):
        return int(bool(msg.data)), "", "", ""
    if topic in ("/arc/behavior_mode", "/arc/ui/command"):
        return "", "", "", str(msg.data)
    if topic == "/cmd_vel":
        tw = msg.twist if hasattr(msg, "twist") else msg
        return round(tw.linear.x, 4), round(tw.angular.z, 4), "", ""
    if topic in ("/arc_hw/cmd_wheels", "/arc_hw/wheels"):
        v = list(msg.velocity) + [0.0, 0.0]
        e = list(getattr(msg, "effort", []) or [])
        return round(v[0], 4), round(v[1], 4), (round(e[0], 1) if e else ""), ""
    if topic == "/arc/target_lock":
        try:
            d = json.loads(msg.data)
        except ValueError:
            return "", "", "", "bad json"
        return d.get("fps", ""), d.get("proc_ms", ""), "", f"{d.get('lock', '')}|{d.get('mode', '')}"
    return "", "", "", ""


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--twist", action="store_true", help="/cmd_vel is geometry_msgs/Twist (default TwistStamped)")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/arc_hil_logs"))
    a = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, _sigterm)

    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = Node("arc_hil_logger")
    os.makedirs(a.out_dir, exist_ok=True)
    path = os.path.join(a.out_dir, f"chain_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    fh = open(path, "w", newline="")
    w = csv.writer(fh)
    w.writerow(FIELDS)
    t0 = time.monotonic()
    counts = {}
    topics = {"/person_moving": Bool, "/person_confused": Bool, "/arc/behavior_mode": String,
              "/arc/ui/command": String, "/arc/target_lock": String,
              "/cmd_vel": Twist if a.twist else TwistStamped,
              "/arc_hw/cmd_wheels": JointState, "/arc_hw/wheels": JointState}

    def make_cb(topic):
        def cb(msg):
            w.writerow([f"{time.monotonic() - t0:.4f}", topic, *row_for(topic, msg)])
            counts[topic] = counts.get(topic, 0) + 1
        return cb
    for topic, typ in topics.items():
        node.create_subscription(typ, topic, make_cb(topic), 50)

    def report():
        fh.flush()
        missing = [t for t in topics if t not in counts and t != "/arc/ui/command"]
        node.get_logger().info(f"{sum(counts.values())} msgs" + (f"; nothing yet on {', '.join(missing)}" if missing else ""))
    node.create_timer(10.0, report)
    node.get_logger().info(f"logging to {path} (Ctrl+C to stop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"saved {path}\nplot:  python3 ~/arc_pi/laptop/plot_hil_timeline.py {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
