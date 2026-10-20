#!/usr/bin/env python3
"""Laptop-side safety net for the webcam bridge on the Pi.

The behaviour tree keeps the last /person_moving and /person_confused it received (it has no
staleness check). If the Pi crashes, loses power or the cable is pulled while the flags say "move",
the cart would keep going. This node closes that gap WITHOUT changing the BT:

  * it watches the bridge's heartbeat, /arc/target_lock (one message per processed frame);
  * while the heartbeat is fresh it publishes NOTHING (the Pi is in charge);
  * when no heartbeat arrives for --timeout s (also before the first one), it publishes
    /person_moving=false and /person_confused=false at --rate Hz, so the BT goes to WaitForPerson
    and the motion gate stops the robot. When the Pi comes back, it goes quiet again.

The motors are covered separately: bridge_node.py zeroes the wheels when /cmd_vel stops for 0.5 s,
and the Uno firmware stops on its own after 0.5 s without a command.

Run it only when the BT inputs come from the webcam (PERSON_INPUT=camera). With person_state_input
it would override the keyboard choice.

    source ~/arc_test_kit/env.sh
    python3 ~/arc_pi/laptop/flag_watchdog.py --timeout 3
"""
import argparse
import json
import signal
import sys
import time


def decide(now, last_beat, timeout, last_flags):
    """-> (publish_wait, reason). Fresh heartbeat whose flags are false/false also counts as alive."""
    if last_beat is None:
        return True, "no heartbeat from the webcam bridge yet"
    age = now - last_beat
    if age > timeout:
        return True, f"webcam bridge silent for {age:.1f} s"
    return False, ""


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="s without /arc/target_lock before forcing wait (> 1/fps of the Pi bridge)")
    ap.add_argument("--rate", type=float, default=5.0)
    a = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, _sigterm)

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = Node("arc_flag_watchdog")
    pm = node.create_publisher(Bool, "/person_moving", 10)
    pc = node.create_publisher(Bool, "/person_confused", 10)
    st = {"beat": None, "flags": None, "forcing": None, "host": "?", "fps": None}

    def on_lock(msg):
        st["beat"] = time.monotonic()
        try:
            d = json.loads(msg.data)
            st["flags"] = (d.get("moving"), d.get("confused"))
            st["host"], st["fps"] = d.get("host", "?"), d.get("fps")
        except (ValueError, AttributeError):
            pass

    def tick():
        force, why = decide(time.monotonic(), st["beat"], a.timeout, st["flags"])
        if force:
            pm.publish(Bool(data=False))
            pc.publish(Bool(data=False))
        if force != st["forcing"]:
            if force:
                node.get_logger().warn(f"FORCING WAIT: {why}")
            else:
                node.get_logger().info(f"webcam bridge alive on {st['host']} ({st['fps']} fps): watchdog quiet")
            st["forcing"] = force

    node.create_subscription(String, "/arc/target_lock", on_lock, 10)
    node.create_timer(1.0 / a.rate, tick)
    node.get_logger().info(f"watching /arc/target_lock, timeout {a.timeout} s")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
