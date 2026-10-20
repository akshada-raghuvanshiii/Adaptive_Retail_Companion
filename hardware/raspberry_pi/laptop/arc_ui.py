#!/usr/bin/env python3
"""Shopper buttons for the headless webcam bridge (on the Pi), from a laptop terminal.

    source ~/arc_test_kit/env.sh
    python3 ~/arc_pi/laptop/arc_ui.py

Keys (no Enter needed):
    e  enrol the largest person in view (stand alone in front of the camera)
    r  reset the lock
    l  "lead me"       y  yes        n  no        s  stop
    q  quit
Each key publishes one std_msgs/String on /arc/ui/command. The status line below shows what the
bridge reports on /arc/target_lock (lock, mode, BT flags, fps on the Pi) and the BT mode.
"""
import json
import select
import sys
import termios
import time
import tty

KEYS = {"e": "enrol", "r": "reset", "l": "lead_me", "y": "yes", "n": "no", "s": "stop"}


def status_line(diag, mode):
    if not diag:
        return f"bridge: (no /arc/target_lock yet) | BT: {mode or '?'}"
    return (f"lock {diag.get('lock', '?'):<11} mode {diag.get('mode', '-'):<10} "
            f"moving={diag.get('moving')} confused={diag.get('confused')} | "
            f"{diag.get('fps', '?')} fps on {diag.get('host', '?')} | BT: {mode or '?'}")


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    node = Node("arc_ui_keys")
    pub = node.create_publisher(String, "/arc/ui/command", 10)
    st = {"diag": None, "mode": None, "t": 0.0}

    def on_diag(m):
        try:
            st["diag"] = json.loads(m.data)
        except ValueError:
            pass

    def on_mode(m):
        st["mode"] = m.data
    node.create_subscription(String, "/arc/target_lock", on_diag, 10)
    node.create_subscription(String, "/arc/behavior_mode", on_mode, 10)
    print(__doc__.split("Keys", 1)[1].split("Each key")[0])
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1).lower()
                if k == "q":
                    break
                if k in KEYS:
                    pub.publish(String(data=KEYS[k]))
                    print(f"\n-> sent '{KEYS[k]}'")
            if time.monotonic() - st["t"] > 0.5:
                st["t"] = time.monotonic()
                sys.stdout.write("\r" + status_line(st["diag"], st["mode"])[:150].ljust(150))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
