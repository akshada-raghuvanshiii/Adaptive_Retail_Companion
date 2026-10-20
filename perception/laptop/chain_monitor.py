#!/usr/bin/env python3
"""Live view of the whole ARC chain, one line twice a second:

    webcam (Pi) -> BT flags -> BT mode -> /cmd_vel (motion gate) -> PWM sent to the Uno -> measured wheels

    source ~/arc_test_kit/env.sh && python3 ~/arc_pi_udp/laptop/chain_monitor.py

Read-only: it only subscribes. Each field shows its age in seconds when it is older than 2 s, so a
stage that has stopped talking is visible at once.
"""
import json
import sys
import time


def fmt_line(s, now):
    def age(k):
        t = s.get(k + "_t")
        if t is None:
            return " (none)"
        return f" ({now - t:.0f}s old)" if now - t > 2.0 else ""
    lock = s.get("lock", "?")
    cam = f"webcam {lock}/{s.get('mode', '?')} {s.get('fps', '?')}fps{age('diag')}"
    flags = f"flags moving={int(s.get('moving', 0))} confused={int(s.get('confused', 0))}{age('moving')}"
    bt = f"BT {s.get('bt', '?')}{age('bt')}"
    cmd = f"cmd_vel v={s.get('v', 0):+.2f} w={s.get('w', 0):+.2f}{age('cmd')}"
    pwm = f"PWM {s.get('pl', 0):.0f}/{s.get('pr', 0):.0f}{age('pwm')}"
    whl = f"wheels {s.get('ml', 0):+.3f}/{s.get('mr', 0):+.3f} m/s{age('meas')}"
    return " | ".join((cam, flags, bt, cmd, pwm, whl))


def main():
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    rclpy.init()
    node = Node("arc_chain_monitor")
    s = {}

    def on_diag(m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        s.update(lock=d.get("lock", "?"), mode=d.get("mode", "?"), fps=d.get("fps", "?"), diag_t=time.time())

    def on_flag(key):
        def cb(m):
            s[key] = bool(m.data)
            s[key + "_t"] = time.time()
        return cb

    def on_bt(m):
        s.update(bt=m.data, bt_t=time.time())

    def on_cmd(m):
        s.update(v=m.twist.linear.x, w=m.twist.angular.z, cmd_t=time.time())

    def on_pwm(m):
        e = list(m.effort) + [0, 0]
        s.update(pl=e[0], pr=e[1], pwm_t=time.time())

    def on_meas(m):
        v = list(m.velocity) + [0, 0]
        s.update(ml=v[0], mr=v[1], meas_t=time.time())

    node.create_subscription(String, "/arc/target_lock", on_diag, 10)
    node.create_subscription(Bool, "/person_moving", on_flag("moving"), 10)
    node.create_subscription(Bool, "/person_confused", on_flag("confused"), 10)
    node.create_subscription(String, "/arc/behavior_mode", on_bt, 10)
    node.create_subscription(TwistStamped, "/cmd_vel", on_cmd, 10)
    node.create_subscription(JointState, "/arc_hw/cmd_wheels", on_pwm, 10)
    node.create_subscription(JointState, "/arc_hw/wheels", on_meas, 10)
    node.create_timer(0.5, lambda: print(fmt_line(s, time.time()), flush=True))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
