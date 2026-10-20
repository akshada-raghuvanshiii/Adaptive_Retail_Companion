#!/usr/bin/env python3
"""ARC hardware-in-the-loop bridge: /cmd_vel from the (simulated) ARC stack -> real motors.

    source ~/arc_test_kit/env.sh
    python3 bridge_node.py --port /dev/ttyACM0 --wheel 0.10 --track 0.40 --ppr 8 --pwm-min 60 --ff 0.6

What it does:
  - subscribes to /cmd_vel (TwistStamped, as ARC's motion gate publishes; --twist for plain Twist);
  - converts it to left/right wheel speeds and sends them to the Arduino at 20 Hz
    (zero if /cmd_vel goes quiet for 0.5 s; the Arduino also has its own watchdog);
  - publishes what the wheels actually do on /arc_hw/wheels (sensor_msgs/JointState:
    velocity = measured wheel speed in m/s, effort = PWM) and /arc_hw/cmd_wheels (commanded, m/s);
  - logs every report with the BT mode (/arc/behavior_mode) to hil_logs/hil_<time>.csv for plot_hil.py.

What it does NOT do: publish /odom or TF. Gazebo owns those during the demo; a second /odom
would corrupt the simulation. Nothing in ARC's packages is changed.
"""
import argparse
import csv
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arc_hw import protocol as P  # noqa: E402

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402


class Bridge(Node):
    def __init__(self, a, ser):
        super().__init__("arc_hw_bridge")
        self.a, self.ser = a, ser
        self.g = P.Geometry(a.wheel, a.track, a.ppr)
        self.v = self.w = 0.0
        self.cmd_t = 0.0
        self.mode = ""
        self.tl = self.tr = 0.0
        self.lock = threading.Lock()
        msg_type = Twist if a.twist else TwistStamped
        self.create_subscription(msg_type, a.cmd_topic, self.on_cmd, 10)
        self.create_subscription(String, "/arc/behavior_mode", self.on_mode, 10)
        self.pub_meas = self.create_publisher(JointState, "/arc_hw/wheels", 10)
        self.pub_cmd = self.create_publisher(JointState, "/arc_hw/cmd_wheels", 10)
        self.create_timer(0.05, self.tick)
        os.makedirs(a.log_dir, exist_ok=True)
        self.log_path = os.path.join(a.log_dir, f"hil_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        self.f = open(self.log_path, "w", newline="")
        self.csv = csv.writer(self.f)
        self.csv.writerow(["t", "bt_mode", "cmd_v", "cmd_w", "target_l_mm_s", "target_r_mm_s",
                           "meas_l_mm_s", "meas_r_mm_s", "pwm_l", "pwm_r", "count_l", "count_r", "arduino_ms"])
        self.t0 = time.time()
        self.buf = b""
        self.send(P.cmd_mm_per_pulse(self.g.mm_per_pulse))
        self.send(P.cmd_gains(a.pwm_min, a.ff, a.ki))
        self.get_logger().info(f"{self.g.mm_per_pulse:.1f} mm/pulse, track {a.track} m, max {a.max_wheel} mm/s. "
                               f"Log: {self.log_path}")

    def send(self, s):
        try:
            self.ser.write(s.encode())
        except Exception as e:  # unplugged cable etc.
            self.get_logger().error(f"serial write failed: {e}")

    def on_cmd(self, msg):
        t = msg.twist if hasattr(msg, "twist") else msg
        with self.lock:
            self.v, self.w, self.cmd_t = t.linear.x, t.angular.z, time.time()

    def on_mode(self, msg):
        self.mode = msg.data

    def tick(self):
        with self.lock:
            fresh = time.time() - self.cmd_t < 0.5
            v, w = (self.v, self.w) if fresh else (0.0, 0.0)
        tl, tr = P.clamp_wheels(*P.twist_to_wheels(v, w, self.a.track), self.a.max_wheel)
        self.tl, self.tr = tl, tr
        self.send(P.cmd_speed(tl, tr))
        now = self.get_clock().now().to_msg()
        js = JointState(name=["left_wheel", "right_wheel"], velocity=[tl / 1000.0, tr / 1000.0])
        js.header.stamp = now
        self.pub_cmd.publish(js)
        # read whatever the Arduino has sent
        try:
            self.buf += self.ser.read(self.ser.in_waiting or 0)
        except Exception as e:
            self.get_logger().error(f"serial read failed: {e}")
            return
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            r = P.parse_report(line.decode(errors="ignore"))
            if not r:
                continue
            m = JointState(name=["left_wheel", "right_wheel"],
                           position=[r.count_l * self.g.mm_per_pulse / 1000.0, r.count_r * self.g.mm_per_pulse / 1000.0],
                           velocity=[r.speed_l / 1000.0, r.speed_r / 1000.0], effort=[float(r.pwm_l), float(r.pwm_r)])
            m.header.stamp = now
            self.pub_meas.publish(m)
            self.csv.writerow([f"{time.time() - self.t0:.3f}", self.mode, f"{v:.4f}", f"{w:.4f}", f"{tl:.1f}",
                               f"{tr:.1f}", r.speed_l, r.speed_r, r.pwm_l, r.pwm_r, r.count_l, r.count_r, r.ms])

    def close(self):
        self.send(P.CMD_STOP)
        self.f.close()


def ros_init(rclpy):
    """rclpy.init without rclpy's own SIGINT/SIGTERM handlers, so Ctrl+C and SIGTERM reach our
    cleanup (send stop to the Arduino) while the ROS context is still alive."""
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


def _sigterm(*_):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wheel", type=float, required=True, help="wheel diameter (m)")
    ap.add_argument("--track", type=float, required=True, help="track width (m)")
    ap.add_argument("--ppr", type=int, required=True, help="Hall pulses per wheel revolution")
    ap.add_argument("--pwm-min", type=float, default=60)
    ap.add_argument("--ff", type=float, default=0.6)
    ap.add_argument("--ki", type=float, default=0.0)
    ap.add_argument("--max-wheel", type=float, default=300.0, help="wheel speed limit (mm/s)")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--twist", action="store_true", help="/cmd_vel is plain Twist, not TwistStamped")
    ap.add_argument("--log-dir", default="hil_logs")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _sigterm)   # pi_down.sh / systemd stop -> same clean exit as Ctrl+C
    try:
        import serial
    except ImportError:
        sys.exit("pyserial missing: sudo apt install python3-serial")
    ser = serial.Serial(a.port, a.baud, timeout=0)
    time.sleep(2.0)                       # Uno resets when the port opens
    ser.reset_input_buffer()
    ros_init(rclpy)
    node = Bridge(a, ser)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        ser.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
