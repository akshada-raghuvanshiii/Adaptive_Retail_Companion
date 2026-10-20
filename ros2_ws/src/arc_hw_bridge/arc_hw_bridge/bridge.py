#!/usr/bin/env python3
"""ARC motor bridge: /cmd_vel from ARC's motion gate -> the team's Uno sketch -> SmartElex 15S drivers -> wheels.

    ros2 launch arc_hw_bridge arc_hw.launch.py                     # settings from config/arc_hw.yaml
    ros2 run arc_hw_bridge motor_bridge --ros-args -p port:=/dev/ttyACM1

Subscribes  /cmd_vel             geometry_msgs/TwistStamped (ARC's motion gate); twist:=true for plain Twist
            /arc/behavior_mode   std_msgs/String, only written to the log
Publishes   /arc_hw/cmd_wheels   sensor_msgs/JointState  velocity = target wheel speed (m/s), effort = PWM sent
            /arc_hw/wheels       sensor_msgs/JointState  position = wheel travel (m), velocity = measured speed
                                                         (m/s, 1 s average from ENC), effort = PWM sent
            /arc_hw/connected    std_msgs/Bool           true while the sketch answers
            /arc_hw/estop_ok     std_msgs/Bool           false while the Uno answers ERR,ESTOP

Speed control is feed-forward here (the sketch takes raw PWM), per wheel:
|PWM| = max(pwm_start, pwm_min + ff * |wheel mm/s|), capped at max_pwm (the sketch caps at 60 itself).
The values come from `ros2 run arc_hw_bridge motor_bench calibrate`, which prints them for each wheel.

Safety: wheels go to 0 when /cmd_vel is older than cmd_timeout; the sketch stops by itself 300 ms after its
last M command and whenever its E-stop loop opens; Ctrl+C / SIGTERM send STOP first.
It does NOT publish /odom or TF: Gazebo owns those in the ARC simulation, and a second /odom would corrupt it.
"""
import csv
import os
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from . import protocol as P
from .link import Link

PARAMS = {                       # name: (default, description)
    "port": ("/dev/ttyACM0", "Uno serial port (clones: often /dev/ttyUSB0)"),
    "baud": (115200, "must match Serial.begin in the sketch"),
    "wheel_diameter": (0.10, "m, measured on the cart"),
    "track_width": (0.40, "m, centre of left tyre to centre of right tyre"),
    "pulses_per_rev": (210, "encoder pulses per WHEEL turn = encoder PPR x gear ratio"),
    "pwm_start_left": (25.0, "lowest PWM at which the left wheel breaks free (motor_bench calibrate)"),
    "pwm_min_left": (20.0, "left: PWM = pwm_min + ff * mm/s, line through the moving points (motor_bench calibrate)"),
    "ff_left": (0.2, "left: PWM per mm/s (motor_bench calibrate)"),
    "pwm_start_right": (25.0, "lowest PWM at which the right wheel breaks free"),
    "pwm_min_right": (20.0, "right: line offset"),
    "ff_right": (0.2, "right: PWM per mm/s"),
    "max_pwm": (60, "PWM cap; the sketch's MAX_PWM is 60, higher values are clamped there anyway"),
    "deadband": (5.0, "mm/s; smaller wheel targets are sent as 0"),
    "invert_left": (False, "flip the left motor direction (if it turns backwards)"),
    "invert_right": (False, "flip the right motor direction"),
    "max_wheel_speed": (300.0, "mm/s limit per wheel"),
    "cmd_topic": ("/cmd_vel", "velocity command topic"),
    "twist": (False, "true if cmd_topic is plain Twist instead of TwistStamped"),
    "cmd_timeout": (0.5, "s without a command before the wheels are set to 0"),
    "rate": (20.0, "M commands per second (the sketch's watchdog needs at least 5)"),
    "telemetry_period": (1.0, "s, the sketch's TELEMETRY_MS / 1000"),
    "noise_pulses": (5, "pulses per period while stopped above which a sensor-noise warning is printed"),
    "log_dir": ("~/arc_hw_logs", "CSV log folder; empty = no log"),
}


class WheelState:
    """Per-wheel bookkeeping between two ENC reports."""

    def __init__(self):
        self.sign = 1            # direction of the last non-zero command
        self.target_sum = 0.0
        self.n = 0
        self.moved = False       # any non-zero PWM since the last ENC
        self.travel_m = 0.0

    def add_target(self, mm_s, pwm):
        self.target_sum += mm_s
        self.n += 1
        if pwm != 0:
            self.sign = 1 if pwm > 0 else -1
            self.moved = True

    def close_period(self):
        avg = self.target_sum / self.n if self.n else 0.0
        moved = self.moved
        self.target_sum, self.n, self.moved = 0.0, 0, False
        return avg, moved


class MotorBridge(Node):
    def __init__(self):
        super().__init__("arc_hw_bridge")
        for name, (default, _) in PARAMS.items():
            self.declare_parameter(name, default)
        self.p = {n: self.get_parameter(n).value for n in PARAMS}
        if self.p["wheel_diameter"] <= 0 or self.p["track_width"] <= 0 or self.p["pulses_per_rev"] <= 0:
            raise ValueError("wheel_diameter, track_width and pulses_per_rev must be > 0")
        if float(self.p["rate"]) < 5.0:
            raise ValueError("rate must be >= 5 Hz: the sketch stops the motors after 300 ms without M")
        self.geo = P.Geometry(float(self.p["wheel_diameter"]), float(self.p["track_width"]),
                              int(self.p["pulses_per_rev"]))
        self.max_pwm = min(int(self.p["max_pwm"]), P.MAX_PWM_SKETCH)
        self.v = self.w = 0.0
        self.cmd_t = 0.0
        self.mode = ""
        self.pwm = (0, 0)
        self.targets = (0.0, 0.0)
        self.wheels = (WheelState(), WheelState())
        self.lock = threading.Lock()
        self.create_subscription(Twist if self.p["twist"] else TwistStamped, self.p["cmd_topic"], self.on_cmd, 10)
        self.create_subscription(String, "/arc/behavior_mode", self.on_mode, 10)
        self.pub_meas = self.create_publisher(JointState, "/arc_hw/wheels", 10)
        self.pub_cmd = self.create_publisher(JointState, "/arc_hw/cmd_wheels", 10)
        self.pub_conn = self.create_publisher(Bool, "/arc_hw/connected", 10)
        self.pub_estop = self.create_publisher(Bool, "/arc_hw/estop_ok", 10)
        self.f = self.csv = None
        log_dir = os.path.expanduser(str(self.p["log_dir"]))
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, f"hil_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            self.f = open(path, "w", newline="")
            self.csv = csv.writer(self.f)
            self.csv.writerow(["t", "bt_mode", "cmd_v", "cmd_w", "target_l_mm_s", "target_r_mm_s",
                               "meas_l_mm_s", "meas_r_mm_s", "pwm_l", "pwm_r", "enc_l", "enc_r", "estop_open"])
            self.get_logger().info(f"log: {path}")
        self.t0 = time.time()
        self.link = Link(self.p["port"], int(self.p["baud"]), log=self.log)
        self.ffw = [tuple(float(self.p[f"{k}_{s}"]) for k in ("pwm_start", "pwm_min", "ff")) for s in ("left", "right")]
        for s, (st, mn, ff) in zip(("left", "right"), self.ffw):
            if ff <= 0:
                raise ValueError(f"ff_{s} must be > 0 (got {ff}); rerun motor_bench calibrate")
        tops = [P.max_speed_for_pwm(mn, ff, self.max_pwm) for _, mn, ff in self.ffw]
        self.get_logger().info(
            f"{self.geo.mm_per_pulse:.2f} mm/pulse, track {self.geo.track_width_m} m, PWM cap {self.max_pwm}: "
            f"top speed about {tops[0]:.0f} (left) / {tops[1]:.0f} (right) mm/s, "
            f"listening on {self.p['cmd_topic']} ({'Twist' if self.p['twist'] else 'TwistStamped'})")
        self.noise_warned = [False, False]
        self.create_timer(1.0 / float(self.p["rate"]), self.tick)
        self.create_timer(5.0, self.status)

    def log(self, text, warn=False):
        (self.get_logger().warn if warn else self.get_logger().info)(text)

    def on_cmd(self, msg):
        t = msg.twist if hasattr(msg, "twist") else msg
        with self.lock:
            self.v, self.w, self.cmd_t = t.linear.x, t.angular.z, time.time()

    def on_mode(self, msg):
        self.mode = msg.data

    def compute(self, v, w):
        """Body twist -> (target mm/s, PWM to send) per wheel."""
        tl, tr = P.clamp_wheels(*P.twist_to_wheels(v, w, self.geo.track_width_m), float(self.p["max_wheel_speed"]))
        db = float(self.p["deadband"])
        (sl, ml, fl), (sr, mr, fr) = self.ffw
        pl = P.speed_to_pwm(tl, ml, fl, self.max_pwm, db, pwm_start=sl)
        pr = P.speed_to_pwm(tr, mr, fr, self.max_pwm, db, pwm_start=sr)
        return (tl, tr), (pl, pr)

    def tick(self):
        with self.lock:
            fresh = time.time() - self.cmd_t < float(self.p["cmd_timeout"])
            v, w = (self.v, self.w) if fresh else (0.0, 0.0)
        (tl, tr), (pl, pr) = self.compute(v, w)
        self.targets, self.pwm = (tl, tr), (pl, pr)
        now = self.get_clock().now().to_msg()
        js = JointState(name=["left_wheel", "right_wheel"], velocity=[tl / 1000.0, tr / 1000.0],
                        effort=[float(pl), float(pr)])
        js.header.stamp = now
        self.pub_cmd.publish(js)
        if not self.link.ensure():
            self.pub_conn.publish(Bool(data=False))
            return
        sl = -pl if self.p["invert_left"] else pl
        sr = -pr if self.p["invert_right"] else pr
        self.link.write(P.cmd_motor(sl, sr))
        refused = bool(self.link.estop_open)          # the sketch drives nothing while its E-stop is open
        self.wheels[0].add_target(tl, 0 if refused else pl)
        self.wheels[1].add_target(tr, 0 if refused else pr)
        for m in self.link.read():
            if m[0] == "enc":
                self.on_enc(m[1], m[2], v, w, now)
        self.pub_conn.publish(Bool(data=self.link.alive()))
        if self.link.estop_open is not None:
            self.pub_estop.publish(Bool(data=not self.link.estop_open))

    def on_enc(self, el, er, v, w, stamp):
        mm, period = self.geo.mm_per_pulse, float(self.p["telemetry_period"])
        meas, avg_t = [], []
        for i, (ws, pulses) in enumerate(zip(self.wheels, (el, er))):
            avg, moved = ws.close_period()
            speed = P.signed_speed(pulses, ws.sign, mm, period)
            ws.travel_m += speed * period / 1000.0
            meas.append(speed)
            avg_t.append(avg)
            if not moved and pulses > int(self.p["noise_pulses"]) and not self.noise_warned[i]:
                side = "left" if i == 0 else "right"
                self.get_logger().warn(
                    f"{side} encoder: {pulses} pulses in {period:.0f} s while that wheel was commanded 0. "
                    "Electrical noise or a loose encoder connector (VCC, GND, channel A).")
                self.noise_warned[i] = True
        m = JointState(name=["left_wheel", "right_wheel"],
                       position=[self.wheels[0].travel_m, self.wheels[1].travel_m],
                       velocity=[meas[0] / 1000.0, meas[1] / 1000.0],
                       effort=[float(self.pwm[0]), float(self.pwm[1])])
        m.header.stamp = stamp
        self.pub_meas.publish(m)
        if self.csv:
            self.csv.writerow([f"{time.time() - self.t0:.3f}", self.mode, f"{v:.4f}", f"{w:.4f}",
                               f"{avg_t[0]:.1f}", f"{avg_t[1]:.1f}", f"{meas[0]:.1f}", f"{meas[1]:.1f}",
                               self.pwm[0], self.pwm[1], el, er, int(bool(self.link.estop_open))])
            self.f.flush()

    def status(self):
        with self.lock:
            age = time.time() - self.cmd_t
            v, w = self.v, self.w
        cmd = (f"/cmd_vel v={v:.2f} w={w:.2f} -> PWM {self.pwm[0]}/{self.pwm[1]}"
               if age < float(self.p["cmd_timeout"]) else "no fresh /cmd_vel (wheels at 0)")
        d = self.link.diagnosis()
        bad = d.startswith(("WRONG", "E-STOP", "Uno NOT"))
        (self.get_logger().warn if bad else self.get_logger().info)(f"{d} | {cmd}")

    def close(self):
        self.link.write(P.CMD_STOP)
        if self.link.ser is not None:
            try:
                self.link.ser.flush()
                self.link.ser.close()
            except Exception:
                pass
        if self.f:
            self.f.close()


def _sigterm(*_):
    raise KeyboardInterrupt


def main(args=None):
    try:
        import serial  # noqa: F401
    except ImportError:
        raise SystemExit("pyserial missing: sudo apt install python3-serial")
    try:                                     # keep Ctrl+C / SIGTERM for our own clean stop (send STOP first)
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    signal.signal(signal.SIGTERM, _sigterm)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
