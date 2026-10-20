#!/usr/bin/env python3
"""ARC Arduino <-> ROS 2 bridge (single file). /cmd_vel from ROS -> Arduino Uno -> motors; Hall counts back.

    source ~/arc_test_kit/env.sh          # or on the Pi: source ~/arc_pi/pi/pi_env.sh
    python3 arc_arduino_bridge.py --port /dev/ttyACM0 --wheel 0.10 --track 0.40 --ppr 8

Needs only rclpy (ROS 2 Jazzy) and pyserial (sudo apt install python3-serial).
Talks to firmware/arc_motor_bridge/arc_motor_bridge.ino over USB serial, 115200 baud, one line per message:
    to the Uno:   V <left mm/s> <right mm/s>   wheel speed targets (the Uno runs feed-forward + optional I)
                  K <pwm_min> <ff> <ki>        gains          M <mm per pulse>   wheel geometry
                  S                            stop now
    from the Uno: E <ms> <count_l> <count_r> <speed_l mm/s> <speed_r mm/s> <pwm_l> <pwm_r>   every 50 ms

ROS side:
    subscribes  /cmd_vel              geometry_msgs/TwistStamped (ARC's motion gate); --twist for plain Twist
                /arc/behavior_mode    std_msgs/String, only written to the log
    publishes   /arc_hw/cmd_wheels    sensor_msgs/JointState  velocity = commanded wheel speed, m/s
                /arc_hw/wheels        sensor_msgs/JointState  position = wheel travel (m), velocity = measured
                                      wheel speed (m/s), effort = PWM
                /arc_hw/connected     std_msgs/Bool           true while the Uno is answering

Safety:
    - /cmd_vel older than 0.5 s -> wheels commanded to 0 (and the Uno stops by itself 0.5 s after its
      last command, if this script or the USB link dies);
    - both wheels are scaled together to stay under --max-wheel, so the turn radius is kept;
    - Ctrl+C or SIGTERM sends S (stop) to the Uno before exiting;
    - USB unplugged: the script keeps running, publishes connected=false, and reconnects every second.
    Wheels off the ground for the first runs; the physical E-stop must cut motor power.

It does NOT publish /odom or TF: Gazebo owns those in the ARC demo, and a second /odom would corrupt it.
"""
import argparse
import csv
import math
import os
import signal
import sys
import threading
import time

# ---------------------------------------------------------------- protocol and kinematics (no ROS)


def mm_per_pulse(wheel_diameter_m, pulses_per_rev):
    return math.pi * wheel_diameter_m * 1000.0 / pulses_per_rev


def twist_to_wheels(v, w, track_m):
    """Body velocity (m/s, rad/s) -> left/right wheel speed (mm/s)."""
    half = track_m / 2.0
    return (v - w * half) * 1000.0, (v + w * half) * 1000.0


def clamp_wheels(vl, vr, max_mm_s):
    """Scale both wheels together so neither exceeds max (keeps the turn radius)."""
    m = max(abs(vl), abs(vr))
    if m <= max_mm_s or m == 0:
        return vl, vr
    k = max_mm_s / m
    return vl * k, vr * k


def cmd_speed(vl, vr):
    return f"V {vl:.1f} {vr:.1f}\n"


def cmd_gains(pwm_min, ff, ki):
    return f"K {pwm_min:.1f} {ff:.4f} {ki:.4f}\n"


def cmd_mm_per_pulse(mm):
    return f"M {mm:.3f}\n"


CMD_STOP = "S\n"


def parse_report(line):
    """'E ms cl cr sl sr pl pr' -> tuple (ms, cl, cr, sl, sr, pl, pr), or None (banner, noise)."""
    p = line.strip().split()
    if len(p) != 8 or p[0] != "E":
        return None
    try:
        return (int(p[1]), int(p[2]), int(p[3]), float(p[4]), float(p[5]), int(p[6]), int(p[7]))
    except ValueError:
        return None


class Link:
    """USB serial to the Uno, with reconnect. All methods are safe to call while unplugged."""

    def __init__(self, port, baud, on_open, log):
        self.port, self.baud, self.on_open, self.log = port, baud, on_open, log
        self.ser = None
        self.buf = b""
        self.next_try = 0.0
        self.last_rx = 0.0

    def ok(self):
        return self.ser is not None

    def ensure(self):
        if self.ser is not None or time.time() < self.next_try:
            return self.ser is not None
        self.next_try = time.time() + 1.0
        import serial
        try:
            s = serial.Serial(self.port, self.baud, timeout=0)
        except (serial.SerialException, OSError) as e:
            self.log(f"cannot open {self.port}: {e} (retrying every 1 s)", warn=True)
            return False
        time.sleep(2.0)                    # the Uno resets when the port opens
        s.reset_input_buffer()
        self.ser, self.buf, self.last_rx = s, b"", time.time()
        self.log(f"connected to {self.port}")
        self.on_open()
        return True

    def drop(self, why):
        if self.ser is not None:
            self.log(f"serial link lost: {why}", warn=True)
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def write(self, s):
        if self.ser is None:
            return
        try:
            self.ser.write(s.encode())
        except Exception as e:
            self.drop(e)

    def lines(self):
        if self.ser is None:
            return []
        try:
            self.buf += self.ser.read(self.ser.in_waiting or 0)
        except Exception as e:
            self.drop(e)
            return []
        out = []
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            out.append(line.decode(errors="ignore"))
        if out:
            self.last_rx = time.time()
        return out

    def close(self, final=""):
        if self.ser is not None and final:
            try:
                self.ser.write(final.encode())
                self.ser.flush()
            except Exception:
                pass
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


# ---------------------------------------------------------------- ROS node


def make_node(a):
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String

    class ArduinoBridge(Node):
        def __init__(self):
            super().__init__("arc_arduino_bridge")
            self.mmpp = mm_per_pulse(a.wheel, a.ppr)
            self.v = self.w = 0.0
            self.cmd_t = 0.0
            self.mode = ""
            self.lock = threading.Lock()
            self.n_reports = 0
            self.create_subscription(Twist if a.twist else TwistStamped, a.cmd_topic, self.on_cmd, 10)
            self.create_subscription(String, "/arc/behavior_mode", self.on_mode, 10)
            self.pub_meas = self.create_publisher(JointState, "/arc_hw/wheels", 10)
            self.pub_cmd = self.create_publisher(JointState, "/arc_hw/cmd_wheels", 10)
            self.pub_conn = self.create_publisher(Bool, "/arc_hw/connected", 10)
            self.csv = self.f = None
            if a.log_dir:
                os.makedirs(a.log_dir, exist_ok=True)
                path = os.path.join(a.log_dir, f"hil_{time.strftime('%Y%m%d_%H%M%S')}.csv")
                self.f = open(path, "w", newline="")
                self.csv = csv.writer(self.f)
                self.csv.writerow(["t", "bt_mode", "cmd_v", "cmd_w", "target_l_mm_s", "target_r_mm_s",
                                   "meas_l_mm_s", "meas_r_mm_s", "pwm_l", "pwm_r", "count_l", "count_r", "arduino_ms"])
                self.get_logger().info(f"log: {path}")
            self.t0 = time.time()
            self.link = Link(a.port, a.baud, self.on_open, self.log)
            self.get_logger().info(f"{self.mmpp:.1f} mm per pulse, track {a.track} m, max {a.max_wheel} mm/s, "
                                   f"listening on {a.cmd_topic} ({'Twist' if a.twist else 'TwistStamped'})")
            self.create_timer(1.0 / a.rate, self.tick)
            self.create_timer(5.0, self.status)

        def log(self, text, warn=False):
            (self.get_logger().warn if warn else self.get_logger().info)(text)

        def on_open(self):                  # (re)connected: send geometry and gains first
            self.link.write(cmd_mm_per_pulse(self.mmpp))
            self.link.write(cmd_gains(a.pwm_min, a.ff, a.ki))

        def on_cmd(self, msg):
            t = msg.twist if hasattr(msg, "twist") else msg
            with self.lock:
                self.v, self.w, self.cmd_t = t.linear.x, t.angular.z, time.time()

        def on_mode(self, msg):
            self.mode = msg.data

        def tick(self):
            with self.lock:
                fresh = time.time() - self.cmd_t < a.cmd_timeout
                v, w = (self.v, self.w) if fresh else (0.0, 0.0)
            tl, tr = clamp_wheels(*twist_to_wheels(v, w, a.track), a.max_wheel)
            now = self.get_clock().now().to_msg()
            js = JointState(name=["left_wheel", "right_wheel"], velocity=[tl / 1000.0, tr / 1000.0])
            js.header.stamp = now
            self.pub_cmd.publish(js)
            if not self.link.ensure():
                self.pub_conn.publish(Bool(data=False))
                return
            self.link.write(cmd_speed(tl, tr))
            for line in self.link.lines():
                r = parse_report(line)
                if r is None:
                    continue
                ms, cl, cr, sl, sr, pl, pr = r
                self.n_reports += 1
                m = JointState(name=["left_wheel", "right_wheel"],
                               position=[cl * self.mmpp / 1000.0, cr * self.mmpp / 1000.0],
                               velocity=[sl / 1000.0, sr / 1000.0], effort=[float(pl), float(pr)])
                m.header.stamp = now
                self.pub_meas.publish(m)
                if self.csv:
                    self.csv.writerow([f"{time.time() - self.t0:.3f}", self.mode, f"{v:.4f}", f"{w:.4f}",
                                       f"{tl:.1f}", f"{tr:.1f}", sl, sr, pl, pr, cl, cr, ms])
            alive = self.link.ok() and time.time() - self.link.last_rx < 1.0
            if self.link.ok() and not alive and time.time() - self.link.last_rx > 3.0:
                self.link.drop("no reports from the Uno for 3 s (right sketch? right port?)")
            self.pub_conn.publish(Bool(data=alive))

        def status(self):
            with self.lock:
                age = time.time() - self.cmd_t
            cmd = f"/cmd_vel v={self.v:.2f} w={self.w:.2f}" if age < a.cmd_timeout else "no fresh /cmd_vel (wheels at 0)"
            self.get_logger().info(f"{'Uno OK' if self.link.ok() else 'Uno NOT connected'}, "
                                   f"{self.n_reports} reports, {cmd}")

        def close(self):
            self.link.close(final=CMD_STOP)
            if self.f:
                self.f.close()

    return ArduinoBridge()


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0", help="Uno serial port (Uno clones: often /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wheel", type=float, required=True, help="wheel diameter, m")
    ap.add_argument("--track", type=float, required=True, help="distance between the two wheel centres, m")
    ap.add_argument("--ppr", type=int, required=True, help="Hall pulses (magnets) per wheel revolution")
    ap.add_argument("--pwm-min", type=float, default=60, help="PWM where the wheel starts to turn (bench.py calibrate)")
    ap.add_argument("--ff", type=float, default=0.6, help="PWM per mm/s above pwm-min (bench.py calibrate)")
    ap.add_argument("--ki", type=float, default=0.0, help="integral gain on the Uno (0 = feed-forward only)")
    ap.add_argument("--max-wheel", type=float, default=300.0, help="wheel speed limit, mm/s")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--twist", action="store_true", help="the command topic is plain Twist, not TwistStamped")
    ap.add_argument("--cmd-timeout", type=float, default=0.5, help="s without /cmd_vel before the wheels go to 0")
    ap.add_argument("--rate", type=float, default=20.0, help="commands per second to the Uno")
    ap.add_argument("--log-dir", default="hil_logs", help="CSV log folder ('' = no log)")
    from rclpy.utilities import remove_ros_args
    a = ap.parse_args(remove_ros_args(sys.argv if argv is None else argv)[1:])
    try:
        import serial  # noqa: F401
    except ImportError:
        sys.exit("pyserial missing: sudo apt install python3-serial")

    import rclpy
    try:                                   # keep Ctrl+C / SIGTERM for our own clean stop
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    signal.signal(signal.SIGTERM, _sigterm)
    node = make_node(a)
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
