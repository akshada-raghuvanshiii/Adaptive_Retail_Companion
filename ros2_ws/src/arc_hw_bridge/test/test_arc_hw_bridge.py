"""Offline tests: the bridge and bench against a simulated copy of the team's Uno sketch. No ROS, no Uno.

    cd ~/slam_ws/src/arc_hw_bridge && python3 -m unittest discover -s test -v
"""
import importlib
import os
import sys
import tempfile
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from arc_hw_bridge import protocol as P  # noqa: E402
from arc_hw_bridge.link import Link  # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def time(self):
        return self.t

    def sleep(self, s):
        self.t += s


class FakeSketch:
    """Behaves like firmware/arc_cart_motor/arc_cart_motor.ino as seen over USB."""

    def __init__(self, clock, estop_closed=True, noise_right=0, speed=lambda pwm: max(0, abs(pwm) - 20) * 4.0,
                 mm_per_pulse=P.Geometry(0.10, 0.4, 8).mm_per_pulse, other_sketch=False):
        self.c, self.estop_closed, self.noise_right = clock, estop_closed, noise_right
        self.speed, self.mmpp, self.other = speed, mm_per_pulse, other_sketch
        self.out = b"ARC_MOTOR_READY\r\n" if not other_sketch else b""
        self.inbuf = b""
        self.pwm = [0, 0]
        self.last_cmd = clock.time()
        self.last_tel = clock.time()
        self.acc = [0.0, 0.0]
        self.sim_t = clock.time()
        self.written = []
        self.closed = False

    # --- the sketch's loop, run up to "now"
    def _advance(self):
        now = self.c.time()
        while self.sim_t < now:
            dt = min(0.01, now - self.sim_t)
            self.sim_t += dt
            if self.sim_t - self.last_cmd > P.SKETCH_WATCHDOG_S or not self.estop_closed:
                self.pwm = [0, 0]
            for i in (0, 1):
                self.acc[i] += self.speed(self.pwm[i]) * dt / self.mmpp
            self.acc[1] += self.noise_right * dt
            if self.sim_t - self.last_tel >= P.TELEMETRY_PERIOD_S:
                self.last_tel = self.sim_t
                l, r = int(self.acc[0]), int(self.acc[1])
                self.acc = [self.acc[0] - l, self.acc[1] - r]
                if self.other:
                    self.out += f"ENC,0,{r}\r\n".encode()   # same prefix, never answers M -> still "other"? see test
                else:
                    self.out += f"ENC,{l},{r}\r\n".encode()

    def _handle(self, cmd):
        if self.other:
            return
        if cmd == "STOP":
            self.pwm = [0, 0]; self.last_cmd = self.sim_t; self.out += b"OK,STOP\r\n"
        elif cmd == "PING":
            self.out += b"ARC_MOTOR_READY\r\n"
        elif cmd.startswith("M,"):
            try:
                l, r = (int(x) for x in cmd[2:].split(","))
            except ValueError:
                self.out += b"ERR,COMMAND\r\n"; return
            if not self.estop_closed:
                self.pwm = [0, 0]; self.out += b"ERR,ESTOP\r\n"; return
            self.pwm = [max(-60, min(60, l)), max(-60, min(60, r))]
            self.last_cmd = self.sim_t
            self.out += b"OK\r\n"
        else:
            self.out += b"ERR,COMMAND\r\n"

    # --- pyserial surface
    def write(self, b):
        self._advance()
        self.written.append(b.decode())
        self.inbuf += b
        while b"\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\n", 1)
            self._handle(line.decode().strip())

    @property
    def in_waiting(self):
        self._advance()
        return len(self.out)

    def read(self, n):
        self._advance()
        d, self.out = self.out[:n], self.out[n:]
        return d

    def reset_input_buffer(self):
        self.out = b""

    def flush(self):
        pass

    def close(self):
        self.closed = True


class ProtocolTests(unittest.TestCase):
    def test_parse_lines(self):
        self.assertEqual(P.parse_line("ENC,0,623\r"), ("enc", 0, 623))
        self.assertEqual(P.parse_line("OK"), ("ok",))
        self.assertEqual(P.parse_line("OK,STOP"), ("ok_stop",))
        self.assertEqual(P.parse_line("ARC_MOTOR_READY"), ("ready",))
        self.assertEqual(P.parse_line("ERR,ESTOP"), ("err", "ESTOP"))
        self.assertIsNone(P.parse_line("E 1 2 3 4 5 6 7"))
        self.assertIsNone(P.parse_line("ENC,x,1"))

    def test_speed_to_pwm(self):
        self.assertEqual(P.speed_to_pwm(0, 25, 0.2, 60), 0)
        self.assertEqual(P.speed_to_pwm(3, 25, 0.2, 60), 0)               # deadband
        self.assertEqual(P.speed_to_pwm(100, 25, 0.2, 60), 45)
        self.assertEqual(P.speed_to_pwm(-100, 25, 0.2, 60), -45)
        self.assertEqual(P.speed_to_pwm(1000, 25, 0.2, 60), 60)           # capped like the sketch
        self.assertAlmostEqual(P.max_speed_for_pwm(25, 0.2, 60), 175.0)
        self.assertEqual(P.cmd_motor(45, -45), "M,45,-45\n")

    def test_fit_wheel_on_the_real_16_oct_sweep(self):
        """Pulses/s from the first real calibrate run (ttyACM1, 16 Oct 16:26): left breaks free late and drops at 60."""
        pwm = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60]
        left = [0, 0, 0, 0, 0, 0, 0, 0, 63.25, 76.0, 88.0, 101.25, 26.25]
        right = [0, 0, 0, 18.75, 26.5, 34.0, 41.25, 48.75, 55.75, 63.0, 70.0, 77.0, 85.25]
        fl = P.fit_wheel(list(zip(pwm, left)))
        fr = P.fit_wheel(list(zip(pwm, right)))
        self.assertEqual(fl["start"], 40.0)
        self.assertEqual(fl["dropped"], [60])
        self.assertGreater(fl["ff"], 0)
        self.assertAlmostEqual(fl["pwm_min"] + fl["ff"] * 76.0, 45, delta=1.0)   # reproduces its own points
        self.assertEqual(fr["start"], 15.0)
        self.assertEqual(fr["dropped"], [])
        self.assertAlmostEqual(fr["pwm_min"] + fr["ff"] * 63.0, 45, delta=1.0)
        # the breakaway floor: a slow target still gets at least the start PWM
        self.assertEqual(P.speed_to_pwm(10, fl["pwm_min"], fl["ff"], 60, pwm_start=fl["start"]), 40)
        self.assertIsNone(P.fit_wheel([(0, 0), (40, 0), (60, 0)]))

    def test_kinematics_and_fit(self):
        self.assertEqual(P.twist_to_wheels(0.1, 0.0, 0.4), (100.0, 100.0))
        l, r = P.clamp_wheels(200, 400, 300)
        self.assertAlmostEqual(l / r, 0.5)
        pts = [(p, max(0, p - 20) * 4.0) for p in range(0, 61, 5)]
        pmin, ff = P.fit_feedforward(pts)
        self.assertAlmostEqual(pmin, 20.0, places=6)
        self.assertAlmostEqual(ff, 0.25, places=6)
        self.assertAlmostEqual(P.signed_speed(10, -1, 39.27), -392.7)


class LinkTests(unittest.TestCase):
    def make(self, **kw):
        c = Clock()
        s = FakeSketch(c, **kw)
        logs = []
        L = Link("/dev/fake", 115200, opener=lambda p, b: s, clock=c.time, sleep=c.sleep,
                 log=lambda t, warn=False: logs.append(t))
        self.assertTrue(L.ensure())
        return c, s, L, logs

    def pump(self, c, L, seconds, cmd=(0, 0)):
        out = []
        for _ in range(int(seconds * 20)):
            L.ensure()
            L.write(P.cmd_motor(*cmd))
            out += L.read()
            c.sleep(0.05)
        return out

    def test_healthy(self):
        c, s, L, _ = self.make()
        msgs = self.pump(c, L, 3.5)
        self.assertIn(("ready",), msgs)                                    # PING answered
        self.assertGreaterEqual(sum(m[0] == "enc" for m in msgs), 3)
        self.assertTrue(L.alive())
        self.assertFalse(L.estop_open)
        self.assertTrue(L.diagnosis().startswith("Uno OK"))

    def test_estop_open_is_reported(self):
        c, s, L, logs = self.make(estop_closed=False)
        self.pump(c, L, 4, cmd=(40, 40))
        self.assertTrue(L.estop_open)
        self.assertTrue(L.diagnosis().startswith("E-STOP OPEN"), L.diagnosis())
        self.assertTrue(any("ERR,ESTOP" in t for t in logs))
        self.assertEqual(s.pwm, [0, 0])

    def test_wrong_sketch(self):
        c = Clock()

        class Other(FakeSketch):          # prints something this protocol does not know, ignores commands
            def _advance(self):
                now = self.c.time()
                if now - self.last_tel >= 1.0:
                    self.last_tel = now
                    self.out += b"E 123 0 0 0.0 0.0 0 0\r\n"
        s = Other(c, other_sketch=True)
        L = Link("/dev/fake", 115200, opener=lambda p, b: s, clock=c.time, sleep=c.sleep)
        L.ensure()
        self.pump(c, L, 4)
        self.assertTrue(L.diagnosis().startswith("WRONG SKETCH"), L.diagnosis())

    def test_silent_uno_is_dropped_and_reopened(self):
        c = Clock()

        class Dead(FakeSketch):
            def _advance(self):
                pass

            def _handle(self, cmd):
                pass
        opened = []
        L = Link("/dev/fake", 115200, opener=lambda p, b: opened.append(1) or Dead(c), clock=c.time, sleep=c.sleep)
        L.ensure()
        self.pump(c, L, 4)
        self.assertGreaterEqual(len(opened), 2)

    def test_permission_hint(self):
        def deny(p, b):
            raise PermissionError("[Errno 13] Permission denied: '/dev/ttyACM0'")
        logs = []
        L = Link("/dev/ttyACM0", 115200, opener=deny, log=lambda t, warn=False: logs.append(t))
        self.assertFalse(L.ensure())
        self.assertIn("dialout", logs[0])


# ----------------------------------------------------------------- ROS stubs for the node test

def install_ros_stubs():
    class Msg:
        def __init__(self, **kw):
            self.header = types.SimpleNamespace(stamp=None)
            for k, v in kw.items():
                setattr(self, k, v)

    class Twist:
        def __init__(self, v=0.0, w=0.0):
            self.linear = types.SimpleNamespace(x=v)
            self.angular = types.SimpleNamespace(z=w)

    class TwistStamped:
        def __init__(self, v=0.0, w=0.0):
            self.twist = Twist(v, w)

    class Pub:
        def __init__(self):
            self.msgs = []

        def publish(self, m):
            self.msgs.append(m)

    class Logger:
        def __init__(self):
            self.lines = []

        def info(self, t):
            self.lines.append(("info", t))

        def warn(self, t):
            self.lines.append(("warn", t))

        error = warn

    class Node:
        overrides = {}

        def __init__(self, name):
            self._params, self.pubs, self.subs, self.timers = {}, {}, {}, []
            self._logger = Logger()

        def declare_parameter(self, n, d):
            self._params[n] = Node.overrides.get(n, d)

        def get_parameter(self, n):
            return types.SimpleNamespace(value=self._params[n])

        def create_subscription(self, t, topic, cb, q):
            self.subs[topic] = cb

        def create_publisher(self, t, topic, q):
            self.pubs[topic] = Pub()
            return self.pubs[topic]

        def create_timer(self, p, cb):
            self.timers.append((p, cb))

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 0))

        def destroy_node(self):
            pass

    mods = {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
    }
    mods["rclpy.node"].Node = Node
    mods["geometry_msgs.msg"].Twist = Twist
    mods["geometry_msgs.msg"].TwistStamped = TwistStamped
    mods["sensor_msgs.msg"].JointState = Msg
    mods["std_msgs.msg"].Bool = Msg
    mods["std_msgs.msg"].String = Msg
    return mods, Node, TwistStamped


class NodeTests(unittest.TestCase):
    def setUp(self):
        self.mods, self.Node, self.TS = install_ros_stubs()
        self.patch = mock.patch.dict(sys.modules, self.mods)
        self.patch.start()
        sys.modules.pop("arc_hw_bridge.bridge", None)
        self.B = importlib.import_module("arc_hw_bridge.bridge")
        self.tmp = tempfile.TemporaryDirectory()
        self.c = Clock()
        self.tp = mock.patch.multiple(self.B.time, time=self.c.time)
        self.tp.start()

    def tearDown(self):
        self.tp.stop()
        self.patch.stop()
        sys.modules.pop("arc_hw_bridge.bridge", None)
        self.tmp.cleanup()

    def node(self, sketch, **params):
        base = dict(log_dir=self.tmp.name, pulses_per_rev=8, pwm_start_left=0.0, pwm_min_left=20.0, ff_left=0.25,
                    pwm_start_right=0.0, pwm_min_right=20.0, ff_right=0.25)
        base.update(params)
        self.Node.overrides = base
        n = self.B.MotorBridge()
        n.link.opener = lambda p, b: sketch
        n.link.clock, n.link.sleep = self.c.time, self.c.sleep
        return n

    def run_ticks(self, n, seconds, v=None, w=0.0):
        for _ in range(int(seconds * 20)):
            if v is not None:
                n.subs["/cmd_vel"](self.TS(v, w))
            n.tick()
            self.c.sleep(0.05)

    def test_cmd_vel_drives_wheels_and_measures_them(self):
        s = FakeSketch(self.c)
        n = self.node(s)
        self.run_ticks(n, 6.0, v=0.10)                       # 100 mm/s each wheel
        self.assertIn("M,45,45\n", s.written)                # 20 + 0.25*100
        meas = n.pubs["/arc_hw/wheels"].msgs
        self.assertGreaterEqual(len(meas), 3)
        # sim: (45-20)*4 = 100 mm/s = 2.5 pulses/s, so single reports read 78.5 or 117.8 mm/s (1 pulse = 39 mm).
        later = [m.velocity[0] for m in meas[2:]]            # the first two cover the 2 s Uno reset wait
        self.assertAlmostEqual(sum(later) / len(later), 0.1, delta=0.025)
        self.assertTrue(all(abs(x - 0.1) <= 0.04 for x in later))
        self.assertTrue(n.pubs["/arc_hw/connected"].msgs[-1].data)
        self.assertTrue(n.pubs["/arc_hw/estop_ok"].msgs[-1].data)
        self.run_ticks(n, 1.0)                               # /cmd_vel goes quiet -> 0 after cmd_timeout
        self.assertEqual(s.written[-1], "M,0,0\n")
        n.close()
        self.assertEqual(s.written[-1], "STOP\n")
        rows = open(n.f.name).read().splitlines()
        self.assertTrue(rows[0].startswith("t,bt_mode,cmd_v,cmd_w,target_l_mm_s,target_r_mm_s,meas_l_mm_s"))
        self.assertGreaterEqual(len(rows), 4)

    def test_turn_in_place_signs_and_invert(self):
        s = FakeSketch(self.c)
        n = self.node(s, invert_right=True)
        self.run_ticks(n, 3.5, v=0.0, w=0.5)                 # left -100, right +100 mm/s
        self.assertIn("M,-45,-45\n", s.written)             # right flipped on the wire
        last = n.pubs["/arc_hw/wheels"].msgs[-1]
        self.assertLess(last.velocity[0], 0)                 # measured sign follows the logical command
        self.assertGreater(last.velocity[1], 0)

    def test_estop_and_noise_warnings(self):
        s = FakeSketch(self.c, estop_closed=False, noise_right=623)
        n = self.node(s)
        self.run_ticks(n, 4.0, v=0.10)
        n.status()
        warns = " ".join(t for k, t in n.get_logger().lines if k == "warn")
        self.assertIn("E-STOP OPEN", warns)
        self.assertIn("right encoder", warns)
        self.assertFalse(n.pubs["/arc_hw/estop_ok"].msgs[-1].data)
        self.assertEqual(s.pwm, [0, 0])

    def test_rate_below_watchdog_refused(self):
        with self.assertRaises(ValueError):
            self.node(FakeSketch(self.c), rate=3.0)


class BenchTests(unittest.TestCase):
    def setUp(self):
        self.c = Clock()
        self.tmp = tempfile.TemporaryDirectory()
        self.patches = [mock.patch("time.time", self.c.time), mock.patch("time.sleep", self.c.sleep),
                        mock.patch.dict(os.environ, {"HOME": self.tmp.name})]
        for p in self.patches:
            p.start()
        from arc_hw_bridge import bench
        self.bench = bench

    def tearDown(self):
        for p in reversed(self.patches):
            p.stop()
        self.tmp.cleanup()

    def run_bench(self, args, sketch):
        with mock.patch.object(self.bench.Link, "_open", lambda self_: sketch), \
             mock.patch("sys.stdout", new_callable=__import__("io").StringIO) as out:
            try:
                self.bench.main(args)
                code = 0
            except SystemExit as e:
                code = e.code
        return code, out.getvalue()

    def test_check_pass_and_fail(self):
        code, out = self.run_bench(["check"], FakeSketch(self.c))
        self.assertEqual(code, 0, out)
        self.assertIn("CHECK PASS", out)
        code, out = self.run_bench(["check"], FakeSketch(self.c, estop_closed=False, noise_right=623))
        self.assertEqual(code, 1)
        self.assertIn("ERR,ESTOP", out)
        self.assertIn("right encoder counts", out)

    def test_watchdog_pass(self):
        code, out = self.run_bench(["watchdog", "--pwm", "40"], FakeSketch(self.c))
        self.assertEqual(code, 0, out)
        self.assertIn("PASS", out)

    def test_calibrate_recovers_model(self):
        code, out = self.run_bench(["calibrate"], FakeSketch(self.c))
        self.assertEqual(code, 0, out)
        val = {ln.split(":")[0].strip(): float(ln.split(":")[1]) for ln in out.splitlines()
               if ln.strip().startswith(("pwm_start_", "pwm_min_", "ff_"))}
        pmin, ff = val["pwm_min_left"], val["ff_left"]
        self.assertEqual(val["pwm_start_left"], 25.0)        # first moving step of the sweep (sim moves above 20)
        self.assertAlmostEqual(pmin, 20.0, delta=3.0)        # sim truth: 20 and 0.25 (1 s pulse rounding)
        self.assertAlmostEqual(ff, 0.25, delta=0.03)
        self.assertTrue(os.listdir(os.path.join(self.tmp.name, "arc_hw_logs", "bench")))


if __name__ == "__main__":
    unittest.main()
