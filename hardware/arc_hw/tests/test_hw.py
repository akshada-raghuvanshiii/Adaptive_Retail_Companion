"""Offline tests for arc_hw (no Arduino, no ROS):  python3 -m unittest discover -s tests -v"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arc_hw import protocol as P  # noqa: E402


class Kinematics(unittest.TestCase):
    def test_straight_and_spin(self):
        self.assertEqual(P.twist_to_wheels(0.1, 0.0, 0.4), (100.0, 100.0))
        l, r = P.twist_to_wheels(0.0, 0.5, 0.4)          # turn left in place
        self.assertAlmostEqual(l, -100.0)
        self.assertAlmostEqual(r, 100.0)

    def test_round_trip(self):
        for v, w in ((0.1, 0.3), (-0.05, -0.2), (0.15, 0.0)):
            back = P.wheels_to_twist(*P.twist_to_wheels(v, w, 0.37), 0.37)
            self.assertAlmostEqual(back[0], v, places=9)
            self.assertAlmostEqual(back[1], w, places=9)

    def test_clamp_keeps_ratio(self):
        l, r = P.clamp_wheels(200, 400, 300)
        self.assertAlmostEqual(r, 300)
        self.assertAlmostEqual(l / r, 0.5)
        self.assertEqual(P.clamp_wheels(50, -60, 300), (50, -60))

    def test_mm_per_pulse(self):
        self.assertAlmostEqual(P.Geometry(0.10, 0.4, 8).mm_per_pulse, math.pi * 100 / 8)


class Protocol(unittest.TestCase):
    def test_commands(self):
        self.assertEqual(P.cmd_speed(100, -50.25), "V 100.0 -50.2\n")
        self.assertEqual(P.cmd_pwm(80.7, -3), "P 80 -3\n")
        self.assertTrue(P.cmd_gains(60, 0.6, 0).startswith("K 60.0 0.6000"))

    def test_parse(self):
        r = P.parse_report("E 1234 10 -3 98.5 -40.0 120 -90\r\n")
        self.assertEqual((r.ms, r.count_l, r.count_r, r.pwm_r), (1234, 10, -3, -90))
        self.assertAlmostEqual(r.speed_l, 98.5)
        for bad in ("ARC motor bridge ready", "E 1 2 3", "E a b c d e f g", ""):
            self.assertIsNone(P.parse_report(bad))

    def test_fit(self):
        pts = [(0, 0), (20, 0), (40, 0), (60, 10), (80, 43.3), (100, 76.7), (120, 110)]
        pm, ff = P.fit_feedforward(pts)
        self.assertAlmostEqual(ff, 0.6, delta=0.02)
        self.assertAlmostEqual(pm, 54, delta=2.5)
        self.assertIsNone(P.fit_feedforward([(0, 0), (50, 0)]))


class FirmwareConsistency(unittest.TestCase):
    """The PC side and the sketch must agree on the command letters and reply format."""

    def test_sketch_handles_every_command(self):
        ino = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "firmware", "arc_motor_bridge", "arc_motor_bridge.ino")).read()
        for letter in "VPKMS":
            self.assertIn(f"'{letter}'", ino)
        self.assertIn('Serial.print("E ")', ino)
        self.assertIn("WATCHDOG_MS = 500", ino)


if __name__ == "__main__":
    unittest.main()
