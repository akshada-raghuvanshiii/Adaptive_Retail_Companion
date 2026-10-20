import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros_tools"))
import layout as L  # noqa: E402
from auto_map import ROUTE, RouteFollower, check_route  # noqa: E402

BURGER_R = 0.105


def box_dist(x, y, b):
    return math.hypot(x - min(max(x, b.x0), b.x1), y - min(max(y, b.y0), b.y1))


class AutoMapTests(unittest.TestCase):
    def test_every_leg_clears_fixtures(self):
        for label, p, q, d, name in check_route():
            self.assertGreater(d, BURGER_R + 0.25, f"{label}: {d:.2f} m from {name}")

    def test_simulated_drive_reaches_every_waypoint_without_contact(self):
        """Unicycle simulation with odometry == truth: the controller must reach all waypoints
        and the robot body must never come within 0.15 m of any fixture."""
        obstacles = L.ALL_OBSTACLES + L.parked_obstacles()
        f = RouteFollower(ROUTE)
        x = y = yaw = 0.0
        t, dt, closest = 0.0, 0.05, float("inf")
        while not f.done and t < 1200:
            v, w = f.step(t, x, y, yaw, float("inf"))
            self.assertLessEqual(abs(w), 0.3 + 1e-9)
            self.assertLessEqual(v, 0.12 + 1e-9)
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += w * dt
            t += dt
            closest = min(closest, min(box_dist(x, y, b) for b in obstacles) - BURGER_R)
        self.assertTrue(f.done, f"stuck at waypoint {f.i} state {f.state}")
        self.assertEqual(sum(1 for e in f.events if e[1] == "reached"), len(ROUTE))
        self.assertGreater(closest, 0.15)
        self.assertLess(t, 600, "route should take under 10 simulated minutes")

    def test_blocked_backs_up_three_times_then_skips(self):
        f = RouteFollower([(1.0, 0.0, "a"), (2.0, 0.0, "b")])
        t, vs = 0.0, []
        for _ in range(1200):
            v, _ = f.step(t, 0.0, 0.0, 0.0, 0.1)    # obstacle 0.1 m ahead, robot never moves
            self.assertLessEqual(v, 0.0)            # never drives forward into it
            vs.append(v)
            t += 0.05
        kinds = [e[1] for e in f.events]
        self.assertEqual(sum(1 for e in f.events if e[1] == "backup" and e[2] == 0), 3)
        self.assertIn("skipped", kinds)
        self.assertTrue(any(v < 0 for v in vs))    # it reversed


if __name__ == "__main__":
    unittest.main()
