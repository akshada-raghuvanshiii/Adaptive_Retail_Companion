import ast
import sys
import unittest
from math import pi
from pathlib import Path

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))
sys.path.insert(0, str(KIT / "arc_planner_addon"))

from apply_patches import patch_manager, patch_planner  # noqa: E402
from goal_frame_guard import apply_2d, goal_cell_status, resolve_goal, yaw_from_quaternion  # noqa: E402

PLANNER = '''import rclpy
from rclpy.node import Node


class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_planner")
        self.goal_x = None

    def shopping_goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info("goal received")
        self.plan()
'''

MANAGER = '''class ShoppingManager:
    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x = x
'''


def no_tf(target, source):
    raise LookupError("no transform")


class GuardTests(unittest.TestCase):
    def test_map_goal_passes(self):
        r = resolve_goal("map", 1.0, 2.0, 0.5, "map", no_tf)
        self.assertTrue(r.ok)
        self.assertEqual((r.x, r.y, r.yaw), (1.0, 2.0, 0.5))

    def test_world_goal_without_tf_is_rejected_not_guessed(self):
        r = resolve_goal("world", 1.9, 7.2, 0.0, "map", no_tf)
        self.assertFalse(r.ok)
        self.assertIn("no TF", r.reason)

    def test_empty_frame_rejected(self):
        self.assertFalse(resolve_goal("", 1, 1, 0, "map", no_tf).ok)

    def test_goal_transformed_through_lookup(self):
        r = resolve_goal("odom", 1.0, 0.0, 0.0, "map", lambda t, s: (2.0, 3.0, pi / 2))
        self.assertTrue(r.ok)
        self.assertAlmostEqual(r.x, 2.0)
        self.assertAlmostEqual(r.y, 4.0)
        self.assertAlmostEqual(r.yaw, pi / 2)

    def test_quaternion_yaw(self):
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, 1, 0), pi)
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, 0.7071068, 0.7071068), pi / 2, places=5)

    def test_cell_status_with_the_reported_map(self):
        # 203 x 179 cells at 0.05 m, origin (-4.2251, -4.0208) as reported
        w, h = 203, 179
        data = [0] * (w * h)
        data[(int((0 + 4.0208) / 0.05)) * w + int((1 + 4.2251) / 0.05)] = 100
        self.assertEqual(goal_cell_status(-4.2251, -4.0208, 0.05, w, h, data, 1.9, 7.2)[0], "outside")
        self.assertEqual(goal_cell_status(-4.2251, -4.0208, 0.05, w, h, data, 1.0, 0.0)[0], "occupied")
        data[0] = -1
        self.assertEqual(goal_cell_status(-4.2251, -4.0208, 0.05, w, h, data, -4.2, -4.0)[0], "unknown")
        self.assertEqual(goal_cell_status(-4.2251, -4.0208, 0.05, w, h, data, 0.0, 0.0)[0], "ok")


class PatchTests(unittest.TestCase):
    def test_planner_patch_is_minimal_and_valid_python(self):
        out, msg = patch_planner(PLANNER)
        self.assertIn("patched", msg)
        ast.parse(out)
        self.assertIn("resolve_goal_for_node(self, msg)", out)
        self.assertIn("self.plan()", out)                       # rest of callback untouched
        self.assertNotIn("self.goal_x = msg.pose.position.x", out)
        out2, msg2 = patch_planner(out)
        self.assertEqual(out2, out)
        self.assertEqual(msg2, "already patched")

    def test_planner_patch_skips_when_anchor_differs(self):
        src = PLANNER.replace("self.goal_y = msg.pose.position.y", "self.goal_y = float(msg.pose.position.y)")
        out, msg = patch_planner(src)
        self.assertEqual(out, src)
        self.assertIn("SKIPPED", msg)

    def test_manager_frame_literal(self):
        out, msg = patch_manager(MANAGER)
        self.assertIn("goal.header.frame_id = 'map'", out)
        self.assertIn("1", msg)
        out2, msg2 = patch_manager("frame = cfg['frame_id']\n")
        self.assertIn("unchanged", msg2)


if __name__ == "__main__":
    unittest.main()
