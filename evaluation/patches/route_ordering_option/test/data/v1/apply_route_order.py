#!/usr/bin/env python3
"""Add route ordering by Dijkstra path cost to ARC's shopping_manager.py.

    python3 apply_route_order.py            # dry run: show what would change
    python3 apply_route_order.py --write    # back up, patch, copy route_order.py

What it changes (nothing else is touched):
  1. copies route_order.py next to shopping_manager.py (new file);
  2. shopping_manager.py: 4 insertions at exact anchors.
     - imports;
     - after the subscriptions, a map subscription, a TF listener and two parameters:
       route_ordering (exact | nearest | as_entered, default exact) and
       route_reorder_each_stop (default true);
     - after the goals are created, they are re-ordered from the robot's pose;
     - when a stop is reached, the stops still to come are re-ordered from where
       the robot now is (answers the plan's open question: costs are recomputed
       from the current pose).
  Optional: route_end_at_checkout:=true (default false) adds the checkout counter as the
  last stop and optimises the order with it as the fixed end. Position from checkout_x /
  checkout_y / checkout_yaw, or a "checkout" entry in the item file.
  If the map or the robot pose is not available within 10 s, or the item frame is
  not 'map', it keeps the entered order and says so. Old behaviour on request:
      ros2 run arc_planner shopping_manager --ros-args -p route_ordering:=as_entered

Backup: ~/slam_ws/arc_backups/shopping_manager.before_route_order_<time>.py
Every anchor must match exactly once, or nothing is written.
"""
import argparse
import os
import py_compile
import re
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.expanduser("~/slam_ws/src/arc_planner/arc_planner")
MARK = "ARC route ordering (route_order.py)"

IMPORTS_ANCHOR = "from std_msgs.msg import String, Bool\n"
IMPORTS_ADD = """
# --- ARC route ordering (route_order.py): imports
import math as _ro_math
import time as _ro_time
import tf2_ros as _ro_tf2
from nav_msgs.msg import OccupancyGrid as _RoOccupancyGrid
from rclpy.time import Time as _RoTime
from arc_planner.route_order import CostGrid as _RoCostGrid, plan_route as _ro_plan_route
"""

INIT_ANCHOR = """        # ============================================================
        # LOAD GENERATED STORE ITEM LOCATIONS
        # ============================================================
"""
INIT_ADD = """        # --- ARC route ordering (route_order.py): map, robot pose, parameters
        self.route_ordering = str(self.declare_parameter('route_ordering', 'exact').value)
        self.route_reorder_each_stop = bool(self.declare_parameter('route_reorder_each_stop', True).value)
        # optional final destination after the last item (off by default = old behaviour)
        self.route_end_at_checkout = bool(self.declare_parameter('route_end_at_checkout', False).value)
        self.checkout_x = float(self.declare_parameter('checkout_x', float('nan')).value)
        self.checkout_y = float(self.declare_parameter('checkout_y', float('nan')).value)
        self.checkout_yaw = float(self.declare_parameter('checkout_yaw', 0.0).value)
        self._ro_map = None
        self._ro_grid = None
        self.create_subscription(
            _RoOccupancyGrid, '/rtabmap/map', self._ro_on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._ro_tf_buffer = _ro_tf2.Buffer()
        self._ro_tf_listener = _ro_tf2.TransformListener(self._ro_tf_buffer, self)

"""

ORDER_ANCHOR = """        # ============================================================
        # DISPLAY NAVIGATION SEQUENCE
        # ============================================================
"""
ORDER_ADD = """        # --- ARC route ordering (route_order.py): optional checkout as the last stop,
        # then order by Dijkstra path cost (the checkout stays last)
        self.navigation_goals = self._ro_with_checkout(self.navigation_goals)
        self.navigation_goals = self._ro_order(self.navigation_goals, wait_s=10.0)

"""

NEXT_ANCHOR = """        current_goal = self.navigation_goals[
            self.current_goal_index
        ]
"""
NEXT_NEW = NEXT_ANCHOR + """
        # --- ARC route ordering (route_order.py): the robot is at this stop now, so
        # re-order the stops still to come from here (before "Next item" is shown)
        if self.route_reorder_each_stop:
            k = self.current_goal_index + 1
            if len([g for g in self.navigation_goals[k:] if not g.get('is_checkout')]) > 1:
                self.navigation_goals = (self.navigation_goals[:k]
                                         + self._ro_order(self.navigation_goals[k:], wait_s=0.0, quiet=True))
"""

METHODS_RE = re.compile(r"\n\n# =+\n# MAIN\n# =+\n")
METHODS_ADD = '''
    # ================================================================
    # ARC route ordering (route_order.py)
    # ================================================================

    def _ro_on_map(self, msg):
        self._ro_map = msg
        self._ro_grid = None          # rebuilt on next use

    def _ro_robot_xy(self):
        try:
            tf = self._ro_tf_buffer.lookup_transform('map', 'base_footprint', _RoTime())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _ro_with_checkout(self, goals):
        """Append the checkout counter as the final goal when route_end_at_checkout is set."""
        if not self.route_end_at_checkout or not goals:
            return goals
        x, y, yaw = self.checkout_x, self.checkout_y, self.checkout_yaw
        if _ro_math.isnan(x) or _ro_math.isnan(y):
            loc = self.items.get('checkout') or self.items.get('Checkout') or {}
            if 'x' in loc and 'y' in loc:
                x, y, yaw = float(loc['x']), float(loc['y']), float(loc.get('yaw', yaw))
        if _ro_math.isnan(x) or _ro_math.isnan(y):
            self.get_logger().warn('route_end_at_checkout is set but no checkout position: set '
                                   'checkout_x/checkout_y or add "checkout" to the item file. Not added.')
            return goals
        return list(goals) + [{'item': 'Checkout', 'shelf_id': 'checkout', 'shelf_name': 'checkout counter',
                               'x': x, 'y': y, 'yaw': yaw, 'is_checkout': True}]

    def _ro_order(self, goals, wait_s=10.0, quiet=False):
        """Order goals by Dijkstra path cost from the robot; falls back to the given order.
        A trailing checkout goal stays last, and the order is optimised with it as the fixed end."""
        tail = [goals[-1]] if goals and goals[-1].get('is_checkout') else []
        if tail:
            goals = goals[:-1]
        if self.route_ordering == 'as_entered' or len(goals) < 2:
            return list(goals) + tail
        if self.item_frame != 'map':
            self.get_logger().warn(
                f'Route ordering skipped: items are in frame "{self.item_frame}", not "map".')
            return list(goals) + tail
        end = _ro_time.time() + wait_s
        while (self._ro_map is None or self._ro_robot_xy() is None) and _ro_time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        start = self._ro_robot_xy()
        if self._ro_map is None or start is None:
            self.get_logger().warn(
                'Route ordering skipped: no /rtabmap/map or no map -> base_footprint yet. '
                'Keeping the entered order.')
            return list(goals) + tail
        if self._ro_grid is None:
            self._ro_grid = _RoCostGrid.from_msg(self._ro_map)
        t0 = _ro_time.time()
        end_xy = (tail[0]['x'], tail[0]['y']) if tail else None
        ordered, info = _ro_plan_route(self._ro_grid, start, goals, self.route_ordering, end_xy=end_xy)
        ms = (_ro_time.time() - t0) * 1000.0
        if not quiet:
            saved = info['cost_as_entered'] - info['cost']
            pct = 100.0 * saved / info['cost_as_entered'] if info['cost_as_entered'] > 0 else 0.0
            print('\\n========== ROUTE ORDER (Dijkstra path cost) ==========')
            print(f"  strategy {info['strategy']}, {info['stops']} stops "
                  f"({info['merged_items']} items share a spot), computed in {ms:.0f} ms")
            print(f"  route {info['cost']:.2f} m vs {info['cost_as_entered']:.2f} m as entered "
                  f"({pct:.0f}% shorter)")
            for i, g in enumerate(ordered, start=1):
                print(f"  {i}. {g['item']}  ({g['x']:.2f}, {g['y']:.2f})")
            if info['unreachable']:
                print(f"  not reachable on the map (kept last): {', '.join(info['unreachable'])}")
            if tail:
                print(f"  {len(ordered) + 1}. Checkout  ({tail[0]['x']:.2f}, {tail[0]['y']:.2f})  (final destination)")
            print('======================================================')
        self.get_logger().info(
            f"Route order: {info['strategy']}, {info['cost']:.2f} m "
            f"(as entered {info['cost_as_entered']:.2f} m), {ms:.0f} ms")
        return ordered + tail
'''


def patch(src):
    if MARK in src:
        raise SystemExit("shopping_manager.py already has route ordering - nothing to do.")
    for name, anchor in (("imports", IMPORTS_ANCHOR), ("init", INIT_ANCHOR),
                         ("order", ORDER_ANCHOR), ("next stop", NEXT_ANCHOR)):
        n = src.count(anchor)
        if n != 1:
            raise SystemExit(f"anchor '{name}' found {n} times (need exactly 1) - file differs from the "
                             "version I inspected; nothing written. Send me the file.")
    if len(METHODS_RE.findall(src)) != 1:
        raise SystemExit("anchor 'MAIN block' not found exactly once - nothing written.")
    out = src.replace(IMPORTS_ANCHOR, IMPORTS_ANCHOR + IMPORTS_ADD, 1)
    out = out.replace(INIT_ANCHOR, INIT_ADD + INIT_ANCHOR, 1)
    out = out.replace(ORDER_ANCHOR, ORDER_ADD + ORDER_ANCHOR, 1)
    out = out.replace(NEXT_ANCHOR, NEXT_NEW, 1)
    m = METHODS_RE.search(out)
    out = out[:m.start()] + "\n" + METHODS_ADD + out[m.start():]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pkg", default=PKG, help="folder that holds shopping_manager.py")
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    target = os.path.join(a.pkg, "shopping_manager.py")
    src = open(target).read()
    out = patch(src)
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as t:
        t.write(out)
    py_compile.compile(t.name, doraise=True)
    os.unlink(t.name)
    added = len(out.splitlines()) - len(src.splitlines())
    print(f"{target}: patch applies cleanly (+{added} lines), compiles.")
    if not a.write:
        print("DRY RUN - nothing written. Re-run with --write.")
        return
    bk_dir = os.path.expanduser("~/slam_ws/arc_backups")
    os.makedirs(bk_dir, exist_ok=True)
    bk = os.path.join(bk_dir, f"shopping_manager.before_route_order_{time.strftime('%Y%m%d_%H%M%S')}.py")
    shutil.copy2(target, bk)
    with open(target, "w") as f:
        f.write(out)
    shutil.copy2(os.path.join(HERE, "arc_planner", "arc_planner", "route_order.py"),
                 os.path.join(a.pkg, "route_order.py"))
    print(f"backup: {bk}")
    print(f"wrote {target} and {os.path.join(a.pkg, 'route_order.py')}")
    print("Next: cd ~/slam_ws && colcon build --packages-select arc_planner && source install/setup.bash")
    print(f"Undo: cp {bk} {target} && rm {os.path.join(a.pkg, 'route_order.py')} && "
          "cd ~/slam_ws && colcon build --packages-select arc_planner")


if __name__ == "__main__":
    sys.exit(main())
