#!/usr/bin/env python3
"""Give ARC's shopping_manager.py a route option: Dijkstra-optimised order or the typed order.

    python3 apply_route_option.py            # dry run: show what would change, write nothing
    python3 apply_route_option.py --write    # back up, patch, copy route_order.py

After you confirm the list, the shopping manager prices two routes on the live map, from where the
robot is now, using the same Dijkstra costs the planner drives on:

    [1] Optimised (exact):  9.84 m   tea_coffee -> fruits -> pasta
    [2] As typed:          15.30 m   pasta -> tea_coffee -> fruits
    The optimised route is 5.46 m (36%) shorter.
    Choose route: 1 = optimised, 2 = as typed [1]:

Parameter route_ordering:
    ask        (default) show both routes and let the shopper choose
    exact      always use the optimised order (Held-Karp; nearest-next above 12 stops)
    nearest    always use nearest-next by path cost
    as_entered always use the typed order (the original behaviour)

If the old route-order patch (apply_route_order.py) is already in the file, this upgrades it in
place: it only adds the choice. If it is not, it adds the full route ordering plus the choice.
Untouched: the Dijkstra planner, the TEB-like controller, the BT, TF, RTAB-Map, arc_person_follower.
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
MARK_V1 = "ARC route ordering (route_order.py)"
MARK_V2 = "ARC route ordering option (ask)"

# ---------------------------------------------------------------- base route ordering (as v1)
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
ORDER_CALL_V1 = "        self.navigation_goals = self._ro_order(self.navigation_goals, wait_s=10.0)\n"
ORDER_ADD = """        # --- ARC route ordering (route_order.py): optional checkout as the last stop,
        # then order by Dijkstra path cost (the checkout stays last)
        self.navigation_goals = self._ro_with_checkout(self.navigation_goals)
""" + ORDER_CALL_V1 + "\n"

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

# ---------------------------------------------------------------- the option (v2)
IMPORT_V1_LINE = ("from arc_planner.route_order import CostGrid as _RoCostGrid, "
                  "plan_route as _ro_plan_route\n")
IMPORT_V2_LINE = ("from arc_planner.route_order import CostGrid as _RoCostGrid, "
                  "plan_route as _ro_plan_route, compare_routes as _ro_compare_routes\n")
PARAM_V1 = "self.declare_parameter('route_ordering', 'exact')"
PARAM_V2 = "self.declare_parameter('route_ordering', 'ask')"
ORDER_CALL_V2 = ("        # --- ARC route ordering option (ask): show the optimised and the typed route, let the\n"
                 "        # shopper choose (route_ordering ask | exact | nearest | as_entered)\n"
                 "        self.navigation_goals = self._ro_choose(self.navigation_goals, wait_s=10.0)\n")
CHOOSE_ANCHOR = "    def _ro_order(self, goals, wait_s=10.0, quiet=False):\n"
CHOOSE_ADD = '''    def _ro_choose(self, goals, wait_s=10.0):
        """ARC route ordering option (ask): price the optimised and the typed route on the live map,
        from the robot's pose, and let the shopper choose. Any other route_ordering value goes
        straight to _ro_order. The choice also decides whether later stops are re-ordered."""
        if self.route_ordering != 'ask':
            return self._ro_order(goals, wait_s=wait_s)
        tail = [goals[-1]] if goals and goals[-1].get('is_checkout') else []
        body = list(goals[:-1]) if tail else list(goals)
        if len(body) < 2:
            self.route_ordering = 'as_entered'
            return list(goals)
        reason = None
        start = None
        if self.item_frame != 'map':
            reason = f'items are in frame "{self.item_frame}", not "map"'
        else:
            end = _ro_time.time() + wait_s
            while (self._ro_map is None or self._ro_robot_xy() is None) and _ro_time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            start = self._ro_robot_xy()
            if self._ro_map is None or start is None:
                reason = 'no /rtabmap/map or no map -> base_footprint yet'
        if reason:
            self.get_logger().warn(f'Route options unavailable ({reason}). Keeping the typed order.')
            self.route_ordering = 'as_entered'
            return list(goals)
        if self._ro_grid is None:
            self._ro_grid = _RoCostGrid.from_msg(self._ro_map)
        end_xy = (tail[0]['x'], tail[0]['y']) if tail else None
        t0 = _ro_time.time()
        cmp_ = _ro_compare_routes(self._ro_grid, start, body, 'exact', end_xy=end_xy)
        ms = (_ro_time.time() - t0) * 1000.0
        (opt, oi), (typ, ti) = cmp_['optimised'], cmp_['typed']

        def names(stops):
            return ' -> '.join([s['item'] for s in stops] + (['Checkout'] if tail else []))

        print('\\n========== ROUTE OPTIONS (Dijkstra path cost from the robot) ==========')
        l1, l2 = f"[1] Optimised ({oi['strategy']}):", '[2] As typed:'
        w = max(len(l1), len(l2))
        print(f"  {l1.ljust(w)} {oi['cost']:7.2f} m   {names(opt)}")
        print(f"  {l2.ljust(w)} {ti['cost']:7.2f} m   {names(typ)}")
        if cmp_['saved_m'] > 0.005:
            print(f"  The optimised route is {cmp_['saved_m']:.2f} m ({cmp_['saved_pct']:.0f}%) shorter.")
        else:
            print('  The typed order is already the shortest.')
        if oi['merged_items']:
            print(f"  {oi['merged_items']} item(s) share a spot with another item and are collected together.")
        if oi['unreachable']:
            print(f"  Not reachable on the map (visited last): {', '.join(oi['unreachable'])}")
        print(f'  (computed in {ms:.0f} ms)')
        print('=' * 72)
        while True:
            ans = input('\\nChoose route: 1 = optimised, 2 = as typed [1]: ').strip().lower()
            if ans in ('', '1', 'o', 'optimised', 'optimized', 'y', 'yes'):
                self.route_ordering, chosen, label, cost = 'exact', opt, 'optimised', oi['cost']
                break
            if ans in ('2', 't', 'typed', 'n', 'no'):
                self.route_ordering, chosen, label, cost = 'as_entered', typ, 'as typed', ti['cost']
                break
            print('Please enter 1 or 2.')
        self.get_logger().info(
            f"Route chosen: {label}, {cost:.2f} m (optimised {oi['cost']:.2f} m, typed {ti['cost']:.2f} m)")
        return list(chosen) + tail

'''


def _once(src, anchor, name):
    n = src.count(anchor)
    if n != 1:
        raise SystemExit(f"anchor '{name}' found {n} times (need exactly 1) - the file differs from the "
                         "version I inspected; nothing written. Send me the file.")


def base_patch(src):
    """The full route ordering, as the earlier apply_route_order.py adds it."""
    for name, anchor in (("imports", IMPORTS_ANCHOR), ("init", INIT_ANCHOR),
                         ("order", ORDER_ANCHOR), ("next stop", NEXT_ANCHOR)):
        _once(src, anchor, name)
    if len(METHODS_RE.findall(src)) != 1:
        raise SystemExit("anchor 'MAIN block' not found exactly once - nothing written.")
    out = src.replace(IMPORTS_ANCHOR, IMPORTS_ANCHOR + IMPORTS_ADD, 1)
    out = out.replace(INIT_ANCHOR, INIT_ADD + INIT_ANCHOR, 1)
    out = out.replace(ORDER_ANCHOR, ORDER_ADD + ORDER_ANCHOR, 1)
    out = out.replace(NEXT_ANCHOR, NEXT_NEW, 1)
    m = METHODS_RE.search(out)
    return out[:m.start()] + "\n" + METHODS_ADD + out[m.start():]


def option_patch(src):
    """Add the choice to a file that already has the route ordering."""
    for name, anchor in (("route_order import", IMPORT_V1_LINE), ("route_ordering parameter", PARAM_V1),
                         ("ordering call", ORDER_CALL_V1), ("_ro_order method", CHOOSE_ANCHOR)):
        _once(src, anchor, name)
    out = src.replace(IMPORT_V1_LINE, IMPORT_V2_LINE, 1)
    out = out.replace(PARAM_V1, PARAM_V2, 1)
    out = out.replace(ORDER_CALL_V1, ORDER_CALL_V2, 1)
    return out.replace(CHOOSE_ANCHOR, CHOOSE_ADD + CHOOSE_ANCHOR, 1)


def patch(src):
    if MARK_V2 in src:
        raise SystemExit("shopping_manager.py already has the route option - nothing to do.")
    if MARK_V1 in src:
        return option_patch(src), "upgrade (route ordering already present: adds the choice)"
    return option_patch(base_patch(src)), "fresh (adds route ordering and the choice)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pkg", default=PKG, help="folder that holds shopping_manager.py")
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    target = os.path.join(a.pkg, "shopping_manager.py")
    src = open(target).read()
    out, kind = patch(src)
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as t:
        t.write(out)
    py_compile.compile(t.name, doraise=True)
    os.unlink(t.name)
    added = len(out.splitlines()) - len(src.splitlines())
    print(f"{target}: {kind}; applies cleanly (+{added} lines), compiles.")
    if not a.write:
        print("DRY RUN - nothing written. Re-run with --write.")
        return
    bk_dir = os.path.expanduser("~/slam_ws/arc_backups")
    os.makedirs(bk_dir, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    bk = os.path.join(bk_dir, f"shopping_manager.before_route_option_{stamp}.py")
    shutil.copy2(target, bk)
    ro_dst = os.path.join(a.pkg, "route_order.py")
    ro_bk = None
    if os.path.exists(ro_dst):
        ro_bk = os.path.join(bk_dir, f"route_order.before_route_option_{stamp}.py")
        shutil.copy2(ro_dst, ro_bk)
    with open(target, "w") as f:
        f.write(out)
    shutil.copy2(os.path.join(HERE, "arc_planner", "arc_planner", "route_order.py"), ro_dst)
    print(f"backup: {bk}" + (f" and {ro_bk}" if ro_bk else ""))
    print(f"wrote {target} and {ro_dst}")
    print("Next: cd ~/slam_ws && colcon build --packages-select arc_planner && source install/setup.bash")
    undo_ro = f"cp {ro_bk} {ro_dst}" if ro_bk else f"rm {ro_dst}"
    print(f"Undo: cp {bk} {target} && {undo_ro} && cd ~/slam_ws && colcon build --packages-select arc_planner")


if __name__ == "__main__":
    sys.exit(main())
