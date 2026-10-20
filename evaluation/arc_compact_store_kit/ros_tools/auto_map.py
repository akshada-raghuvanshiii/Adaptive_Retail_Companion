#!/usr/bin/env python3
"""Drive the robot autonomously along a fixed mapping route through the compact store,
so RTAB-Map can build the map without teleop.

How it drives (chosen to give RTAB-Map clean scans):
  * turn on the spot to face the next waypoint, slowly (<= 0.35 rad/s), then pause 1.5 s;
  * drive straight at 0.12 m/s with a small heading correction, then pause 1.0 s;
  * if the lidar sees anything closer than 0.25 m in front for 2 s, back up ~15 cm and re-aim
    (after 3 back-ups the waypoint is skipped).
Steering uses RTAB-Map's corrected pose (TF map -> base_footprint), the same pose the planner and TEB
use; the robot spawns at world (0, 0, yaw 0), so map == world. (--pose-source odom uses raw /odom.)

Diagnostics: at every waypoint it logs the map pose AND the /odom pose against Gazebo's TRUE pose
(`gz model -m <model> -p`). Small map error = RTAB-Map is correcting the wheel drift (good for
navigation); growing map error = it is not, and navigation will drift the same way.

Run with Gazebo, the robot bridge and RTAB-Map up, and WITHOUT the BT stack / motion gate or
teleop (they publish /cmd_vel too):
    python3 auto_map.py                      # whole route
    python3 auto_map.py --dry-run            # print the route and check its clearances, no ROS
Ctrl+C stops the robot.
"""

import argparse
import csv
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

# Route through the compact store (map == world frame). Every leg is checked against the
# fixtures in layout.py by --dry-run and by tests/test_auto_map.py.
ROUTE = [
    (0.00, -1.50, "south of start"),
    (1.80, -1.50, "south end of aisle 1-2"),
    (1.80, 2.70, "north end of aisle 1-2"),
    (3.475, 2.70, "north end of aisle 2-3"),
    (3.475, -1.50, "south end of aisle 2-3"),
    (0.00, -1.50, "back west along the south side"),
    (-2.50, -1.20, "in front of the checkout"),
    (-2.50, 0.40, "in front of the tea shelf"),
    (-2.50, 2.70, "in front of the fridge"),
    (-0.40, 2.70, "north of the produce island"),
    (0.00, 0.00, "home (spawn point)"),
]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RouteFollower:
    """Pure control logic (no ROS), so it can be tested with a simulated robot."""

    def __init__(self, route, v_max=0.12, w_max=0.25, pos_tol=0.08, yaw_tol=math.radians(4),
                 pause_turn=1.5, pause_point=1.0, stop_dist=0.25, blocked_wait=2.0,
                 backup_time=2.5, backup_speed=0.06, max_backups=3):
        self.route = list(route)
        self.v_max, self.w_max = v_max, w_max
        self.pos_tol, self.yaw_tol = pos_tol, yaw_tol
        self.pause_turn, self.pause_point = pause_turn, pause_point
        self.stop_dist, self.blocked_wait = stop_dist, blocked_wait
        self.backup_time, self.backup_speed, self.max_backups = backup_time, backup_speed, max_backups
        self.backups = 0
        self.i = 0
        self.state = "turn"          # turn -> settle -> drive -> arrived -> (next) turn ... -> done
        self.t_state = 0.0
        self.blocked_since = None
        self.events = []             # (time, kind, waypoint index)

    @property
    def done(self):
        return self.i >= len(self.route)

    def _go(self, state, t):
        self.state, self.t_state = state, t

    def _next(self, t):
        self.i += 1
        self.backups = 0
        self._go("turn", t)

    def step(self, t, x, y, yaw, front_min):
        """Return (v, w). front_min: closest lidar range in the forward sector (inf if none)."""
        if self.done:
            return 0.0, 0.0
        gx, gy, _ = self.route[self.i]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        heading_err = wrap(math.atan2(dy, dx) - yaw)

        if self.state == "turn":
            if dist < self.pos_tol:
                self._go("arrived", t)
                return 0.0, 0.0
            if abs(heading_err) < self.yaw_tol:
                self._go("settle", t)
                return 0.0, 0.0
            w = max(-self.w_max, min(self.w_max, 1.2 * heading_err))
            if abs(w) < 0.08:
                w = math.copysign(0.08, w)
            return 0.0, w
        if self.state == "settle":
            if t - self.t_state >= self.pause_turn:
                self._go("drive", t)
            return 0.0, 0.0
        if self.state == "drive":
            if dist < self.pos_tol:
                self._go("arrived", t)
                return 0.0, 0.0
            if abs(heading_err) > math.radians(25):     # drifted off: stop and re-aim
                self._go("turn", t)
                return 0.0, 0.0
            if front_min < self.stop_dist:
                if self.blocked_since is None:
                    self.blocked_since = t
                    self.events.append((t, "blocked", self.i))
                if t - self.blocked_since > self.blocked_wait:
                    self.blocked_since = None
                    self.backups += 1
                    if self.backups > self.max_backups:
                        self.events.append((t, "skipped", self.i))
                        self._next(t)
                    else:
                        self.events.append((t, "backup", self.i))
                        self._go("backup", t)
                return 0.0, 0.0
            self.blocked_since = None
            v = min(self.v_max, 0.6 * dist + 0.03)
            w = max(-0.3, min(0.3, 1.5 * heading_err))
            return v, w
        if self.state == "backup":                      # reverse ~15 cm, then re-aim
            if t - self.t_state >= self.backup_time:
                self._go("turn", t)
                return 0.0, 0.0
            return -self.backup_speed, 0.0
        if self.state == "arrived":
            if t - self.t_state >= self.pause_point:
                self.events.append((t, "reached", self.i))
                self._next(t)
            return 0.0, 0.0
        return 0.0, 0.0


# ---------------------------------------------------------------- route checks (no ROS)

def seg_box_distance(p, q, box, n=200):
    best = float("inf")
    for k in range(n + 1):
        s = k / n
        x, y = p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1])
        cx = min(max(x, box.x0), box.x1)
        cy = min(max(y, box.y0), box.y1)
        best = min(best, math.hypot(x - cx, y - cy))
    return best


def check_route(route=ROUTE, start=(0.0, 0.0)):
    import layout as L
    obstacles = L.ALL_OBSTACLES + L.parked_obstacles()
    rows, prev = [], start
    for x, y, label in route:
        d, name = min((seg_box_distance(prev, (x, y), b), b.name) for b in obstacles)
        rows.append((label, prev, (x, y), d, name))
        prev = (x, y)
    return rows


# ---------------------------------------------------------------- ROS node

def gz_truth(model):
    try:
        out = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    import re
    num = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
    tr = re.findall(rf"\[\s*({num})\s+({num})\s+({num})\s*\]", out.split("Pose", 1)[-1])
    if len(tr) < 2:
        return None
    return float(tr[0][0]), float(tr[0][1]), float(tr[1][2])


def run_ros(args):
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    def yaw_q(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    class AutoMap(Node):
        def __init__(self):
            super().__init__("arc_auto_map")
            self.odom = None
            self.front = float("inf")
            self.tf = Buffer()
            self.tfl = TransformListener(self.tf, self)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
            self.pub = self.create_publisher(Twist if args.unstamped else TwistStamped, "/cmd_vel", 10)

        def on_odom(self, m):
            self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation))

        def map_pose(self):
            try:
                t = self.tf.lookup_transform("map", "base_footprint", Time())
            except Exception:
                return None
            return t.transform.translation.x, t.transform.translation.y, yaw_q(t.transform.rotation)

        def pose(self):
            return self.map_pose() if args.pose_source == "map" else self.odom

        def on_scan(self, m):
            best = float("inf")
            for k, r in enumerate(m.ranges):
                a = wrap(m.angle_min + k * m.angle_increment)
                if abs(a) <= math.radians(25) and m.range_min < r < m.range_max and math.isfinite(r):
                    best = min(best, r)
            self.front = best

        def send(self, v, w):
            if args.unstamped:
                msg = Twist()
                msg.linear.x, msg.angular.z = float(v), float(w)
            else:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_footprint"
                msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
            self.pub.publish(msg)

    def fmt(p):
        return f"({p[0]:5.2f},{p[1]:5.2f},{math.degrees(p[2]):6.1f}°)" if p else "(   n/a          )"

    def err(p, truth):
        if not p or not truth:
            return "", ""
        return (f"{math.hypot(p[0] - truth[0], p[1] - truth[1]):.2f}",
                f"{abs(math.degrees(wrap(p[2] - truth[2]))):.1f}")

    rclpy.init()
    node = AutoMap()
    follower = RouteFollower(ROUTE)
    log_path = Path(args.log).expanduser()
    log = open(log_path, "w", newline="")
    wr = csv.writer(log)
    wr.writerow(["waypoint", "label", "event", "map_x", "map_y", "map_yaw_deg", "odom_x", "odom_y", "odom_yaw_deg",
                 "gz_x", "gz_y", "gz_yaw_deg", "map_err_m", "map_err_deg", "odom_err_m", "odom_err_deg"])
    print(f"Steering by: {'RTAB-Map map -> base_footprint' if args.pose_source == 'map' else '/odom'}"
          f"   (log: {log_path})")
    print("Waiting for /odom, /scan and the map -> base_footprint transform ...")
    t0 = time.time()
    while (node.odom is None or node.pose() is None) and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None:
        print("No /odom received in 20 s: is the robot bridge running?")
        return 2
    if node.pose() is None:
        print("No map -> base_footprint transform in 20 s: is RTAB-Map running (and receiving /odom + /scan)?")
        return 2
    # The route is in world coordinates, so the steering pose must equal Gazebo's truth now.
    truth = gz_truth(args.model)
    if truth and not args.force:
        p = node.pose()
        pe, ye = err(p, truth)
        print(f"Start check: steering pose {fmt(p)}  gazebo {fmt(truth)}")
        if float(pe) > 0.20 or float(ye) > 5.0 or math.hypot(*truth[:2]) > 0.20:
            print(f"REFUSED: steering pose is {pe} m / {ye}° from the truth, or the robot is not at the spawn"
                  " point. Restart Gazebo and respawn the robot at 0,0,0, restart RTAB-Map"
                  " with -d, then run this again. (--force skips this check.)")
            node.destroy_node()
            rclpy.shutdown()
            return 3
    # Safety guard: poll Gazebo truth in the background and stop if the steering pose disagrees.
    # Truth is sampled about once a second (the gz CLI is slow), so it is only compared while the robot
    # is standing still (settling after a turn, or pausing at a waypoint) and the sample was taken
    # after it stopped. Comparing while moving would flag the sample's age, not a real error.
    guard = {"truth": None, "t": 0.0, "stop": False}

    def poll_truth():
        while not guard["stop"]:
            t_req = time.time()
            tr = gz_truth(args.model)
            guard["truth"], guard["t"] = tr, t_req
            time.sleep(0.5)

    if not args.no_guard:
        threading.Thread(target=poll_truth, daemon=True).start()
    aborted = None
    seen = 0
    last = node.pose()
    try:
        while rclpy.ok() and not follower.done:
            rclpy.spin_once(node, timeout_sec=0.05)
            p = node.pose() or last
            last = p
            tr = guard["truth"]
            still = follower.state in ("settle", "arrived") and guard["t"] > follower.t_state + 0.3
            if tr and still and not args.no_guard:
                pe = math.hypot(p[0] - tr[0], p[1] - tr[1])
                ye = abs(math.degrees(wrap(p[2] - tr[2])))
                if pe > args.guard_m or ye > args.guard_deg:
                    aborted = (f"STOPPED: steering pose {fmt(p)} is {pe:.2f} m / {ye:.1f}° from Gazebo truth "
                               f"{fmt(tr)} (limit {args.guard_m} m / {args.guard_deg}°). Localization is lost; the "
                               "map is no longer trustworthy. Look at the robot in Gazebo (stuck? tipped?), then "
                               "restart from scratch.")
                    break
            v, w = follower.step(time.time(), *p, node.front)
            node.send(v, w)
            while seen < len(follower.events):
                t, kind, i = follower.events[seen]
                seen += 1
                label = ROUTE[i][2]
                mp, op = node.map_pose(), node.odom
                if kind == "blocked":
                    print(f"[{kind:7s}] {i + 1:2d}/{len(ROUTE)} {label:30s} map {fmt(mp)}  (something within 0.25 m ahead)")
                    continue
                truth = gz_truth(args.model)
                me, mye = err(mp, truth)
                oe, oye = err(op, truth)
                print(f"[{kind:7s}] {i + 1:2d}/{len(ROUTE)} {label:30s} gazebo {fmt(truth)}"
                      f"  map err {me or '?'} m {mye or '?'}°   odom err {oe or '?'} m {oye or '?'}°")
                row = [i + 1, label, kind]
                for q in (mp, op, truth):
                    row += [f"{q[0]:.3f}", f"{q[1]:.3f}", f"{math.degrees(q[2]):.1f}"] if q else ["", "", ""]
                wr.writerow(row + [me, mye, oe, oye])
                log.flush()
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C.")
    finally:
        guard["stop"] = True
        for _ in range(5):
            node.send(0.0, 0.0)
            time.sleep(0.05)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    if aborted:
        print("\n" + aborted)
        return 4
    reached = sum(1 for e in follower.events if e[1] == "reached")
    skipped = sum(1 for e in follower.events if e[1] == "skipped")
    print(f"\nRoute finished: {reached} reached, {skipped} skipped, of {len(ROUTE)}. Log: {log_path}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", default="arc", help="robot model name in Gazebo (for the truth comparison)")
    ap.add_argument("--log", default="~/arc_auto_map_log.csv")
    ap.add_argument("--unstamped", action="store_true", help="publish Twist instead of TwistStamped")
    ap.add_argument("--pose-source", choices=["map", "odom"], default="map",
                    help="steer by RTAB-Map's corrected pose (default) or raw wheel odometry")
    ap.add_argument("--guard-m", type=float, default=0.30, help="stop if steering pose is this far from truth")
    ap.add_argument("--guard-deg", type=float, default=10.0)
    ap.add_argument("--no-guard", action="store_true", help="disable the truth guard")
    ap.add_argument("--force", action="store_true", help="skip the odom-vs-truth start check")
    ap.add_argument("--dry-run", action="store_true", help="print the route and its clearances; no ROS")
    args = ap.parse_args(argv)
    if args.dry_run:
        worst = min(r[3] for r in check_route())
        for label, p, q, d, name in check_route():
            print(f"  ({p[0]:5.2f},{p[1]:5.2f}) -> ({q[0]:5.2f},{q[1]:5.2f})  {label:32s} closest: {name} {d:.2f} m")
        print(f"Worst clearance from the robot centre: {worst:.2f} m (Burger radius 0.105 m)")
        return 0
    return run_ros(args)


if __name__ == "__main__":
    sys.exit(main())
