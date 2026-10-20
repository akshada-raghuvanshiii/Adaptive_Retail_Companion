#!/usr/bin/env python3
"""Drive-test the existing pipeline to one or all compact-store destinations.

For each destination it publishes a PoseStamped (frame map) on --topic
(default /shopping_goal, which your Dijkstra planner subscribes to), then reports:
  path       did a /planned_path arrive, how many poses, does it end near the goal?
  motion     did the robot actually move (TF map -> base_footprint)?
  arrival    did it get within --tolerance of the goal before --timeout?
The CSV result separates "no path", "path but no motion" and "moved but did not arrive",
which point at planner, controller/motion-gate, and controller tuning respectively.

    source ~/slam_ws/install/setup.bash
    python3 send_test_goal.py --dest produce
    python3 send_test_goal.py --all --out drive_test.csv

Stop or idle the Shopping Manager first so it does not publish competing goals, and put
the Behavior Tree / arc_motion_gate in a state that allows motion.

UNTESTED in the build sandbox (no ROS there): the first run on your machine is the test.
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

HERE = Path(__file__).resolve().parent


def load_destinations(path):
    data = yaml.safe_load(Path(path).read_text())
    if data.get("frame_id") != "map":
        raise SystemExit(f"{path} is not in frame 'map'")
    return data["destinations"]


class Tester(Node):
    def __init__(self, topic, path_topic, map_frame, base_frame):
        super().__init__("arc_send_test_goal")
        # Must match dijkstra_planner's /shopping_goal subscription (RELIABLE, TRANSIENT_LOCAL,
        # depth 1); a VOLATILE publisher is incompatible and its goals would be silently dropped.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseStamped, topic, qos)
        self.create_subscription(PathMsg, path_topic, self._on_path, 10)
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.map_frame, self.base_frame = map_frame, base_frame
        self.last_path = None
        self.last_path_t = 0.0

    def _on_path(self, msg):
        self.last_path, self.last_path_t = msg, time.time()

    def pose(self):
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2))
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def goal_msg(self, d):
        m = PoseStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(d["x"]), float(d["y"])
        m.pose.orientation.z, m.pose.orientation.w = math.sin(d["yaw"] / 2), math.cos(d["yaw"] / 2)
        return m

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def run_one(node, did, d, timeout, tol):
    node.spin_for(1.0)
    start = node.pose()
    if start is None:
        return {"destination": did, "result": "FAIL", "reason": "no TF map -> base frame (is RTAB-Map running?)"}
    sent = time.time()
    node.pub.publish(node.goal_msg(d))
    republished = False
    travelled, prev, best = 0.0, start, math.hypot(start[0] - d["x"], start[1] - d["y"])
    path_info = None
    while time.time() - sent < timeout:
        node.spin_for(0.2)
        if node.last_path is not None and node.last_path_t >= sent and path_info is None:
            poses = node.last_path.poses
            end = poses[-1].pose.position if poses else None
            path_info = (len(poses), math.hypot(end.x - d["x"], end.y - d["y"]) if end else float("nan"),
                         node.last_path.header.frame_id)
        if path_info is None and not republished and time.time() - sent > 2.0:
            node.pub.publish(node.goal_msg(d))
            republished = True
        p = node.pose()
        if p is None:
            continue
        travelled += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        dist = math.hypot(p[0] - d["x"], p[1] - d["y"])
        best = min(best, dist)
        if dist <= tol:
            node.spin_for(1.0)
            p = node.pose() or p
            yaw_err = abs(math.degrees((p[2] - d["yaw"] + math.pi) % (2 * math.pi) - math.pi))
            return {"destination": did, "result": "PASS", "reason": "arrived",
                    "time_s": round(time.time() - sent, 1), "final_dist_m": round(dist, 3),
                    "final_yaw_err_deg": round(yaw_err, 1), "travelled_m": round(travelled, 2),
                    "path_poses": path_info[0] if path_info else "", "path_end_err_m":
                    round(path_info[1], 3) if path_info else "", "path_frame": path_info[2] if path_info else ""}
    if path_info is None:
        reason = "no /planned_path received after the goal (planner rejected it? see planner log)"
    elif travelled < 0.10:
        reason = "path received but robot did not move (controller / bridge / arc_motion_gate / BT state)"
    else:
        reason = f"moved {travelled:.2f} m but did not arrive (closest {best:.2f} m)"
    return {"destination": did, "result": "FAIL", "reason": reason, "time_s": timeout,
            "final_dist_m": round(best, 3), "travelled_m": round(travelled, 2),
            "path_poses": path_info[0] if path_info else "",
            "path_end_err_m": round(path_info[1], 3) if path_info else "",
            "path_frame": path_info[2] if path_info else ""}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--destinations", default=str(HERE.parent / "output" / "destinations_map.yaml"))
    ap.add_argument("--dest", nargs="*", default=[])
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--topic", default="/shopping_goal")
    ap.add_argument("--path-topic", default="/planned_path")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--tolerance", type=float, default=0.25)
    ap.add_argument("--out", default="drive_test_results.csv")
    args = ap.parse_args(argv)

    dests = load_destinations(args.destinations)
    ids = list(dests) if args.all else args.dest
    if not ids:
        ap.error("give --dest ID [ID ...] or --all; ids: " + ", ".join(dests))
    rclpy.init()
    node = Tester(args.topic, args.path_topic, args.map_frame, args.base_frame)
    rows = []
    for did in ids:
        if did not in dests:
            print(f"unknown destination {did}")
            continue
        print(f"-> {did} ({dests[did]['x']:.2f}, {dests[did]['y']:.2f}) ...", flush=True)
        r = run_one(node, did, dests[did], args.timeout, args.tolerance)
        rows.append(r)
        print(f"   {r['result']}: {r['reason']}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    fields = ["destination", "result", "reason", "time_s", "final_dist_m", "final_yaw_err_deg",
              "travelled_m", "path_poses", "path_end_err_m", "path_frame"]
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    n = sum(r["result"] == "PASS" for r in rows)
    print(f"\n{n}/{len(rows)} destinations reached. Results: {args.out}")
    return 0 if n == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
