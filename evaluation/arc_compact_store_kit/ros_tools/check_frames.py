#!/usr/bin/env python3
"""Verify the coordinate convention: Gazebo world pose of the robot vs RTAB-Map's
map -> base_footprint, at several places in the store.

PASS when every sample agrees within --pos-tol metres and --yaw-tol degrees.
That is the evidence that destinations written in "map" are also correct Gazebo
world positions, without inventing any transform.

    source ~/slam_ws/install/setup.bash
    python3 check_frames.py --model turtlebot3_waffle --samples 4
Drive the robot (teleop) to a new spot between samples, then press Enter.

Gazebo truth comes from `gz model -m <model> -p`. If that command is unavailable
or its output cannot be parsed, you are asked to type the pose shown in the
Gazebo GUI (Entity Tree -> model -> Pose).

UNTESTED in the build sandbox (no ROS there): first run on your machine is the test.
"""

import argparse
import math
import re
import subprocess
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
TRIPLE = re.compile(rf"\[\s*({NUM})\s+({NUM})\s+({NUM})\s*\]")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def gz_truth(model):
    try:
        out = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=10).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None, "gz CLI not available"
    after = out.split("Pose", 1)[1] if "Pose" in out else ""
    triples = TRIPLE.findall(after)
    if len(triples) < 2:
        return None, f"could not parse gz output:\n{out.strip()[:400]}"
    (x, y, _), (_, _, yaw) = triples[0], triples[1]
    return (float(x), float(y), float(yaw)), "gz model"


def ask_truth():
    s = input("  Type the Gazebo pose as 'x y yaw_deg' (from the GUI): ").split()
    return float(s[0]), float(s[1]), math.radians(float(s[2]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", required=True, help="robot model name in Gazebo")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--samples", type=int, default=4)
    ap.add_argument("--pos-tol", type=float, default=0.15)
    ap.add_argument("--yaw-tol", type=float, default=5.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Node("arc_check_frames")
    buf = Buffer()
    TransformListener(buf, node)
    worst_p = worst_y = 0.0
    for k in range(args.samples):
        input(f"\nSample {k + 1}/{args.samples}: stop the robot somewhere new, then press Enter...")
        t_end = time.time() + 1.5
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        try:
            m = buf.lookup_transform(args.map_frame, args.base_frame, Time(), timeout=Duration(seconds=2.0))
            o = buf.lookup_transform(args.odom_frame, args.base_frame, Time(), timeout=Duration(seconds=2.0))
        except Exception as exc:
            print(f"  TF lookup failed: {type(exc).__name__}: {exc}")
            continue
        truth, src = gz_truth(args.model)
        if truth is None:
            print(f"  {src}")
            truth = ask_truth()
            src = "typed"
        mx, my, myaw = m.transform.translation.x, m.transform.translation.y, yaw_of(m.transform.rotation)
        ox, oy, oyaw = o.transform.translation.x, o.transform.translation.y, yaw_of(o.transform.rotation)
        dp = math.hypot(mx - truth[0], my - truth[1])
        dy = abs(math.degrees((myaw - truth[2] + math.pi) % (2 * math.pi) - math.pi))
        worst_p, worst_y = max(worst_p, dp), max(worst_y, dy)
        print(f"  Gazebo world ({src}): ({truth[0]:.3f}, {truth[1]:.3f}, {math.degrees(truth[2]):.1f} deg)")
        print(f"  map  -> {args.base_frame}: ({mx:.3f}, {my:.3f}, {math.degrees(myaw):.1f} deg)  "
              f"error {dp:.3f} m, {dy:.1f} deg")
        print(f"  odom -> {args.base_frame}: ({ox:.3f}, {oy:.3f}, {math.degrees(oyaw):.1f} deg)")
    node.destroy_node()
    rclpy.shutdown()
    ok = worst_p <= args.pos_tol and worst_y <= args.yaw_tol
    print(f"\n{'PASS' if ok else 'FAIL'}: worst map-vs-world error {worst_p:.3f} m, {worst_y:.1f} deg "
          f"(tolerance {args.pos_tol} m, {args.yaw_tol} deg)")
    if not ok:
        print("Map and Gazebo world do NOT coincide. Most likely the robot did not spawn at (0, 0, yaw 0), "
              "or RTAB-Map loaded an old database. See RUNBOOK 'If check_frames fails'.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
