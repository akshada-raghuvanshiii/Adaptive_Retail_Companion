#!/usr/bin/env python3
"""Publish RViz markers for the compact store: shelf outlines, aisle names, and every item stop
(a disc at the stop, an arrow for the heading, and a label with the section and its items).

Reads ~/slam_ws/src/arc_store_sim/config/item_locations.yaml (frame map) and the fixtures in
layout.py. Publishes once, latched (TRANSIENT_LOCAL), on /arc/store_markers; re-publishes every
5 s so RViz picks it up whenever it starts. Display only: nothing subscribes to it but RViz.

    python3 item_markers.py
"""

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import layout as L  # noqa: E402

AISLE_COLOUR = {
    "Fresh": (0.30, 0.75, 0.35),
    "Aisle 1": (0.25, 0.55, 0.95),
    "Aisle 2": (0.95, 0.55, 0.20),
    "Aisle 3": (0.75, 0.35, 0.85),
}
AISLE_LABELS = [("Aisle 1", 0.15, 2.35), ("Aisle 2", 1.80, 2.35), ("Aisle 3", 3.47, 2.35)]


def stops_from_items(doc):
    """Group items by their stop (x, y, yaw). Works with or without the aisle/section fields."""
    groups = defaultdict(list)
    for name, rec in (doc.get("items") or {}).items():
        key = (round(float(rec["x"]), 2), round(float(rec["y"]), 2), round(float(rec.get("yaw", 0.0)), 3))
        groups[key].append((name, rec))
    out = []
    for (x, y, yaw), members in sorted(groups.items()):
        rec0 = members[0][1]
        out.append({"x": x, "y": y, "yaw": yaw,
                    "aisle": rec0.get("aisle", ""), "section": rec0.get("section", rec0.get("shelf", "")),
                    "items": sorted(n for n, _ in members)})
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--items", default=str(Path.home() / "slam_ws/src/arc_store_sim/config/item_locations.yaml"))
    ap.add_argument("--topic", default="/arc/store_markers")
    args = ap.parse_args(argv)

    import math

    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray

    doc = yaml.safe_load(Path(args.items).expanduser().read_text())
    frame = doc.get("frame_id", "map")
    stops = stops_from_items(doc)

    rclpy.init()
    node = Node("arc_store_markers")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, args.topic, qos)

    def base(ns, mid, mtype):
        m = Marker()
        m.header.frame_id = frame
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def colour(m, rgb, a=1.0):
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = a

    arr = MarkerArray()
    # shelf outlines + names
    for i, f in enumerate(L.FIXTURES):
        m = base("shelves", i, Marker.LINE_STRIP)
        m.scale.x = 0.03
        colour(m, (0.85, 0.85, 0.85))
        for x, y in ((f.x0, f.y0), (f.x1, f.y0), (f.x1, f.y1), (f.x0, f.y1), (f.x0, f.y0)):
            m.points.append(Point(x=x, y=y, z=0.02))
        arr.markers.append(m)
        t = base("shelf_names", i, Marker.TEXT_VIEW_FACING)
        t.pose.position.x, t.pose.position.y = f.centre
        t.pose.position.z = 0.3
        t.scale.z = 0.14
        colour(t, (1.0, 1.0, 1.0))
        t.text = f.name.replace("_", " ")
        arr.markers.append(t)
    # aisle names
    for i, (name, x, y) in enumerate(AISLE_LABELS):
        t = base("aisles", i, Marker.TEXT_VIEW_FACING)
        t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.5
        t.scale.z = 0.22
        colour(t, AISLE_COLOUR[name])
        t.text = name.upper()
        arr.markers.append(t)
    # item stops
    for i, s in enumerate(stops):
        rgb = AISLE_COLOUR.get(s["aisle"], (1.0, 0.85, 0.2))
        d = base("stops", i, Marker.CYLINDER)
        d.pose.position.x, d.pose.position.y, d.pose.position.z = s["x"], s["y"], 0.01
        d.scale.x = d.scale.y = 0.22
        d.scale.z = 0.02
        colour(d, rgb, 0.8)
        arr.markers.append(d)
        a = base("headings", i, Marker.ARROW)
        a.pose.position.x, a.pose.position.y, a.pose.position.z = s["x"], s["y"], 0.03
        a.pose.orientation.z, a.pose.orientation.w = math.sin(s["yaw"] / 2), math.cos(s["yaw"] / 2)
        a.scale.x, a.scale.y, a.scale.z = 0.25, 0.04, 0.04
        colour(a, rgb)
        arr.markers.append(a)
        t = base("labels", i, Marker.TEXT_VIEW_FACING)
        t.pose.position.x, t.pose.position.y, t.pose.position.z = s["x"], s["y"], 0.35
        t.scale.z = 0.10
        colour(t, rgb)
        t.text = (s["section"] + "\n" if s["section"] else "") + ", ".join(s["items"])
        arr.markers.append(t)

    def publish():
        now = node.get_clock().now().to_msg()
        for m in arr.markers:
            m.header.stamp = now
        pub.publish(arr)

    publish()
    node.create_timer(5.0, publish)
    node.get_logger().info(f"Publishing {len(stops)} item stops and {len(L.FIXTURES)} shelves on {args.topic} "
                           f"(frame {frame}). Ctrl+C to stop.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
