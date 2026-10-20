#!/usr/bin/env python3
"""Pre-flight for the ARC tests. Changes nothing in the ARC stack.

Run with the navigation terminals (B1-B5) up, B6 set to 3 (LEAD) and the
Shopping Manager (B7) closed. Prints OK / WARN / FAIL per check and saves
results/probe.json.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy  # noqa: E402

from arc_tests.common import find_item_file, group_stops, load_items  # noqa: E402
from arc_tests.rosio import (FOLLOW_PATH_STATUS, REACHED_TOPIC, Gz, TruthBridge, Watcher,  # noqa: E402
                             find_goal_topic, running_nodes, stack_config, wrap)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="arc_store")
    ap.add_argument("--model", default="arc")
    ap.add_argument("--items", default=None)
    ap.add_argument("--out", default="results")
    ap.add_argument("--truth", choices=["auto", "bridge", "gzcli"], default="auto")
    ap.add_argument("--no-box", action="store_true", help="skip the spawn/remove box test")
    ap.add_argument("--box-at", default="3.9,-2.6", help="x,y for the test box (free floor in the south-east corner)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    w = Watcher(model=args.model, truth=args.truth)
    bridge = TruthBridge(args.world) if args.truth != "gzcli" else None
    if bridge:
        bridge.start()
    report, info = [], {}

    def line(status, what, detail=""):
        report.append((status, what, detail))
        print(f"[{status:4}] {what}" + (f" - {detail}" if detail else ""))

    try:
        w.spin_for(4.0)
        nodes = running_nodes(w)
        info["nodes"] = nodes
        for needed, tab in (("dijkstra", "B3"), ("controller_server", "B1"), ("planned_path_to_teb", "B4"),
                            ("rtabmap", "T5b"), ("motion_gate", "B5")):
            hit = [n for n in nodes if needed in n]
            line("OK" if hit else "FAIL", f"node {needed}", ", ".join(hit) or f"not running - start {tab}")
        cfg = stack_config(w)
        info["stack"] = cfg
        line("OK" if cfg["odometry"] != "unknown" else "WARN", "RTAB-Map odometry",
             f"{cfg['odometry']} ({cfg['rtabmap_odom_topic']}); icp_odometry running: {cfg['icp_odometry_running']}")
        line("OK" if cfg["controller"] != "unknown" else "WARN", "controller plugin", str(cfg["controller_plugin"]))
        if cfg["shopping_manager_running"]:
            line("WARN", "shopping_manager is running", "close tab B7 before tests; it would send its own goals")
        gt, owner = find_goal_topic(w)
        info["goal_topic"] = gt
        line("OK" if owner else "FAIL", "goal topic", f"{gt}" + (f" (subscribed by {owner})" if owner else
                                                                 " - the Dijkstra planner is not subscribed"))
        pubs = [i.node_name for i in w.get_publishers_info_by_topic(REACHED_TOPIC)]
        line("OK" if pubs else "FAIL", REACHED_TOPIC, f"published by {', '.join(pubs)}" if pubs else "no publisher (B4)")
        w.spin_for(1.0)
        line("OK" if w.ctrl_status_seen or [i for i in w.get_publishers_info_by_topic(FOLLOW_PATH_STATUS)] else "WARN",
             "FollowPath action status", "visible" if w.ctrl_status_seen else "publisher present, no goals yet")

        got_map = w.wait_for(lambda: w.map_msg is not None, 10)
        if got_map:
            m = w.map_msg.info
            line("OK", "map /rtabmap/map", f"{m.width}x{m.height} @ {m.resolution:.3f} m")
            info["map"] = {"w": m.width, "h": m.height, "res": m.resolution}
        else:
            line("FAIL", "map /rtabmap/map", "no map received in 10 s")

        mp = None
        if w.wait_for(lambda: w.map_pose() is not None, 8):
            mp = w.map_pose()
            line("OK", "TF map -> base_footprint", f"x={mp[0]:.2f} y={mp[1]:.2f} yaw={math.degrees(mp[2]):.1f}")
        else:
            line("FAIL", "TF map -> base_footprint", "not available")

        if w.wait_truth(8.0):
            tr = w.fresh_truth()
            info["truth_source"] = tr[5]
            line("OK", "Gazebo true pose", f"x={tr[0]:.2f} y={tr[1]:.2f} yaw={math.degrees(tr[2]):.1f} (source {tr[5]})")
            if mp:
                mp = w.map_pose(tr[4])
                e = math.hypot(mp[0] - tr[0], mp[1] - tr[1])
                ye = abs(math.degrees(wrap(mp[2] - tr[2])))
                ok = e <= 0.15 and ye <= 5
                line("OK" if ok else "WARN", "map vs truth",
                     f"{e:.3f} m, {ye:.1f} deg ({'PASS' if ok else 'outside 0.15 m / 5 deg'})")
        else:
            names = ", ".join(sorted(n for n in w.truth_names if n)[:12]) or "nothing named"
            line("FAIL", "Gazebo true pose", f"model '{args.model}' not found (bridge saw: {names}; gz model failed too)")

        line("OK", "/cmd_vel type", w.cmd_type.__name__ + (" (matches arc_motion_gate)" if w.cmd_type.__name__ == "TwistStamped" else ""))
        w.wait_for(lambda: w.mode() is not None, 3)
        mode = w.mode()
        line("OK" if mode in ("LEAD", "FOLLOW") else "FAIL", "BT mode /arc/behavior_mode",
             f"{mode}" + ("" if mode in ("LEAD", "FOLLOW") else " - the gate holds the robot; type 3 in B6"))
        info["bt_mode"] = mode

        path = args.items or find_item_file()
        if path and os.path.exists(path):
            try:
                items = load_items(path)
                stops = group_stops(items)
                line("OK", "item file", f"{path}: {len(items)} items, {len(stops)} driving stops")
                info["items_file"] = path
            except Exception as exc:  # noqa: BLE001
                line("FAIL", "item file", f"{path}: {exc}")
        else:
            line("FAIL", "item file", "item_locations.yaml not found; pass --items PATH")

        gz = Gz(args.world)
        line("OK" if gz.ok else "WARN", "gz CLI", "found" if gz.ok else "not on PATH; obstacle tests (A4) unavailable")
        if gz.ok:
            left = gz.test_boxes()
            if left is None:
                line("WARN", "leftover test boxes", "could not read `gz model --list`")
            elif left:
                stuck = gz.sweep_test_boxes()
                line("FAIL" if stuck else "WARN", "leftover test boxes",
                     f"{len(left)} found, " + (f"could not remove {' '.join(stuck)}" if stuck else "all removed now"))
            else:
                line("OK", "leftover test boxes", "none in Gazebo")
        if gz.ok and not args.no_box:
            bx, by = (float(v) for v in args.box_at.split(","))
            ok = gz.spawn_box("arc_probe_box", bx, by)
            time.sleep(0.5)
            seen = "arc_probe_box" in (gz.test_boxes() or [])
            ok2 = gz.remove_verified("arc_probe_box") if ok else False
            good = ok and seen and ok2
            line("OK" if good else "WARN", "spawn/remove test box",
                 "works (checked in `gz model --list`)" if good else
                 f"failed (spawn {ok}, listed {seen}, removed {ok2}); A4 unavailable")
    finally:
        if bridge:
            bridge.stop()
        if w.poller:
            w.poller.stop()
        info["report"] = report
        with open(os.path.join(args.out, "probe.json"), "w") as f:
            json.dump(info, f, indent=2)
        w.destroy_node()
        rclpy.shutdown()
    fails = sum(1 for s, _, _ in report if s == "FAIL")
    print(f"\n{fails} FAIL, {sum(1 for s, _, _ in report if s == 'WARN')} WARN. Saved {args.out}/probe.json")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
