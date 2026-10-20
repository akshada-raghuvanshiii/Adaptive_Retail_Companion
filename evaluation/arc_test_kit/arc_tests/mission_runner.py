#!/usr/bin/env python3
"""Run repeatable shopping missions through the live ARC stack and log them.

The runner stands in for the Shopping Manager. It picks reproducible shopping
lists (or fixed start-goal pairs), orders the stops with the chosen strategy,
sends each stop on /goal_pose, and records what the Dijkstra planner, the
planned_path_to_teb bridge and the controller actually do.

Arrival is ARC's own signal: /navigation_goal_reached = True after the goal
was sent (the same message the Shopping Manager waits for). A leg that ends
within --tol of the stop with the robot stopped and the controller idle, but
without that message, is logged as 'arrived_tol'. Before every goal the runner
waits for the FollowPath action to be idle, because planned_path_to_teb ignores
new paths while one is running.

Experiments (see README): A1 localisation (every run), A2 reliability
(--lists 30 --size-range 3-12), A3 controllers (--pairs 20, run once per
controller), A4 boxes (--pairs 20 --obstacles 1-3), B1 odometry (--lists 10,
wheel-odometry stack), C2 ordering (--sizes 5,8,12 --per-size 10 --strategies ...).

Outputs in --out (appended, safe to Ctrl-C): samples.csv, legs.csv,
missions.csv, events.csv, run_<label>_<time>.json, map.npz
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy  # noqa: E402

from arc_tests.common import (STRATEGIES, Grid, cost_matrix, find_item_file, group_stops,  # noqa: E402
                              load_items, make_lists, make_pairs, make_random_size_lists, order_for,
                              parse_range, route_cost, stops_for_list)
from arc_tests.rosio import (GOAL_TOPIC, Gz, TruthBridge, Watcher, find_goal_topic,  # noqa: E402
                             label_mismatch, stack_config, wrap)

SAMPLE_COLS = ["label", "mission", "strategy", "leg", "wall_t", "sim_t", "map_x", "map_y", "map_yaw",
               "true_x", "true_y", "true_yaw", "pos_err", "yaw_err_deg", "truth_src", "v_cmd", "w_cmd",
               "bt_mode", "static_clearance", "truth_age_s",
               "corr_x", "corr_y", "corr_yaw", "lidar_x", "lidar_y", "lidar_yaw", "lidar_err",
               "wheel_x", "wheel_y", "wheel_yaw", "wheel_err"]
LEG_COLS = ["label", "mission", "strategy", "n_items", "n_stops", "leg", "stop_id", "goal_x", "goal_y",
            "planned_len", "paths_received", "driven_len_true", "driven_len_map", "time_s", "success",
            "reason", "reached_msg", "final_dist", "ctrl_aborts", "resends", "stall_events", "n_obstacles",
            "min_obstacle_clearance", "min_static_clearance", "max_pos_err", "w_sign_changes",
            "mean_abs_wdot", "reverse_s", "idle_wait_s", "cmd_msgs"]
MISSION_COLS = ["label", "mission", "list_id", "strategy", "n_items", "n_stops", "order",
                "planned_cost_J", "exact_cost_J", "driven_total", "time_total", "legs_ok", "legs_total",
                "completed", "recoveries", "collisions", "n_obstacles"]
EVENT_COLS = ["label", "mission", "leg", "wall_t", "sim_t", "event", "detail"]


class Csv:
    def __init__(self, path, cols):
        new = not os.path.exists(path) or os.path.getsize(path) == 0
        if not new:
            with open(path) as f:
                head = f.readline().strip().split(",")
            if head != cols:
                # older kit layout: keep it, start a fresh file next to it
                os.rename(path, path.replace(".csv", f".old_{time.strftime('%Y%m%d_%H%M%S')}.csv"))
                new = True
        self.f = open(path, "a", newline="")
        self.w = csv.DictWriter(self.f, fieldnames=cols)
        self.cols = cols
        if new:
            self.w.writeheader()

    def row(self, d):
        self.w.writerow({k: d.get(k, "") for k in self.cols})
        self.f.flush()

    def close(self):
        self.f.close()


def grid_from_msg(msg, robot_radius):
    i = msg.info
    return Grid(list(msg.data), i.width, i.height, i.resolution,
                i.origin.position.x, i.origin.position.y, robot_radius=robot_radius)


def smoothness(cmd_log, max_gap=0.5):
    """(mean |dw/dt| in rad/s^2, w sign reversals, seconds reversing, messages) from (sim_t, v, w)."""
    acc, flips, rev = [], 0, 0.0
    last_sign = 0
    for (t0, v0, w0), (t1, v1, w1) in zip(cmd_log, cmd_log[1:]):
        dt = t1 - t0
        if 1e-3 < dt <= max_gap:
            acc.append(abs(w1 - w0) / dt)
            if v0 < -0.01:
                rev += dt
    for _, _, w in cmd_log:
        if abs(w) > 0.05:
            s = 1 if w > 0 else -1
            if last_sign and s != last_sign:
                flips += 1
            last_sign = s
    return (float(np.mean(acc)) if acc else None), flips, rev, len(cmd_log)


def static_clearance(occ, x, y, robot_radius, window=1.0):
    """Robot edge to nearest mapped obstacle cell (m); None if nothing within `window`."""
    if occ is None or not len(occ):
        return None
    near = occ[(np.abs(occ[:, 0] - x) < window) & (np.abs(occ[:, 1] - y) < window)]
    if not len(near):
        return None
    return float(np.min(np.hypot(near[:, 0] - x, near[:, 1] - y))) - robot_radius


class Runner:
    def __init__(self, args, w, gz, occ):
        self.a, self.w, self.gz, self.occ = args, w, gz, occ
        os.makedirs(args.out, exist_ok=True)
        self.samples = Csv(os.path.join(args.out, "samples.csv"), SAMPLE_COLS)
        self.legs = Csv(os.path.join(args.out, "legs.csv"), LEG_COLS)
        self.missions = Csv(os.path.join(args.out, "missions.csv"), MISSION_COLS)
        self.events = Csv(os.path.join(args.out, "events.csv"), EVENT_COLS)
        self.box_seq = 0
        self.cancel_cli = None

    def close(self):
        for c in (self.samples, self.legs, self.missions, self.events):
            c.close()

    def event(self, mission, leg, ev, detail=""):
        self.events.row({"label": self.a.label, "mission": mission, "leg": leg, "wall_t": f"{time.time():.3f}",
                         "sim_t": f"{self.w.sim_now():.3f}", "event": ev, "detail": detail})

    def cancel_follow_path(self):
        """Cancel every FollowPath goal (zero uuid + zero stamp = all), so the next leg starts clean."""
        try:
            from action_msgs.srv import CancelGoal
            if self.cancel_cli is None:
                self.cancel_cli = self.w.create_client(CancelGoal, "/follow_path/_action/cancel_goal")
            if not self.cancel_cli.wait_for_service(timeout_sec=2.0):
                return False
            fut = self.cancel_cli.call_async(CancelGoal.Request())
            self.w.wait_for(fut.done, 3.0)
            return fut.done()
        except Exception:  # noqa: BLE001
            return False

    # ------------------------------------------------------------------ samples
    def sample(self, mission, strategy, leg):
        w = self.w
        tr = w.fresh_truth()
        mp = w.map_pose(tr[4] if tr and tr[4] is not None else None)
        v, wz = w.cmd_now()
        row = {"label": self.a.label, "mission": mission, "strategy": strategy, "leg": leg,
               "wall_t": f"{time.time():.3f}", "sim_t": f"{w.sim_now():.3f}",
               "v_cmd": f"{v:.4f}", "w_cmd": f"{wz:.4f}", "bt_mode": w.mode() or ""}
        if mp:
            row.update(map_x=f"{mp[0]:.4f}", map_y=f"{mp[1]:.4f}", map_yaw=f"{mp[2]:.4f}")
        if tr:
            row.update(true_x=f"{tr[0]:.4f}", true_y=f"{tr[1]:.4f}", true_yaw=f"{tr[2]:.4f}", truth_src=tr[5],
                       truth_age_s=f"{time.time() - tr[3]:.3f}")
            if mp:
                row["pos_err"] = f"{math.hypot(mp[0] - tr[0], mp[1] - tr[1]):.4f}"
                row["yaw_err_deg"] = f"{abs(math.degrees(wrap(mp[2] - tr[2]))):.3f}"
            # odometry frames start where the robot started (world origin, yaw 0), so they compare to truth
            for key, val in w.tf_layers().items():
                if val is None:
                    continue
                row.update({f"{key}_x": f"{val[0]:.4f}", f"{key}_y": f"{val[1]:.4f}", f"{key}_yaw": f"{val[2]:.4f}"})
                if key != "corr":
                    row[f"{key}_err"] = f"{math.hypot(val[0] - tr[0], val[1] - tr[1]):.4f}"
            c = static_clearance(self.occ, tr[0], tr[1], self.a.robot_radius)
            if c is not None:
                row["static_clearance"] = f"{c:.3f}"
        return row

    def place_obstacles(self, n, pts, here, goal):
        """n boxes on the planned path at 30/45/70 % of its length, nudged sideways."""
        if n <= 0 or not pts or len(pts) < 3 or not self.gz.ok:
            return []
        cum = [0.0]
        for p0, p1 in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
        total = cum[-1]
        if total < 1.5:
            return []
        boxes = []
        for k, frac in enumerate([0.45, 0.70, 0.30][:n]):
            target = frac * total
            i = next(j for j, c in enumerate(cum) if c >= target)
            i = max(1, min(i, len(pts) - 1))
            (x0, y0), (x1, y1) = pts[i - 1], pts[i]
            hd = math.atan2(y1 - y0, x1 - x0)
            side = self.a.box_offset * (1 if k % 2 == 0 else -1)
            bx, by = x1 - side * math.sin(hd), y1 + side * math.cos(hd)
            if math.hypot(bx - goal[0], by - goal[1]) < 0.6 or math.hypot(bx - here[0], by - here[1]) < 0.6:
                continue
            self.box_seq += 1
            name = f"arc_test_box_{self.box_seq}"
            if self.gz.spawn_box(name, bx, by, size=self.a.box_size):
                boxes.append((name, bx, by))
        return boxes

    # ------------------------------------------------------------------ one leg
    def run_leg(self, mission, strategy, leg, stop, n_items, n_stops, log=True, n_boxes=0):
        a, w = self.a, self.w
        goal = (stop["x"], stop["y"], stop.get("yaw", 0.0))
        t_idle = time.time()
        if w.controller_active():
            self.event(mission, leg, "wait_controller_idle")
            if not w.wait_for(lambda: not w.controller_active(), a.idle_wait):
                self.event(mission, leg, "controller_still_busy", "cancelling FollowPath")
                self.cancel_follow_path()
                w.wait_for(lambda: not w.controller_active(), 5.0)
        idle_wait = time.time() - t_idle
        path_before = w.last_path[0] if w.last_path else 0.0
        t0 = time.time()
        sim0 = w.sim_now()
        n_cmd0 = len(w.cmd_log)
        aborts0, reached0 = len(w.ctrl_aborts), len(w.reached_times)
        # A path counts for this goal only if it ends near it. After an arrival the planner may still
        # publish a re-plan to the OLD goal (it replans whenever the start cell changes); on 15 Oct that
        # 0.07 m path was taken as this leg's path, run by the path bridge and "arrived" in 0.02 s.
        def for_this_goal(lp):
            return bool(lp and lp[2] and math.hypot(lp[2][-1][0] - goal[0], lp[2][-1][1] - goal[1]) <= a.goal_snap_m)

        def new_path():
            return w.last_path and w.last_path[0] > path_before and for_this_goal(w.last_path)

        w.send_goal(a.goal_topic, *goal)
        self.event(mission, leg, "goal_sent", f"{stop.get('id', '?')} ({goal[0]:.2f}, {goal[1]:.2f})")
        if not w.wait_for(new_path, 6.0):
            w.send_goal(a.goal_topic, *goal)  # the planner runs every 2 s; one more try
            self.event(mission, leg, "goal_resent_no_path")
            w.wait_for(new_path, 6.0)
        got_path = bool(new_path())
        planned_len, boxes = "", []
        last_seen_path = path_before
        paths = 0
        if got_path:
            planned_len = round(w.last_path[1], 4)
            last_seen_path = w.last_path[0]
            paths = 1
            self.event(mission, leg, "path_received", f"{planned_len} m")
            if n_boxes > 0:
                here = w.pose() or (goal[0] + 99, goal[1])
                boxes = self.place_obstacles(n_boxes, w.last_path[2], here, goal)
                for name, bx, by in boxes:
                    self.event(mission, leg, "box_spawned", f"{name} ({bx:.2f}, {by:.2f})")
        else:
            self.event(mission, leg, "no_path")
        timeout = a.leg_timeout or max(90.0, 40.0 + 2.5 * (planned_len or 5.0) / a.speed)
        driven_t = driven_m = 0.0
        prev_t = prev_m = None
        stall_events, last_move_pose, last_move_t = 0, w.pose(), time.time()
        stalled_flag = False
        min_clear, min_static, max_err = float("inf"), float("inf"), 0.0
        still_since = None
        loss_since = None
        last_truth_t = time.time()
        sim_seen, sim_seen_t, gap_logged = w.sim_now(), time.time(), False
        ctrl_aborts = resends = far_msgs = 0
        reason, success, reached_msg = "timeout", False, 0
        next_sample = 0.0
        while time.time() - t0 < timeout:
            rclpy.spin_once(w, timeout_sec=0.02)
            now = time.time()
            if w.last_path and w.last_path[0] > last_seen_path:
                last_seen_path = w.last_path[0]
                if for_this_goal(w.last_path):
                    paths += 1
                    if not got_path:
                        got_path, planned_len = True, round(w.last_path[1], 4)
                else:
                    self.event(mission, leg, "stale_path", f"{w.last_path[1]:.2f} m, ends away from this goal")
            # ARC's completion signal - only if the robot is actually near this goal
            if len(w.reached_times) > reached0 + far_msgs and w.reached_times[-1] > t0:
                p_now = w.pose()
                d_now = math.hypot(p_now[0] - goal[0], p_now[1] - goal[1]) if p_now else float("inf")
                if d_now <= a.goal_snap_m:
                    reached_msg = 1
                    success, reason = True, "arrived"
                    self.event(mission, leg, "goal_reached_msg")
                    break
                far_msgs += 1
                self.event(mission, leg, "reached_msg_far", f"robot {d_now:.2f} m from this goal; ignored")
                if resends < a.retries:
                    resends += 1
                    w.wait_for(lambda: not w.controller_active(), 5.0)
                    w.send_goal(a.goal_topic, *goal)
                    self.event(mission, leg, "goal_resent", "after a reached message for another goal")
                    last_move_t = time.time()
            # controller gave up on the path
            if len(w.ctrl_aborts) > aborts0 + ctrl_aborts:
                ctrl_aborts += 1
                self.event(mission, leg, "controller_aborted")
                if resends < a.retries:
                    resends += 1
                    w.spin_for(1.0)
                    w.send_goal(a.goal_topic, *goal)
                    self.event(mission, leg, "goal_resent", "after controller abort")
                    last_move_t = time.time()
                else:
                    reason = "controller_aborted"
                    break
            if now < next_sample:
                continue
            next_sample = now + 1.0 / a.rate
            row = self.sample(mission, strategy, leg)
            self.samples.row(row)
            if row.get("pos_err"):
                pe = float(row["pos_err"])
                max_err = max(max_err, pe)
                if a.loss_abort_m > 0 and pe > a.loss_abort_m:
                    loss_since = loss_since or now
                    if now - loss_since >= a.loss_abort_s:
                        reason = "localisation_lost"
                        self.event(mission, leg, "localisation_lost",
                                   f"map pose {pe:.2f} m from truth for {a.loss_abort_s:.0f} s")
                        break
                else:
                    loss_since = None
            if row.get("static_clearance"):
                min_static = min(min_static, float(row["static_clearance"]))
            tr = w.fresh_truth()
            mp = w.map_pose()
            # Simulation link: /clock must keep advancing. If it stops, ROS no longer hears Gazebo
            # (and the robot may be driving on its last command), so stop the run.
            st_now = w.sim_now()
            if st_now > sim_seen + 1e-6:
                sim_seen, sim_seen_t = st_now, now
            elif a.link_abort_s > 0 and now - sim_seen_t > a.link_abort_s:
                reason = "sim_link_lost"
                self.event(mission, leg, "sim_link_lost", f"/clock frozen for {a.link_abort_s:.0f} s")
                break
            # The true pose comes from `gz model` (a Gazebo service). A gap there only blanks A1 samples.
            if tr:
                last_truth_t, gap_logged = now, False
            elif now - last_truth_t > 15 and not gap_logged:
                gap_logged = True
                self.event(mission, leg, "truth_gap", "no Gazebo pose for 15 s (gz service); A1 samples blank")
            if tr:
                if prev_t:
                    driven_t += math.hypot(tr[0] - prev_t[0], tr[1] - prev_t[1])
                prev_t = tr
                for _, bx, by in boxes:
                    min_clear = min(min_clear, math.hypot(tr[0] - bx, tr[1] - by) - a.box_size / 2 - a.robot_radius)
            if mp:
                if prev_m:
                    driven_m += math.hypot(mp[0] - prev_m[0], mp[1] - prev_m[1])
                prev_m = mp
            p = w.pose()
            if not p:
                continue
            dist = math.hypot(p[0] - goal[0], p[1] - goal[1])
            v, wz = w.cmd_now()
            moving = abs(v) > 0.02 or abs(wz) > 0.05
            if dist <= a.tol and not moving and not w.controller_active():
                still_since = still_since or now
                if now - still_since >= 2.0:
                    success, reason = True, "arrived_tol"
                    break
            else:
                still_since = None
            # stall: neither position nor heading changes
            if last_move_pose and (math.hypot(p[0] - last_move_pose[0], p[1] - last_move_pose[1]) > 0.05
                                   or abs(wrap(p[2] - last_move_pose[2])) > 0.1):
                last_move_pose, last_move_t, stalled_flag = p, now, False
            elif now - last_move_t > a.stall_s:
                if not stalled_flag:
                    stalled_flag = True
                    stall_events += 1
                    self.event(mission, leg, "stall", f"no motion for {a.stall_s:.0f} s")
                if now - last_move_t > a.stall_fail_s:
                    if resends < a.retries and got_path:
                        resends += 1
                        if w.controller_active():
                            self.cancel_follow_path()
                            w.wait_for(lambda: not w.controller_active(), 5.0)
                        w.send_goal(a.goal_topic, *goal)
                        self.event(mission, leg, "goal_resent", "after stall")
                        last_move_t, stalled_flag = time.time(), False
                    else:
                        reason = "stalled" if got_path else "no_path"
                        break
        if not success:
            self.event(mission, leg, "leg_failed", reason)
            if w.controller_active():
                self.cancel_follow_path()
                self.event(mission, leg, "runner_cancelled_follow_path")
        for name, _, _ in boxes:
            if self.gz.remove_verified(name):
                self.event(mission, leg, "box_removed", name)
            else:
                self.event(mission, leg, "box_remove_failed", name)
        fin = w.pose()
        final_dist = math.hypot(fin[0] - goal[0], fin[1] - goal[1]) if fin else float("nan")
        wdot, flips, rev, ncmd = smoothness([c for c in w.cmd_log[n_cmd0:] if c[0] >= sim0])
        res = {"label": a.label, "mission": mission, "strategy": strategy, "n_items": n_items,
               "n_stops": n_stops, "leg": leg, "stop_id": stop.get("id", "home"),
               "goal_x": round(goal[0], 3), "goal_y": round(goal[1], 3), "planned_len": planned_len,
               "paths_received": paths, "driven_len_true": round(driven_t, 4) if prev_t else "",
               "driven_len_map": round(driven_m, 4), "time_s": round(time.time() - t0, 2),
               "success": int(success), "reason": reason, "reached_msg": reached_msg,
               "final_dist": round(final_dist, 3), "ctrl_aborts": ctrl_aborts, "resends": resends,
               "stall_events": stall_events, "n_obstacles": len(boxes),
               "min_obstacle_clearance": round(min_clear, 3) if boxes and math.isfinite(min_clear) else "",
               "min_static_clearance": round(min_static, 3) if math.isfinite(min_static) else "",
               "max_pos_err": round(max_err, 4) if max_err else "",
               "w_sign_changes": flips, "mean_abs_wdot": round(wdot, 4) if wdot is not None else "",
               "reverse_s": round(rev, 2), "idle_wait_s": round(idle_wait, 2), "cmd_msgs": ncmd}
        self.legs.row(res)   # start legs are logged as leg -1 and left out of the A2 tables
        status = "OK " if success else "XX "
        print(f"   {status}leg {leg} -> {res['stop_id']}: {res['time_s']} s, path {planned_len} m, "
              f"driven {res['driven_len_true'] or res['driven_len_map']} m, {reason}"
              + (f", {len(boxes)} box(es), clearance {res['min_obstacle_clearance']} m" if boxes else "")
              + (f", aborts {ctrl_aborts}, resends {resends}" if ctrl_aborts or resends else ""))
        w.spin_for(a.dwell)
        # drop old command history so memory stays small on long runs
        if len(w.cmd_log) > 20000:
            del w.cmd_log[:-2000]
        return res


def build_plan(a, items, stops, home):
    plan = []
    if a.pairs:
        for pid, s, g in make_pairs(stops, a.pairs, seed=a.seed, min_dist=a.pair_min_dist, home=home[:2]):
            plan.append({"kind": "pair", "list_id": pid, "start": s, "goal": g, "strategy": "pair",
                         "n_items": 0})
        return plan
    strategies = [s.strip() for s in a.strategies.split(",") if s.strip()]
    for s in strategies:
        if s not in STRATEGIES:
            sys.exit(f"unknown strategy {s}")
    if a.lists:
        lo, hi = parse_range(a.size_range)
        for lid, n, names in make_random_size_lists(items, a.lists, lo, hi, seed=a.seed):
            for s in strategies:
                plan.append({"kind": "list", "list_id": lid, "names": names, "strategy": s, "n_items": n})
    else:
        sizes = [int(s) for s in a.sizes.split(",")]
        lists = make_lists(items, sizes, a.per_size, seed=a.seed)
        for n in sizes:
            for li, names in enumerate(lists[n]):
                for s in strategies:
                    plan.append({"kind": "list", "list_id": f"s{a.seed}_n{n}_{li}", "names": names,
                                 "strategy": s, "n_items": n})
    return plan


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True, help="configuration name, e.g. teb_icp (tokens icp/wheel/teb/rpp are checked)")
    ap.add_argument("--out", default="results")
    ap.add_argument("--items", default=None)
    ap.add_argument("--world", default="arc_store")
    ap.add_argument("--model", default="arc")
    ap.add_argument("--strategies", default="as_entered",
                    help=f"comma list from {','.join(STRATEGIES)}; each list is run once per strategy. "
                         "as_entered is what shopping_manager.py does today")
    ap.add_argument("--sizes", default="3,5,8", help="fixed list sizes (items), with --per-size")
    ap.add_argument("--per-size", type=int, default=3)
    ap.add_argument("--lists", type=int, default=0, help="N lists with random sizes from --size-range (A2, B1)")
    ap.add_argument("--size-range", default="3-12")
    ap.add_argument("--pairs", type=int, default=0, help="N fixed start-goal pairs instead of lists (A3, A4)")
    ap.add_argument("--pair-min-dist", type=float, default=2.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--home", default="0,0,0", help="x,y,yaw the robot returns to before each mission")
    ap.add_argument("--obstacles", default="0", help="boxes per leg: N or LO-HI random per run (A4)")
    ap.add_argument("--goal-snap-m", type=float, default=1.0,
                    help="a path or /navigation_goal_reached counts for a goal only within this distance "
                         "(the planner snaps goals up to 20 cells = 1.0 m)")
    ap.add_argument("--box-size", type=float, default=0.2)
    ap.add_argument("--box-offset", type=float, default=0.15, help="sideways offset from the path (m)")
    ap.add_argument("--robot-radius", type=float, default=0.105)
    ap.add_argument("--inflation", type=float, default=0.15, help="radius used for the runner's own cost matrix")
    ap.add_argument("--tol", type=float, default=0.35, help="fallback arrival tolerance (m)")
    ap.add_argument("--speed", type=float, default=0.10, help="expected speed for timeouts (LEAD cap 0.10 m/s)")
    ap.add_argument("--retries", type=int, default=1, help="goal re-sends per leg after an abort or stall (0 = none)")
    ap.add_argument("--idle-wait", type=float, default=20.0, help="max wait for FollowPath to finish before a goal")
    ap.add_argument("--dwell", type=float, default=2.0, help="pause at each stop (s)")
    ap.add_argument("--leg-timeout", type=float, default=0.0, help="0 = automatic from path length and --speed")
    ap.add_argument("--stall-s", type=float, default=8.0)
    ap.add_argument("--stall-fail-s", type=float, default=40.0)
    ap.add_argument("--rate", type=float, default=5.0, help="sampling rate (Hz); A1 is reported every 2 s")
    ap.add_argument("--truth", choices=["auto", "bridge", "gzcli"], default="auto")
    ap.add_argument("--goal-topic", default=GOAL_TOPIC)
    ap.add_argument("--path-topic", default="/planned_path")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--map-topic", default="/rtabmap/map")
    ap.add_argument("--start-at", type=int, default=0, help="skip missions before this index (resume)")
    ap.add_argument("--no-label-check", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="print the plan and exit")
    ap.add_argument("--loss-abort-m", type=float, default=1.0,
                    help="stop the run when the map pose stays this far from truth (0 = never)")
    ap.add_argument("--loss-abort-s", type=float, default=10.0)
    ap.add_argument("--link-abort-s", type=float, default=15.0,
                    help="stop the run when /clock stops advancing for this long (Gazebo link lost)")
    a = ap.parse_args()
    exit_code = [0]

    a.items = a.items or find_item_file()
    if not a.items:
        sys.exit("item_locations.yaml not found; pass --items PATH")
    items = load_items(a.items)
    stops = group_stops(items)
    hx, hy, hyaw = (float(v) for v in a.home.split(","))
    home = {"id": "home", "x": hx, "y": hy, "yaw": hyaw}
    obs_lo, obs_hi = parse_range(a.obstacles)
    box_rng = random.Random(a.seed * 1000 + 17)
    plan = build_plan(a, items, stops, (hx, hy, hyaw))
    for m in plan:
        m["n_boxes"] = box_rng.randint(obs_lo, obs_hi) if obs_hi > 0 else 0
    print(f"{len(items)} items / {len(stops)} driving stops from {a.items}")
    print(f"{len(plan)} missions ({'pairs' if a.pairs else 'lists'}), label {a.label}")

    rclpy.init()
    w = Watcher(model=a.model, path_topic=a.path_topic, cmd_topic=a.cmd_topic, map_topic=a.map_topic,
                truth=a.truth)
    bridge = TruthBridge(a.world) if a.truth != "gzcli" else None
    if bridge:
        bridge.start()
    gz = Gz(a.world)
    try:
        w.spin_for(3.0)
        gt, owner = find_goal_topic(w, a.goal_topic)
        print(f"goal topic: {a.goal_topic}" + (f" (planner {owner} subscribed)" if owner else
                                                " - WARNING: no dijkstra planner subscribed yet"))
        print(f"/cmd_vel type: {w.cmd_type.__name__}")
        cfg = stack_config(w)
        print(f"stack: odometry={cfg['odometry']} ({cfg['rtabmap_odom_topic']}), controller={cfg['controller']} "
              f"({cfg['controller_plugin']})")
        if cfg["shopping_manager_running"]:
            print("WARNING: shopping_manager is running. Close tab B7 so it does not send its own goals.")
        bad = label_mismatch(a.label, cfg)
        if bad and not a.dry_run and not a.no_label_check:
            sys.exit(f"STOP: {bad}. Fix the stack or the label (or pass --no-label-check).")
        if not w.wait_for(lambda: w.map_msg is not None, 15):
            sys.exit("No map on " + a.map_topic)
        grid = grid_from_msg(w.map_msg, a.inflation)
        os.makedirs(a.out, exist_ok=True)
        grid.save_npz(os.path.join(a.out, "map.npz"))
        occ = grid.occupied_points()
        if not w.wait_for(lambda: w.map_pose() is not None, 10):
            sys.exit("No TF map -> base_footprint")
        if w.wait_truth(8.0):
            print(f"true pose source: {w.fresh_truth()[5]}")
        else:
            print("WARNING: no Gazebo true pose; localisation error (A1) and clearances will be blank.")
        w.wait_for(lambda: w.mode() is not None, 3.0)
        if w.mode() not in ("LEAD", "FOLLOW"):
            print(f"WARNING: BT mode is {w.mode()}; the motion gate only moves in LEAD/FOLLOW (B6 -> 3).")

        for m in plan:
            if m["kind"] == "pair":
                pts = [(m["start"]["x"], m["start"]["y"]), (m["goal"]["x"], m["goal"]["y"])]
                D, E, _ = cost_matrix(grid, pts)
                m["stops"], m["order"] = [m["goal"]], f"{m['start']['id']}->{m['goal']['id']}"
                m["J"] = m["J_opt"] = float(D[0, 1])
                m["n_stops"] = 1
                continue
            need = stops_for_list(m["names"], items, stops)
            pts = [(hx, hy)] + [(s["x"], s["y"]) for s in need]
            D, E, _ = cost_matrix(grid, pts)
            n = len(need)
            order = order_for(m["strategy"], D, E, n)
            m["stops"] = [need[i - 1] for i in order]
            m["order"] = " ".join(need[i - 1]["id"] for i in order)
            m["J"] = route_cost(D, order)
            m["J_opt"] = route_cost(D, order_for("exact", D, E, n))
            m["n_stops"] = n
        meta = {"label": a.label, "args": vars(a), "stack": cfg, "cmd_type": w.cmd_type.__name__,
                "truth_source": (w.fresh_truth() or [None] * 6)[5],
                "plan": [{k: v for k, v in m.items() if k not in ("stops", "start", "goal")} for m in plan],
                "started": time.strftime("%Y-%m-%d %H:%M:%S")}
        with open(os.path.join(a.out, f"run_{a.label}_{time.strftime('%Y%m%d_%H%M%S')}.json"), "w") as f:
            json.dump(meta, f, indent=2, default=float)
        if a.dry_run:
            for i, m in enumerate(plan):
                print(f"{i:3d} {m['list_id']:14s} {m['strategy']:12s} stops {m['n_stops']:2d} boxes {m['n_boxes']} "
                      f"J={m['J']:.2f} m (opt {m['J_opt']:.2f})  order: {m['order']}")
            return

        r = Runner(a, w, gz, occ)
        lost = False
        stop_reasons = ("localisation_lost", "sim_link_lost", "boxes_not_removed")

        def world_clean(mid, leg):
            """No test box may be left in Gazebo when a leg starts (A4, 15 Oct: 12 were)."""
            if not gz.ok:
                return True
            before = gz.test_boxes() or []
            if not before:
                return True
            stuck = gz.sweep_test_boxes()
            r.event(mid, leg, "stray_boxes_removed", f"{len(before) - len(stuck)} of {len(before)}: {' '.join(before)}")
            print(f"   removed {len(before) - len(stuck)} leftover test box(es) before leg {leg}")
            if stuck:
                r.event(mid, leg, "boxes_not_removed", " ".join(stuck))
                print(f"   could not remove: {' '.join(stuck)}")
                return False
            return True

        if not world_clean("setup", -1):
            sys.exit("STOP: test boxes are still in Gazebo and cannot be removed. Restart Gazebo (./bringup.sh all).")
        for i, m in enumerate(plan):
            if i < a.start_at:
                continue
            mid = f"{a.label}_s{a.seed}_{i:03d}"
            print(f"\n[{i + 1}/{len(plan)}] {m['list_id']} {m['strategy']}: {m['n_stops']} stop(s), "
                  f"planned J={m['J']:.2f} m (optimum {m['J_opt']:.2f})")
            start = m["start"] if m["kind"] == "pair" else home
            print(f"   going to start {start['id']} ...")
            r.event(mid, -1, "mission_start", m["list_id"])
            if not world_clean(mid, -1):
                lost = True
                break
            pre = r.run_leg(mid, m["strategy"], -1, start, m["n_items"], m["n_stops"], log=False)
            if pre["reason"] in stop_reasons:
                lost = True
                break
            if not pre["success"]:
                r.event(mid, -1, "start_not_reached", pre["reason"])
            t_start = time.time()
            ok = driven = rec = coll = nbox = 0
            for leg, stop in enumerate(m["stops"]):
                if not world_clean(mid, leg):
                    lost = True
                    break
                res = r.run_leg(mid, m["strategy"], leg, stop, m["n_items"], m["n_stops"], n_boxes=m["n_boxes"])
                if res["reason"] in stop_reasons:
                    lost = True
                ok += res["success"]
                driven += res["driven_len_true"] or res["driven_len_map"] or 0
                rec += res["ctrl_aborts"] + res["resends"]
                nbox += res["n_obstacles"]
                cl = [res["min_obstacle_clearance"], res["min_static_clearance"]]
                coll += int(any(c != "" and c <= 0 for c in cl))
                if lost:
                    break
            r.missions.row({"label": a.label, "mission": mid, "list_id": m["list_id"],
                            "strategy": m["strategy"], "n_items": m["n_items"], "n_stops": m["n_stops"],
                            "order": m["order"], "planned_cost_J": round(m["J"], 4),
                            "exact_cost_J": round(m["J_opt"], 4), "driven_total": round(driven, 4),
                            "time_total": round(time.time() - t_start, 2), "legs_ok": ok,
                            "legs_total": len(m["stops"]), "completed": int(ok == len(m["stops"])),
                            "recoveries": rec, "collisions": coll, "n_obstacles": nbox})
            r.event(mid, -1, "mission_end", f"{ok}/{len(m['stops'])} legs")
            if lost:
                break
        r.close()
        if lost:
            print(f"\nSTOPPED: localisation lost, Gazebo link lost or test boxes stuck (map pose > {a.loss_abort_m} m "
                  f"from truth for {a.loss_abort_s:.0f} s, /clock frozen {a.link_abort_s:.0f} s, or a box Gazebo would "
                  "not remove; see events.csv). "
                  "Rows so far are kept. Restart the stack (re-map) before resuming with --start-at.")
            exit_code[0] = 3
        else:
            print(f"\nDone. Results in {a.out}/ - ./run.sh report")
    except KeyboardInterrupt:
        print("\nInterrupted. Rows written so far are kept; resume with --start-at N (./run.sh status).")
    except RuntimeError:
        if rclpy.ok():
            raise
        # Ctrl-C landed inside rclpy's message conversion; same as an interrupt
        print("\nInterrupted. Rows written so far are kept; resume with --start-at N (./run.sh status).")
    finally:
        if bridge:
            bridge.stop()
        if w.poller:
            w.poller.stop()
        try:
            w.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code[0])


if __name__ == "__main__":
    main()
