"""Offline tests for the ARC test kit (no ROS needed).

    cd ~/arc_test_kit && python3 -m unittest discover -s tests -v
"""
import csv
import itertools
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
import ros_stubs  # noqa: E402

ros_stubs.install()
import numpy as np  # noqa: E402

from arc_tests import common  # noqa: E402
from arc_tests import mission_runner as mr  # noqa: E402
from arc_tests import rosio  # noqa: E402

ITEMS = os.path.join(HERE, "data", "item_locations_aisles.yaml")


class CommonTests(unittest.TestCase):
    def test_exact_matches_brute_force(self):
        rng = random.Random(3)
        for _ in range(25):
            n = rng.randint(1, 6)
            D = np.array([[0 if i == j else rng.uniform(0.5, 9) for j in range(n + 1)] for i in range(n + 1)])
            best = min(common.route_cost(D, p) for p in itertools.permutations(range(1, n + 1)))
            self.assertAlmostEqual(common.route_cost(D, common.order_exact(D, n)), best, places=9)

    def test_items_and_stops(self):
        items = common.load_items(ITEMS)
        stops = common.group_stops(items)
        self.assertEqual(len(items), 29)
        # 13 named stops, 9 distinct spots (both faces of an aisle share the spot)
        self.assertEqual(len(stops), 9)
        self.assertEqual(sum(len(s["items"]) for s in stops), 29)
        for a, b in itertools.combinations(stops, 2):
            self.assertGreater(math.hypot(a["x"] - b["x"], a["y"] - b["y"]), 0.1)

    def test_random_lists_prefix_stable(self):
        items = common.load_items(ITEMS)
        l30 = common.make_random_size_lists(items, 30, 3, 12, seed=7)
        l10 = common.make_random_size_lists(items, 10, 3, 12, seed=7)
        self.assertEqual(l30[:10], l10)            # B1 wheel runs == first 10 A2 lists
        self.assertTrue(all(3 <= n <= 12 and len(names) == n for _, n, names in l30))
        self.assertEqual(len({lid for lid, _, _ in l30}), 30)

    def test_pairs(self):
        stops = common.group_stops(common.load_items(ITEMS))
        pairs = common.make_pairs(stops, 20, seed=5, min_dist=2.0)
        self.assertEqual(len(pairs), 20)
        for _, s, g in pairs:
            self.assertGreaterEqual(math.hypot(s["x"] - g["x"], s["y"] - g["y"]), 2.0)
        self.assertEqual(pairs, common.make_pairs(stops, 20, seed=5, min_dist=2.0))

    def test_parse_range(self):
        self.assertEqual(common.parse_range("2"), (2, 2))
        self.assertEqual(common.parse_range("1-3"), (1, 3))


class RosioPureTests(unittest.TestCase):
    def test_parse_gz_model(self):
        out = """Requesting state for world [arc_store]...
Model: [8]
  - Name: arc
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
              [1.250000 -0.500000 0.010000]
              [0.000000 0.000000 1.570000]
"""
        self.assertEqual(rosio.parse_gz_model_pose(out), (1.25, -0.5, 1.57))

    def test_label_check(self):
        cfg = {"odometry": "wheel", "controller": "teb"}
        self.assertIsNotNone(rosio.label_mismatch("teb_icp", cfg))
        self.assertIsNone(rosio.label_mismatch("teb_wheel", cfg))
        self.assertIsNotNone(rosio.label_mismatch("rpp_icp_pairs", {"odometry": "icp", "controller": "teb"}))
        self.assertIsNone(rosio.label_mismatch("smoke", cfg))

    def test_status_counts_new_aborts_only(self):
        w = types.SimpleNamespace(ctrl_status={}, ctrl_aborts=[], ctrl_succeeded=[], ctrl_status_seen=False,
                                  _t_created=time.time())

        def st(uid, s):
            return types.SimpleNamespace(status=s, goal_info=types.SimpleNamespace(
                goal_id=types.SimpleNamespace(uuid=[uid] * 16)))
        on = rosio.Watcher._on_status
        on(w, types.SimpleNamespace(status_list=[st(1, 6)]))           # latched old abort on connect
        self.assertEqual(len(w.ctrl_aborts), 0)
        on(w, types.SimpleNamespace(status_list=[st(1, 6), st(2, 2)]))
        self.assertTrue(rosio.Watcher.controller_active(w))
        on(w, types.SimpleNamespace(status_list=[st(1, 6), st(2, 6)]))
        self.assertEqual(len(w.ctrl_aborts), 1)
        on(w, types.SimpleNamespace(status_list=[st(1, 6), st(2, 6)]))  # repeat: no double count
        self.assertEqual(len(w.ctrl_aborts), 1)
        self.assertFalse(rosio.Watcher.controller_active(w))


class GzTruthTimingTests(unittest.TestCase):
    def test_gzcli_truth_is_stamped_at_midpoint(self):
        now = time.time()
        w = types.SimpleNamespace(truth=None, sim_now=lambda: 100.0,
                                  poller=types.SimpleNamespace(latest=(1.0, 2.0, 0.5, now - 0.6, now - 0.2)))
        tr = rosio.Watcher.fresh_truth(w)
        self.assertEqual(tr[5], "gzcli")
        self.assertAlmostEqual(tr[4], 100.0 - 0.4, delta=0.02)   # midpoint was 0.4 s ago


class RunnerMathTests(unittest.TestCase):
    def test_smoothness(self):
        log = [(0.0, 0.1, 0.0), (0.1, 0.1, 0.5), (0.2, 0.1, -0.5), (0.3, -0.05, -0.5), (5.0, 0.1, 0.5)]
        wdot, flips, rev, n = mr.smoothness(log)
        self.assertAlmostEqual(wdot, (5 + 10 + 0) / 3)   # the 4.7 s gap is excluded
        self.assertEqual(flips, 2)
        self.assertEqual(n, 5)

    def test_static_clearance(self):
        occ = np.array([[1.0, 0.0], [5.0, 5.0]])
        self.assertAlmostEqual(mr.static_clearance(occ, 0.5, 0.0, 0.105), 0.395)
        self.assertIsNone(mr.static_clearance(occ, -3, -3, 0.105))

    def test_presets_match_plan(self):
        items = common.load_items(ITEMS)
        stops = common.group_stops(items)

        def plan(**kw):
            a = types.SimpleNamespace(pairs=0, lists=0, size_range="3-12", sizes="3,5,8", per_size=3,
                                      strategies="as_entered", seed=7, pair_min_dist=2.0)
            a.__dict__.update(kw)
            return mr.build_plan(a, items, stops, (0, 0, 0))
        self.assertEqual(len(plan(lists=30)), 30)                                       # A2
        self.assertEqual(len(plan(lists=10)), 10)                                       # B1
        self.assertEqual(len(plan(pairs=20, seed=5)), 20)                               # A3, A4
        c2 = plan(sizes="5,8,12", per_size=10, strategies="as_entered,nn_dijkstra,exact", seed=21)
        self.assertEqual(len(c2), 90)                                                   # C2
        self.assertEqual({m["n_items"] for m in c2}, {5, 8, 12})


# ---------------------------------------------------------------------------
# A fake ARC stack that behaves like planned_path_to_teb + controller_server
# ---------------------------------------------------------------------------
class FakeStack:
    """Robot drives straight to the goal of the FollowPath that is running.

    Like ARC: a path arrives ~0.1 s after a goal; new paths are ignored while a
    FollowPath runs; success publishes /navigation_goal_reached.
    """

    def __init__(self, abort_once=False, no_path=False, busy_far=False, speed=4.0, map_offset=0.02,
                 stale_once=False):
        self.stale_once = stale_once
        self.t0 = self._last = time.time()
        self.x = self.y = self.yaw = 0.0
        self.speed, self.abort_once, self.no_path = speed, abort_once, no_path
        self.active, self.follow, self.goal, self.path_due = False, None, None, None
        self.last_path, self.cmd, self.cmd_log = None, (0.0, 0.0), []
        self.ctrl_aborts, self.reached_times, self.goals = [], [], 0
        self.travel = 0.0
        self.map_offset = map_offset
        if busy_far:
            self.active, self.follow = True, (50.0, 0.0)
        self.poller = None

    # ---- what the runner calls
    def sim_now(self):
        return time.time() - self.t0

    def send_goal(self, topic, x, y, yaw):
        self.goals += 1
        self.goal = (x, y)
        if self.stale_once:
            # 15 Oct race: a re-plan to the OLD goal (where the robot already is) lands first, the bridge
            # runs it and reports "reached"; the real path arrives while it runs and is dropped.
            self.stale_once = False
            now = time.time()
            self.last_path = (now, 0.07, [(self.x, self.y), (self.x + 0.05, self.y + 0.05)])
            self.active, self.follow = True, (self.x, self.y)
            return
        if not self.no_path:
            self.path_due = time.time() + 0.1

    def controller_active(self):
        return self.active

    def fresh_truth(self, max_age=2.0):
        return (self.x, self.y, self.yaw, time.time(), None, "fake")

    def map_pose(self, stamp_s=None):
        return (self.x + self.map_offset, self.y, self.yaw)

    def tf_layers(self):
        return {"corr": (0.0, 0.0, 0.0), "lidar": (self.x + self.map_offset, self.y, self.yaw),
                "wheel": (self.x + 0.5, self.y, self.yaw)}

    def pose(self):
        return (self.x, self.y, self.yaw)

    def cmd_now(self):
        return self.cmd

    def mode(self):
        return "LEAD"

    def create_client(self, *a, **k):
        stack = self

        class Fut:
            def done(self):
                return True

        class Cli:
            def wait_for_service(self, timeout_sec=0):
                return True

            def call_async(self, req):
                stack.active = False
                stack.cmd = (0.0, 0.0)
                return Fut()
        return Cli()

    def wait_for(self, pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self._tick()
            if pred():
                return True
            time.sleep(0.005)
        return False

    def spin_for(self, s):
        self.wait_for(lambda: False, s)

    # ---- simulation
    def _tick(self):
        now = time.time()
        dt, self._last = now - self._last, now
        if self.path_due and now >= self.path_due:
            self.path_due = None
            pts = [(self.x, self.y), self.goal]
            self.last_path = (now, math.dist(*pts), [(self.x + (self.goal[0] - self.x) * k / 10,
                                                      self.y + (self.goal[1] - self.y) * k / 10) for k in range(11)])
            if not self.active:
                self.active, self.follow = True, self.goal
        if self.active:
            dx, dy = self.follow[0] - self.x, self.follow[1] - self.y
            d = math.hypot(dx, dy)
            step = min(d, self.speed * dt)
            if d > 1e-9:
                self.x += dx / d * step
                self.y += dy / d * step
                self.travel += step
            self.cmd = (0.1, 0.3 if int(now * 10) % 2 else -0.3)
            self.cmd_log.append((self.sim_now(), *self.cmd))
            if self.abort_once and self.travel > 0.5:
                self.abort_once, self.active, self.cmd = False, False, (0.0, 0.0)
                self.ctrl_aborts.append(now)
            elif math.hypot(self.follow[0] - self.x, self.follow[1] - self.y) < 0.02:
                self.active, self.cmd = False, (0.0, 0.0)
                self.reached_times.append(now)


def runner_args(out, **kw):
    a = types.SimpleNamespace(label="teb_icp", out=out, rate=20.0, tol=0.35, stall_s=0.5, stall_fail_s=1.5,
                              retries=1, idle_wait=1.0, dwell=0.0, leg_timeout=8.0, speed=0.1,
                              goal_topic="/goal_pose", box_size=0.2, box_offset=0.15, robot_radius=0.105,
                              loss_abort_m=1.0, loss_abort_s=10.0, link_abort_s=15.0, goal_snap_m=1.0)
    a.__dict__.update(kw)
    return a


class RunnerLegTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        sys.modules["rclpy"].spin_once = lambda node, timeout_sec=0.0: (node._tick(), time.sleep(0.002))

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def leg(self, stack, **kw):
        r = mr.Runner(runner_args(self.tmp, **kw), stack, types.SimpleNamespace(ok=False),
                      np.array([[10.0, 10.0]]))
        try:
            return r.run_leg("m0", "as_entered", 0, {"id": "s1", "x": 3.0, "y": 0.0}, 3, 1)
        finally:
            r.close()

    def events(self):
        with open(os.path.join(self.tmp, "events.csv")) as f:
            return [r["event"] for r in csv.DictReader(f)]

    def test_stale_path_and_far_reached_are_not_arrival(self):
        st = FakeStack(stale_once=True)
        res = self.leg(st)
        ev = self.events()
        self.assertIn("reached_msg_far", ev)
        self.assertEqual((res["success"], res["reason"]), (1, "arrived"))
        self.assertGreaterEqual(res["resends"], 1)
        self.assertAlmostEqual(st.x, 3.0, delta=0.05)          # it really drove there
        self.assertGreater(res["planned_len"], 1.0)            # the 0.07 m stale path was not taken

    def test_arrival_by_arc_signal(self):
        res = self.leg(FakeStack())
        self.assertEqual((res["success"], res["reason"], res["reached_msg"]), (1, "arrived", 1))
        self.assertAlmostEqual(res["driven_len_true"], 3.0, delta=0.25)
        self.assertGreater(res["cmd_msgs"], 5)
        self.assertNotEqual(res["mean_abs_wdot"], "")
        self.assertIn("goal_reached_msg", self.events())
        with open(os.path.join(self.tmp, "samples.csv")) as f:
            s = list(csv.DictReader(f))
        self.assertTrue(s and abs(float(s[0]["pos_err"]) - 0.02) < 1e-6)

    def test_abort_then_resend_recovers(self):
        st = FakeStack(abort_once=True)
        res = self.leg(st)
        self.assertEqual((res["success"], res["ctrl_aborts"], res["resends"]), (1, 1, 1))
        self.assertEqual(st.goals, 2)
        ev = self.events()
        self.assertIn("controller_aborted", ev)
        self.assertIn("goal_resent", ev)

    def test_stops_on_localisation_loss(self):
        res = self.leg(FakeStack(speed=0.05, map_offset=2.0), loss_abort_s=0.3, leg_timeout=5.0)
        self.assertEqual((res["success"], res["reason"]), (0, "localisation_lost"))
        self.assertIn("localisation_lost", self.events())
        with open(os.path.join(self.tmp, "samples.csv")) as f:
            s = list(csv.DictReader(f))
        self.assertAlmostEqual(float(s[0]["lidar_err"]), 2.0, places=3)
        self.assertAlmostEqual(float(s[0]["wheel_err"]), 0.5, places=3)

    def test_stops_when_clock_freezes(self):
        st = FakeStack(speed=0.05)
        st.sim_now = lambda: 42.0                       # /clock stopped
        res = self.leg(st, link_abort_s=0.4, leg_timeout=5.0)
        self.assertEqual(res["reason"], "sim_link_lost")
        self.assertIn("sim_link_lost", self.events())

    def test_truth_gap_does_not_stop(self):
        st = FakeStack()
        st.fresh_truth = lambda max_age=2.0: None        # gz service not answering
        res = self.leg(st, link_abort_s=0.4)
        self.assertEqual(res["success"], 1)

    def test_start_legs_are_logged(self):
        st = FakeStack()
        r = mr.Runner(runner_args(self.tmp), st, types.SimpleNamespace(ok=False), np.array([[10.0, 10.0]]))
        r.run_leg("m0", "as_entered", -1, {"id": "home", "x": 1.0, "y": 0.0}, 3, 1, log=False)
        r.close()
        with open(os.path.join(self.tmp, "legs.csv")) as f:
            self.assertEqual([l["leg"] for l in csv.DictReader(f)], ["-1"])

    def test_abort_without_retries_fails(self):
        res = self.leg(FakeStack(abort_once=True), retries=0)
        self.assertEqual((res["success"], res["reason"]), (0, "controller_aborted"))

    def test_no_path(self):
        res = self.leg(FakeStack(no_path=True), retries=0, leg_timeout=20.0)
        self.assertEqual((res["success"], res["reason"]), (0, "no_path"))
        self.assertIn("goal_resent_no_path", self.events())

    def test_waits_for_busy_controller_then_cancels(self):
        st = FakeStack(busy_far=True)
        res = self.leg(st)
        ev = self.events()
        self.assertIn("controller_still_busy", ev)
        self.assertEqual(res["success"], 1)
        self.assertGreaterEqual(res["idle_wait_s"], 0.9)


class AnalyzeTests(unittest.TestCase):
    def test_report_from_synthetic_csvs(self):
        tmp = tempfile.mkdtemp()
        try:
            def write(name, cols, rows):
                with open(os.path.join(tmp, name), "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=cols)
                    w.writeheader()
                    for r in rows:
                        w.writerow({k: r.get(k, "") for k in cols})
            samples, legs, missions = [], [], []
            for lab, err in (("teb_icp", 0.05), ("teb_wheel", 0.4)):
                for k in range(6):
                    mid = f"{lab}_{k}"
                    for i in range(50):                          # 5 Hz for 10 s
                        e = err if not (lab == "teb_wheel" and k == 0 and 20 <= i < 30) else 1.6
                        samples.append({"label": lab, "mission": mid, "strategy": "as_entered", "leg": 0,
                                        "sim_t": f"{i * 0.2:.2f}", "pos_err": e, "yaw_err_deg": 1.0,
                                        "truth_src": "bridge"})
                    legs.append({"label": lab, "mission": mid, "strategy": "as_entered", "leg": 0, "success": 1,
                                 "reason": "arrived", "driven_len_true": 3 + k, "mean_abs_wdot": 0.5 + k / 10,
                                 "w_sign_changes": 2, "resends": 0, "ctrl_aborts": 0, "n_obstacles": 0})
                    missions.append({"label": lab, "mission": mid, "list_id": f"L{k}", "strategy": "as_entered",
                                     "completed": 1, "time_total": 60 + k * (2 if lab == "teb_wheel" else 1),
                                     "driven_total": 3 + k, "planned_cost_J": 3 + k, "recoveries": 0,
                                     "collisions": 0})
            write("samples.csv", mr.SAMPLE_COLS, samples)
            write("legs.csv", mr.LEG_COLS, legs)
            write("missions.csv", mr.MISSION_COLS, missions)
            subprocess.run([sys.executable, os.path.join(ROOT, "arc_tests", "analyze.py"), tmp,
                            "--compare", "teb_icp", "teb_wheel"], check=True, capture_output=True)
            with open(os.path.join(tmp, "report", "results.md")) as fh:
                md = fh.read()
            self.assertIn("## A1", md)
            self.assertIn("| teb_icp | 30 | 300 |", md)          # 2 s thinning: 5 per mission x 6
            self.assertIn("## Paired comparison: teb_icp vs teb_wheel (6 shared missions)", md)
            self.assertIn("| Localisation-loss episodes | 0 | 1 | - |", md)
            self.assertIn("## A2", md)
        finally:
            shutil.rmtree(tmp)


class GzBoxTests(unittest.TestCase):
    """Box removal must be confirmed by `gz model --list`, not by the service reply."""

    def _fake(self, models, sticky=()):
        state = {"models": list(models), "removes": 0}

        def run(cmd, capture_output=True, text=True, timeout=None):
            if cmd[:3] == ["gz", "model", "--list"]:
                out = "Available models:\n" + "".join(f"    - {m}\n" for m in state["models"])
                return types.SimpleNamespace(stdout=out, returncode=0)
            if len(cmd) > 3 and "remove" in cmd[3]:
                state["removes"] += 1
                name = cmd[-1].split('"')[1]
                if name not in sticky:
                    state["models"] = [m for m in state["models"] if m != name]
                return types.SimpleNamespace(stdout="data: true", returncode=0)  # Gazebo always says true
            raise AssertionError(cmd)
        return state, run

    def _gz(self):
        g = rosio.Gz("arc_store")
        g.ok = True
        return g

    def test_sweep_removes_leftovers_and_reports_stuck(self):
        state, run = self._fake(["ground_plane", "arc", "arc_test_box_3", "arc_test_box_7"], sticky=("arc_test_box_7",))
        old_run, old_sleep = rosio.subprocess.run, rosio.time.sleep
        rosio.subprocess.run, rosio.time.sleep = run, lambda s: None
        try:
            g = self._gz()
            self.assertEqual(g.test_boxes(), ["arc_test_box_3", "arc_test_box_7"])
            self.assertEqual(g.sweep_test_boxes(), ["arc_test_box_7"])   # service said true; list says still there
            self.assertEqual(g.test_boxes(), ["arc_test_box_7"])
            self.assertNotIn("arc", g.test_boxes())
        finally:
            rosio.subprocess.run, rosio.time.sleep = old_run, old_sleep

    def test_remove_verified_retries(self):
        calls = {"n": 0}
        state, run = self._fake(["arc", "arc_test_box_1"])

        def flaky(cmd, **kw):          # first removal is lost, second works
            if len(cmd) > 3 and "remove" in cmd[3]:
                calls["n"] += 1
                if calls["n"] == 1:
                    return types.SimpleNamespace(stdout="data: true", returncode=0)
            return run(cmd, **kw)
        old_run, old_sleep = rosio.subprocess.run, rosio.time.sleep
        rosio.subprocess.run, rosio.time.sleep = flaky, lambda s: None
        try:
            self.assertTrue(self._gz().remove_verified("arc_test_box_1"))
            self.assertEqual(calls["n"], 2)
        finally:
            rosio.subprocess.run, rosio.time.sleep = old_run, old_sleep


class VoidLabelTests(unittest.TestCase):
    def test_rows_move_and_others_stay(self):
        tmp = tempfile.mkdtemp()
        try:
            with open(os.path.join(tmp, "legs.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["label", "mission"])
                w.writerows([["teb_icp_boxes", "a"], ["teb_icp", "b"], ["teb_icp_boxes", "c"]])
            subprocess.run([sys.executable, os.path.join(ROOT, "arc_tests", "void_label.py"), tmp,
                            "teb_icp_boxes", "boxes", "left"], check=True, capture_output=True)
            left = list(csv.DictReader(open(os.path.join(tmp, "legs.csv"))))
            self.assertEqual([r["mission"] for r in left], ["b"])
            vdir = os.path.join(tmp, "void")
            sub = os.path.join(vdir, os.listdir(vdir)[0])
            self.assertEqual(len(list(csv.DictReader(open(os.path.join(sub, "legs.csv"))))), 2)
            self.assertIn("VOID", open(os.path.join(tmp, "LAB_NOTEBOOK.md")).read())
        finally:
            shutil.rmtree(tmp)


class Phase2Tests(unittest.TestCase):
    def test_progress_variant_keeps_teb_and_adds_checker(self):
        import yaml
        tmp = tempfile.mkdtemp()
        try:
            src = os.path.join(tmp, "teb.yaml")
            yaml.safe_dump({"controller_server": {"ros__parameters": {
                "controller_plugins": ["TEB"], "TEB": {"plugin": "teb_local_planner::TebLocalPlannerROS",
                                                       "max_vel_x": 0.15, "max_vel_theta": 0.5}}}}, open(src, "w"))
            out = os.path.join(tmp, "out.yaml")
            subprocess.run([sys.executable, os.path.join(ROOT, "arc_tests", "make_controller_variant.py"),
                            "--in", src, "--out", out, "--type", "progress"], check=True, capture_output=True)
            p = yaml.safe_load(open(out))["controller_server"]["ros__parameters"]
            self.assertEqual(p["TEB"]["plugin"], "teb_local_planner::TebLocalPlannerROS")   # controller untouched
            self.assertEqual(p["TEB"]["max_vel_theta"], 0.5)
            self.assertEqual(p["progress_checker_plugins"], ["progress_checker"])
            self.assertEqual(p["progress_checker"]["required_movement_radius"], 0.25)
            self.assertEqual(p["progress_checker"]["movement_time_allowance"], 20.0)
            self.assertNotIn("progress_checker", yaml.safe_load(open(src))["controller_server"]["ros__parameters"])
        finally:
            shutil.rmtree(tmp)

    def test_label_check_progress(self):
        base = {"odometry": "icp", "controller": "teb"}
        self.assertIsNone(rosio.label_mismatch("teb_icp_pc", dict(base, progress="relaxed", progress_radius=0.25)))
        self.assertIn("pc", rosio.label_mismatch("teb_icp_pc", dict(base, progress="default", progress_radius=0.5)))
        self.assertIn("baseline", rosio.label_mismatch("teb_icp", dict(base, progress="relaxed", progress_radius=0.25)))
        self.assertIsNone(rosio.label_mismatch("teb_icp", dict(base, progress="default", progress_radius=0.5)))
        self.assertIsNone(rosio.label_mismatch("teb_icp_pc", dict(base, progress="unknown", progress_radius=None)))


class FixedEndTests(unittest.TestCase):
    def test_exact_with_end_matches_brute_force(self):
        rng = random.Random(5)
        for _ in range(20):
            n = rng.randint(1, 6)
            D = np.array([[0 if i == j else rng.uniform(0.5, 9) for j in range(n + 2)] for i in range(n + 2)])
            end = n + 1
            best = min(common.route_cost(D, p, end) for p in itertools.permutations(range(1, n + 1)))
            self.assertAlmostEqual(common.route_cost(D, common.order_exact(D, n, end), end), best, places=9)
            # without an end, behaviour unchanged
            best_open = min(common.route_cost(D, p) for p in itertools.permutations(range(1, n + 1)))
            self.assertAlmostEqual(common.route_cost(D, common.order_exact(D, n)), best_open, places=9)


class FootprintTests(unittest.TestCase):
    def test_wider_robot_never_reaches_more_stops(self):
        tmp = tempfile.mkdtemp()
        try:
            W, H, res = 120, 80, 0.05
            data = np.zeros((H, W), dtype=np.int16)
            data[:, 59:61] = 100           # a wall with a 0.6 m gap: a 0.35 m robot cannot pass
            data[34:46, 59:61] = 0
            common.Grid(data.ravel().tolist(), W, H, res, 0.0, 0.0).save_npz(os.path.join(tmp, "map.npz"))
            items = os.path.join(tmp, "items.yaml")
            open(items, "w").write("items:\n  a: {x: 1.0, y: 2.0, yaw: 0.0, stop: a}\n  b: {x: 5.0, y: 2.0, yaw: 0.0, stop: b}\n")
            r = subprocess.run([sys.executable, os.path.join(ROOT, "arc_tests", "footprint_check.py"), "--map",
                                os.path.join(tmp, "map.npz"), "--items", items, "--out", tmp, "--start", "1.0,1.0"],
                               capture_output=True, text=True)
            rows = list(csv.DictReader(open(os.path.join(tmp, "footprint_check.csv"))))
            got = [int(x["stops_reachable"].split("/")[0]) for x in rows]
            self.assertEqual(got, sorted(got, reverse=True), r.stdout + r.stderr)
            self.assertEqual(got[0], 2)          # 0.15 m robot passes the gap
            self.assertLess(got[-1], 2)          # 0.59 m cart cannot
        finally:
            shutil.rmtree(tmp)


if __name__ == "__main__":
    unittest.main()
