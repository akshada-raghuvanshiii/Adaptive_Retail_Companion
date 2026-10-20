"""Offline checks for the generated store (no Gazebo or ROS needed).

    python3 -m unittest discover -s test -v      # from the package root

These check what can be checked without running the simulator: the SDF is
well-formed and has the systems and models we expect, scripted people never
walk through shelves, pick poses face their shelves, aisles are wide enough,
and scenario labels match the timeline. Whether Gazebo loads and renders the
worlds is checked by running it (see README "First run").
"""

import csv
import importlib.util
import math
import py_compile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("gen", ROOT / "scripts" / "generate_store.py")
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

OBST = gen.build_obstacles()
ITEMS = gen.build_items()
ACTORS = gen.scenario_actors(ITEMS)
L = gen.LAYOUT["store"]["length_x"]
W = gen.LAYOUT["store"]["width_y"]


def clearance(x, y):
    best = float("inf")
    for o in OBST:
        dx = max(o["x0"] - x, 0.0, x - o["x1"])
        dy = max(o["y0"] - y, 0.0, y - o["y1"])
        best = min(best, math.hypot(dx, dy))
    return best


class WorldFiles(unittest.TestCase):
    def setUp(self):
        gen.main()  # regenerate so tests always check current layout

    def test_worlds_parse_and_have_systems(self):
        expected_actors = {"arc_store": 0, "arc_store_normal": 1, "arc_store_searching": 1, "arc_store_busy": 5}
        for name, n_actors in expected_actors.items():
            root = ET.parse(ROOT / "worlds" / f"{name}.sdf").getroot()
            self.assertEqual(root.tag, "sdf")
            world = root.find("world")
            self.assertEqual(world.get("name"), name)
            plugins = {p.get("name") for p in world.findall("plugin")}
            for sysname in ("Physics", "SceneBroadcaster", "UserCommands", "Sensors"):
                self.assertIn(f"gz::sim::systems::{sysname}", plugins)
            names = [m.get("name") for m in world.findall("model")] + [a.get("name") for a in world.findall("actor")]
            self.assertEqual(len(names), len(set(names)), "duplicate model/actor names")
            self.assertIn("shopper", names)
            self.assertIn("overhead_cam", names)
            self.assertEqual(len(world.findall("actor")), n_actors)

    def test_actor_waypoints_strictly_increasing(self):
        root = ET.parse(ROOT / "worlds" / "arc_store_busy.sdf").getroot()
        for actor in root.iter("actor"):
            times = [float(w.find("time").text) for w in actor.iter("waypoint")]
            self.assertGreater(len(times), 3)
            self.assertTrue(all(b > a for a, b in zip(times, times[1:])), actor.get("name"))
            traj = actor.find("script/trajectory")
            self.assertEqual(traj.get("tension"), "1")
            first = actor.find("script/trajectory/waypoint/pose").text.split()
            last = list(actor.iter("waypoint"))[-1].find("pose").text.split()
            self.assertEqual(first[:2], last[:2], f"{actor.get('name')} loop would teleport")

    def test_shopper_plugins(self):
        root = ET.parse(ROOT / "worlds" / "arc_store.sdf").getroot()
        shopper = next(m for m in root.iter("model") if m.get("name") == "shopper")
        names = {p.get("name") for p in shopper.findall("plugin")}
        self.assertIn("gz::sim::systems::VelocityControl", names)
        self.assertIn("gz::sim::systems::OdometryPublisher", names)


class Geometry(unittest.TestCase):
    def test_aisles_wide_enough(self):
        g = gen.LAYOUT["gondola"]
        xs = sorted(g["centers_x"])
        for a, b in zip(xs, xs[1:]):
            self.assertGreaterEqual((b - a) - g["depth"], 2.0)
        east_aisle = (L - gen.LAYOUT["wall_shelf"]["depth"]) - (xs[-1] + g["depth"] / 2)
        self.assertGreaterEqual(east_aisle, 2.0)

    def test_pick_poses_free_and_facing_shelf(self):
        so = gen.LAYOUT["pick_standoff"]
        for name, it in ITEMS.items():
            self.assertGreaterEqual(clearance(it["x"], it["y"]), gen.PERSON_RADIUS, name)
            fx = it["x"] + so * math.cos(it["yaw"])
            fy = it["y"] + so * math.sin(it["yaw"])
            self.assertLess(clearance(fx, fy), 0.05, f"{name}: does not face its shelf")

    def test_scripted_paths_collision_free(self):
        for actor in ACTORS["busy"]:
            for t, x, y, _, _ in actor.sample(20):
                self.assertTrue(0.3 <= x <= L - 0.3 and 0.3 <= y <= W - 0.3, f"{actor.name} leaves store at t={t}")
                self.assertGreaterEqual(clearance(x, y), gen.PERSON_RADIUS - 1e-6,
                                        f"{actor.name} too close to an obstacle at t={t} ({x:.2f},{y:.2f})")

    def test_walking_speed_plausible(self):
        for actor in ACTORS["busy"]:
            pts = actor.sample(20)
            for (t0, x0, y0, *_), (t1, x1, y1, *_) in zip(pts, pts[1:]):
                self.assertLessEqual(math.hypot(x1 - x0, y1 - y0) / (t1 - t0), 1.5, actor.name)


class Scenarios(unittest.TestCase):
    def setUp(self):
        gen.main()

    def labels(self, world):
        with (ROOT / "scenarios" / f"{world}_labels.csv").open() as fh:
            return list(csv.DictReader(fh))

    def test_only_searching_actor_has_searching_labels(self):
        rows = self.labels("arc_store_busy")
        searching = {r["actor"] for r in rows if r["label"].startswith("searching")}
        self.assertEqual(searching, {"shopper_searching"})

    def test_searching_actor_passes_target_before_finding_it(self):
        srch = ACTORS["searching"][0]
        target = ITEMS["pasta"]
        first_search = next(s for s, e, lb in srch.labels if lb.startswith("searching"))
        found = next(s for s, e, lb in srch.labels if lb == "pause_at_item" and s > first_search)
        self.assertGreater(found - first_search, 30.0)
        # where searching starts, the shopper is level with the target item
        x, y = next((x, y) for t, x, y, _, _ in srch.sample(10) if t >= first_search)
        self.assertLess(abs(y - target["y"]), 0.3)

    def test_labels_cover_cycle_without_gaps(self):
        for world in ("arc_store_normal", "arc_store_searching", "arc_store_busy"):
            by = {}
            for r in self.labels(world):
                by.setdefault(r["actor"], []).append((float(r["t_start"]), float(r["t_end"]), float(r["cycle_s"])))
            for actor, ivs in by.items():
                self.assertAlmostEqual(ivs[0][0], 0.0, places=2)
                for (s0, e0, _), (s1, e1, _) in zip(ivs, ivs[1:]):
                    self.assertAlmostEqual(e0, s1, places=2, msg=actor)
                self.assertAlmostEqual(ivs[-1][1], ivs[-1][2], places=1, msg=actor)


class MapAndLaunch(unittest.TestCase):
    def test_map_header_and_cells(self):
        gen.main()
        data = (ROOT / "maps" / "store_map.pgm").read_bytes()
        header, rest = data.split(b"\n", 3)[:3], data.split(b"\n", 3)[3]
        self.assertEqual(header[0], b"P5")
        nx, ny = map(int, header[1].split())
        self.assertEqual(len(rest), nx * ny)
        res, pad = 0.05, 0.5

        def cell(x, y):
            i, j = int((x + pad) / res), int((y + pad) / res)
            return rest[(ny - 1 - j) * nx + i]

        for it in ITEMS.values():
            self.assertEqual(cell(it["x"], it["y"]), 254)
        self.assertEqual(cell(6.0, 9.0), 0)    # inside gondola 1
        self.assertEqual(cell(2.5, -0.1), 254)  # entrance gap is open

    def test_python_files_compile(self):
        for p in [ROOT / "launch" / "store.launch.py", ROOT / "arc_store_sim" / "camera_recorder.py",
                  ROOT / "setup.py"]:
            py_compile.compile(str(p), doraise=True)


if __name__ == "__main__":
    unittest.main()
