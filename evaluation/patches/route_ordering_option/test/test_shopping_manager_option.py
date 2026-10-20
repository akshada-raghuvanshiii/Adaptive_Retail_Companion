"""Runs the patched shopping_manager.py with stand-in ROS modules and typed answers (no ROS needed).

    python3 -m unittest discover -s test -v

Checks, on the shopping manager as it was in slam_ws (12 Oct) and on one that already has the earlier
route-order patch:
  - option 1 publishes the goals in the optimised order, option 2 in the typed order;
  - the later stops are re-ordered only when the optimised route was chosen;
  - route_ordering:=as_entered never asks and keeps the typed order;
  - with no map, it says so and keeps the typed order.
"""
import builtins
from yaml import safe_load as yaml_load
import importlib.util
import io
import math
import os
import shutil
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
KIT = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(KIT, "..", "..", ".."))
DATA = os.path.join(HERE, "data")


def _pick(repo_path, data_path):
    """Use the repository's copy when the kit sits in the repo, else the copy bundled in test/data."""
    return repo_path if os.path.exists(repo_path) else data_path


SM_SRC = _pick(os.path.join(REPO, "ros2_ws", "src", "arc_planner", "arc_planner", "shopping_manager.py"),
               os.path.join(DATA, "shopping_manager_12oct.py"))
ITEMS = _pick(os.path.join(REPO, "ros2_ws", "src", "arc_store_sim", "config", "item_locations.yaml"),
              os.path.join(DATA, "item_locations.yaml"))
DEST = _pick(os.path.join(REPO, "ros2_ws", "src", "arc_store_sim", "config", "destinations_map.yaml"),
             os.path.join(DATA, "destinations_map.yaml"))
V1_KIT = _pick(os.path.join(REPO, "evaluation", "patches", "phase2_fixes", "route_order"),
               os.path.join(DATA, "v1"))

sys.path.insert(0, KIT)
sys.path.insert(0, HERE)
import apply_route_option as apply_v2  # noqa: E402
from test_route_order import store_grid, RES, OX, OY, W, H  # noqa: E402


# ------------------------------------------------------------------ stand-in ROS modules
class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _pose():
    p = _Msg()
    p.header = _Msg(stamp=None, frame_id="")
    p.pose = _Msg(position=_Msg(x=0.0, y=0.0, z=0.0), orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))
    return p


class _Pub:
    def __init__(self, topic, log):
        self.topic, self.log = topic, log

    def publish(self, msg):
        self.log.append((self.topic, msg))


class _Param:
    def __init__(self, v):
        self.value = v


class _Logger:
    def __init__(self, log):
        self.log = log

    def info(self, m):
        self.log.append(("info", m))

    def warn(self, m):
        self.log.append(("warn", m))

    error = warn


class _Clock:
    def now(self):
        return _Msg(to_msg=lambda: None)


class FakeNode:
    PARAMS = {}
    PUBLISHED = []
    LOG = []

    def __init__(self, name):
        self._subs = {}

    def create_publisher(self, typ, topic, qos):
        return _Pub(topic, FakeNode.PUBLISHED)

    def create_subscription(self, typ, topic, cb, qos):
        self._subs[topic] = cb

    def declare_parameter(self, name, default):
        return _Param(FakeNode.PARAMS.get(name, default))

    def get_logger(self):
        return _Logger(FakeNode.LOG)

    def get_clock(self):
        return _Clock()


class FakeTF:
    POSE = (-0.5, -2.0)

    class Buffer:
        def lookup_transform(self, a, b, t):
            if FakeTF.POSE is None:
                raise RuntimeError("no tf")
            x, y = FakeTF.POSE
            return _Msg(transform=_Msg(translation=_Msg(x=x, y=y, z=0.0)))

    class TransformListener:
        def __init__(self, buf, node):
            pass


MAP = {"msg": None}


def install_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None

    def spin_once(node, timeout_sec=0.1):
        cb = node._subs.get("/rtabmap/map")
        if cb and MAP["msg"] is not None:
            cb(MAP["msg"])
    rclpy.spin_once = spin_once
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = FakeNode
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kw: kw
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    rtime = types.ModuleType("rclpy.time")
    rtime.Time = lambda *a, **k: 0
    geo = types.ModuleType("geometry_msgs.msg")
    geo.PoseStamped = _pose
    std = types.ModuleType("std_msgs.msg")
    std.String = lambda: _Msg(data="")
    std.Bool = lambda: _Msg(data=False)
    nav = types.ModuleType("nav_msgs.msg")
    nav.OccupancyGrid = object
    tf2 = types.ModuleType("tf2_ros")
    tf2.Buffer = FakeTF.Buffer
    tf2.TransformListener = FakeTF.TransformListener
    for name, mod in {"rclpy": rclpy, "rclpy.node": node_mod, "rclpy.qos": qos, "rclpy.time": rtime,
                      "geometry_msgs": types.ModuleType("geometry_msgs"), "geometry_msgs.msg": geo,
                      "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": std,
                      "nav_msgs": types.ModuleType("nav_msgs"), "nav_msgs.msg": nav, "tf2_ros": tf2}.items():
        sys.modules[name] = mod
    # arc_planner.route_order -> the kit's copy
    pkg = types.ModuleType("arc_planner")
    pkg.__path__ = [os.path.join(KIT, "arc_planner", "arc_planner")]
    sys.modules["arc_planner"] = pkg
    spec = importlib.util.spec_from_file_location(
        "arc_planner.route_order", os.path.join(KIT, "arc_planner", "arc_planner", "route_order.py"))
    ro = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ro)
    sys.modules["arc_planner.route_order"] = ro


def map_msg():
    g = store_grid()
    data = []
    for gy in range(H):
        for gx in range(W):
            data.append(100 if g.data[gy * W + gx] >= 50 else 0)
    return _Msg(data=data, info=_Msg(width=W, height=H, resolution=RES,
                                     origin=_Msg(position=_Msg(x=OX, y=OY))))


# ------------------------------------------------------------------ helpers
def load_patched(src_text):
    d = tempfile.mkdtemp()
    p = os.path.join(d, "shopping_manager_patched.py")
    with open(p, "w") as f:
        f.write(src_text)
    spec = importlib.util.spec_from_file_location("shopping_manager_patched", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def run_manager(mod, answers):
    FakeNode.PUBLISHED.clear()
    FakeNode.LOG.clear()
    it = iter(answers)
    prompts = []

    def fake_input(prompt=""):
        prompts.append(prompt.strip())
        return next(it)
    old = builtins.input
    builtins.input = fake_input
    out = io.StringIO()
    try:
        with redirect_stdout(out):
            node = mod.ShoppingManager()
    finally:
        builtins.input = old
    return node, prompts, out.getvalue()


def goal_items():
    return [m.data for t, m in FakeNode.PUBLISHED if t == "/shopping_item"]


def shop(node):
    """Item stops only (the checkout is checked separately)."""
    return [g["item"] for g in node.navigation_goals if not g.get("is_checkout")]


def complete_stop(node, pose):
    FakeTF.POSE = pose
    old = builtins.input
    builtins.input = lambda p="": "y"
    out = io.StringIO()
    try:
        with redirect_stdout(out):
            node.navigation_complete_callback(types.SimpleNamespace(data=True))
    finally:
        builtins.input = old
    return out.getvalue()


TYPED = ["pasta", "tea_coffee", "milk", "fruits"]
LIST_ANSWERS = [", ".join(TYPED), "n", "y"]          # add items, no more, confirm


class ShoppingManagerOptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        install_stubs()
        cls.home = tempfile.mkdtemp()
        cfg = os.path.join(cls.home, "slam_ws", "src", "arc_store_sim", "config")
        os.makedirs(cfg)
        shutil.copy(ITEMS, cfg)
        shutil.copy(DEST, cfg)
        cls._old_home = os.environ.get("HOME")
        os.environ["HOME"] = cls.home
        with open(SM_SRC) as f:
            src = f.read()
        cls.fresh_src, cls.fresh_kind = apply_v2.patch(src)
        sys.path.insert(0, V1_KIT)
        import apply_route_order as apply_v1
        cls.v1_src = apply_v1.patch(src)
        cls.upgraded_src, cls.upgrade_kind = apply_v2.patch(cls.v1_src)
        # the state on the laptop after the first route-option install (choice, no checkout default)
        cls.v2_src = apply_v2.option_patch(apply_v2.base_patch(src))
        cls.from_v2_src, cls.from_v2_kind = apply_v2.patch(cls.v2_src)
        with open(DEST) as f:
            cls.checkout = yaml_load(f)["destinations"]["checkout"]
        cls.mod = load_patched(cls.fresh_src)
        cls.mod_up = load_patched(cls.upgraded_src)
        import yaml
        with open(ITEMS) as f:
            cls.items = yaml.safe_load(f)["items"]

    @classmethod
    def tearDownClass(cls):
        if cls._old_home is not None:
            os.environ["HOME"] = cls._old_home

    def setUp(self):
        FakeNode.PARAMS = {}
        FakeTF.POSE = (-0.5, -2.0)
        MAP["msg"] = map_msg()

    # -------------------------------------------------------------- patch mechanics
    def test_patch_kinds_and_refuses_twice(self):
        self.assertTrue(self.fresh_kind.startswith("fresh"))
        self.assertTrue(self.upgrade_kind.startswith("upgrade"))
        with self.assertRaises(SystemExit):
            apply_v2.patch(self.fresh_src)
        # an upgraded v1 file, an upgraded v2 file and a fresh file all end up identical
        self.assertEqual(self.fresh_src, self.upgraded_src)
        self.assertTrue(self.from_v2_kind.startswith("upgrade"))
        self.assertEqual(self.fresh_src, self.from_v2_src)

    def test_only_shopping_manager_logic_changes(self):
        with open(SM_SRC) as f:
            src = f.read()
        for line in src.splitlines():
            if "route_ordering" in line:
                self.fail("original already mentions route_ordering?")
        self.assertIn("'ask'", self.fresh_src)
        self.assertIn("_ro_choose", self.fresh_src)

    # -------------------------------------------------------------- behaviour
    def test_option_1_publishes_optimised_order(self):
        node, prompts, out = run_manager(self.mod, LIST_ANSWERS + ["1"])
        self.assertIn("ROUTE OPTIONS", out)
        self.assertTrue(any(p.startswith("Choose route") for p in prompts))
        self.assertEqual(node.route_ordering, "exact")
        order = shop(node)
        self.assertEqual(sorted(order), sorted(TYPED))
        self.assertNotEqual(order, TYPED)                    # this list is not typed in the best order
        self.assertEqual(goal_items()[0], order[0])          # first published goal is the first optimised stop

    def test_enter_defaults_to_optimised(self):
        node, _, _ = run_manager(self.mod, LIST_ANSWERS + [""])
        self.assertEqual(node.route_ordering, "exact")

    def test_option_2_keeps_typed_order(self):
        node, _, out = run_manager(self.mod, LIST_ANSWERS + ["2"])
        self.assertEqual(shop(node), TYPED)
        self.assertEqual(goal_items()[0], TYPED[0])
        self.assertEqual(node.route_ordering, "as_entered")

    def test_bad_answer_asks_again(self):
        node, prompts, out = run_manager(self.mod, LIST_ANSWERS + ["x", "2"])
        self.assertIn("Please enter 1 or 2.", out)
        self.assertEqual(node.route_ordering, "as_entered")

    def test_optimised_route_is_not_longer(self):
        node, _, out = run_manager(self.mod, LIST_ANSWERS + ["1"])
        line1 = [l for l in out.splitlines() if "[1] Optimised" in l][0]
        line2 = [l for l in out.splitlines() if "[2] As typed" in l][0]
        c1 = float(line1.split(":", 1)[1].split("m")[0])
        c2 = float(line2.split(":", 1)[1].split("m")[0])
        self.assertLessEqual(c1, c2)

    def test_typed_choice_is_not_reordered_at_next_stop(self):
        node, _, _ = run_manager(self.mod, LIST_ANSWERS + ["2"])
        FakeTF.POSE = (self.items[TYPED[0]]["x"], self.items[TYPED[0]]["y"])
        builtins_input = builtins.input
        builtins.input = lambda p="": "y"
        try:
            with redirect_stdout(io.StringIO()):
                node.navigation_complete_callback(types.SimpleNamespace(data=True))
        finally:
            builtins.input = builtins_input
        self.assertEqual(shop(node), TYPED)

    def test_optimised_choice_reorders_from_new_pose(self):
        node, _, _ = run_manager(self.mod, LIST_ANSWERS + ["1"])
        first = node.navigation_goals[0]
        FakeTF.POSE = (first["x"], first["y"])
        builtins_input = builtins.input
        builtins.input = lambda p="": "y"
        try:
            with redirect_stdout(io.StringIO()):
                node.navigation_complete_callback(types.SimpleNamespace(data=True))
        finally:
            builtins.input = builtins_input
        rest = shop(node)
        self.assertEqual(rest[0], first["item"])
        self.assertEqual(sorted(rest), sorted(TYPED))

    def test_as_entered_parameter_never_asks(self):
        FakeNode.PARAMS = {"route_ordering": "as_entered"}
        node, prompts, out = run_manager(self.mod, LIST_ANSWERS)
        self.assertNotIn("ROUTE OPTIONS", out)
        self.assertFalse(any(p.startswith("Choose route") for p in prompts))
        self.assertEqual(shop(node), TYPED)

    def test_exact_parameter_never_asks(self):
        FakeNode.PARAMS = {"route_ordering": "exact"}
        node, prompts, out = run_manager(self.mod, LIST_ANSWERS)
        self.assertFalse(any(p.startswith("Choose route") for p in prompts))
        self.assertIn("ROUTE ORDER (Dijkstra path cost)", out)

    def test_no_map_keeps_typed_order_and_says_why(self):
        MAP["msg"] = None
        node, prompts, out = run_manager(self.mod_up, LIST_ANSWERS)   # upgraded file behaves the same
        self.assertFalse(any(p.startswith("Choose route") for p in prompts))
        self.assertEqual(shop(node), TYPED)
        self.assertTrue(any("Route options unavailable" in m for k, m in FakeNode.LOG if k == "warn"))

    # -------------------------------------------------------------- checkout as the final destination
    def assert_checkout_last(self, node):
        last = node.navigation_goals[-1]
        self.assertTrue(last.get("is_checkout"))
        self.assertAlmostEqual(last["x"], float(self.checkout["x"]))
        self.assertAlmostEqual(last["y"], float(self.checkout["y"]))
        self.assertAlmostEqual(last["yaw"], float(self.checkout.get("yaw", 0.0)))
        self.assertEqual(sum(1 for g in node.navigation_goals if g.get("is_checkout")), 1)

    def test_checkout_is_last_for_both_choices(self):
        for ans in ("1", "2"):
            node, _, out = run_manager(self.mod, LIST_ANSWERS + [ans])
            self.assert_checkout_last(node)
            line = [l for l in out.splitlines() if ("[1] Optimised" if ans == "1" else "[2] As typed") in l][0]
            self.assertTrue(line.rstrip().endswith("-> Checkout"))

    def test_checkout_for_a_single_item(self):
        node, prompts, _ = run_manager(self.mod, ["milk", "n", "y"])
        self.assertEqual([g["item"] for g in node.navigation_goals], ["milk", "Checkout"])
        self.assertFalse(any(p.startswith("Choose route") for p in prompts))   # nothing to choose

    def test_checkout_stays_last_after_reorder(self):
        node, _, _ = run_manager(self.mod, LIST_ANSWERS + ["1"])
        first = node.navigation_goals[0]
        complete_stop(node, (first["x"], first["y"]))
        self.assert_checkout_last(node)

    def test_checkout_even_without_map_or_ordering(self):
        MAP["msg"] = None
        node, _, _ = run_manager(self.mod, LIST_ANSWERS)
        self.assertEqual(shop(node), TYPED)
        self.assert_checkout_last(node)
        FakeNode.PARAMS = {"route_ordering": "as_entered"}
        MAP["msg"] = map_msg()
        node, _, _ = run_manager(self.mod, LIST_ANSWERS)
        self.assert_checkout_last(node)

    def test_route_end_at_checkout_false_is_old_behaviour(self):
        FakeNode.PARAMS = {"route_end_at_checkout": False}
        node, _, _ = run_manager(self.mod, LIST_ANSWERS + ["2"])
        self.assertEqual([g["item"] for g in node.navigation_goals], TYPED)

    def test_explicit_checkout_parameters_win(self):
        FakeNode.PARAMS = {"checkout_x": 0.5, "checkout_y": -2.4, "checkout_yaw": 0.0}
        node, _, _ = run_manager(self.mod, LIST_ANSWERS + ["2"])
        last = node.navigation_goals[-1]
        self.assertTrue(last.get("is_checkout"))
        self.assertEqual((last["x"], last["y"]), (0.5, -2.4))

    def test_missing_destinations_file_warns_and_skips(self):
        FakeNode.PARAMS = {"checkout_destinations_file": "/nonexistent/destinations_map.yaml"}
        node, _, _ = run_manager(self.mod, LIST_ANSWERS + ["2"])
        self.assertEqual([g["item"] for g in node.navigation_goals], TYPED)
        self.assertTrue(any("Checkout: cannot read" in m for k, m in FakeNode.LOG if k == "warn"))

    def test_trip_ends_at_counter_with_message(self):
        node, _, _ = run_manager(self.mod, LIST_ANSWERS + ["2"])
        out = ""
        for g in node.navigation_goals:
            out = complete_stop(node, (g["x"], g["y"]))
        self.assertIn("Reached the checkout counter", out)
        self.assertIn("SHOPPING COMPLETE", out)
        self.assertEqual(goal_items()[-1], "Checkout")


if __name__ == "__main__":
    unittest.main()
