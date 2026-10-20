"""Runs server.UiBridge with tiny stand-ins for rclpy/msgs/tf2 (no ROS needed), to check the API
glue: trip start, stale-arrival guard, hold/cancel on pause, conflict detection, products endpoint."""
import math
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
LOG = []
OVERRIDES = {}


def install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    class Pose:
        def __init__(self):
            self.position = Msg(x=0.0, y=0.0, z=0.0)
            self.orientation = Msg(x=0.0, y=0.0, z=0.0, w=1.0)

    class PoseStamped:
        def __init__(self):
            self.header = Msg(stamp=None, frame_id="")
            self.pose = Pose()

    class String:
        def __init__(self, data=""):
            self.data = data

    class Bool:
        def __init__(self, data=False):
            self.data = data

    class Pub:
        def __init__(self, topic):
            self.topic = topic

        def publish(self, m):
            LOG.append((self.topic, m))

    class Param:
        def __init__(self, v):
            self.value = v

    class Node:
        def __init__(self, name):
            self._p = {}
            self.publishers_count = {"/shopping_goal": 1}

        def declare_parameter(self, n, v):
            if n == "item_locations":
                v = os.environ.get("ARC_ITEM_LOCATIONS",
                                   os.path.expanduser("~/slam_ws/src/arc_store_sim/config/item_locations.yaml"))
            if n == "destinations_file":
                v = os.environ.get("ARC_DESTINATIONS", v)
            v = OVERRIDES.get(n, v)
            self._p[n] = Param(v)

        def get_parameter(self, n):
            return self._p[n]

        def create_publisher(self, t, topic, q):
            return Pub(topic)

        def create_subscription(self, *a):
            return None

        def create_client(self, t, name):
            node = self

            class C:
                def service_is_ready(self):
                    return True

                def call_async(self, req):
                    LOG.append((name, "cancel-all"))
            return C()

        def count_publishers(self, topic):
            return self.publishers_count.get(topic, 0)

        def get_logger(self):
            return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))

    class Buffer:
        pose = (0.0, 0.0, 0.0)

        def lookup_transform(self, a, b, t):
            x, y, th = Buffer.pose
            return Msg(transform=Msg(translation=Msg(x=x, y=y, z=0.0),
                                     rotation=Msg(x=0.0, y=0.0, z=math.sin(th / 2), w=math.cos(th / 2))))

    mod("rclpy", init=lambda: None, ok=lambda: False, shutdown=lambda: None, spin=lambda n: None,
        time=types.SimpleNamespace(Time=lambda: 0))
    mod("rclpy.node", Node=Node)
    mod("rclpy.qos", QoSProfile=lambda **k: None, ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1),
        DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1))
    mod("geometry_msgs"), mod("geometry_msgs.msg", PoseStamped=PoseStamped)
    mod("nav_msgs"), mod("nav_msgs.msg", OccupancyGrid=object)
    mod("std_msgs"), mod("std_msgs.msg", Bool=Bool, String=String)
    mod("action_msgs"), mod("action_msgs.srv", CancelGoal=types.SimpleNamespace(Request=lambda: "req"))
    mod("tf2_ros", Buffer=Buffer, TransformListener=lambda b, n: None)
    return Buffer, Bool


def test_api_flow():
    Buffer, Bool = install_stubs()
    OVERRIDES.clear()
    OVERRIDES["catalog"] = "products"          # the grocery demo list; store mode is tested below
    from arc_ui_bridge import route_order as ro
    from arc_ui_bridge import server
    node = server.UiBridge()
    node._p["item_locations"].value  # loaded already
    node.grid = ro.CostGrid([0] * (160 * 140), 160, 140, 0.05, -4.0, -3.0)
    clock = [100.0]
    node.now = lambda: clock[0]

    prods = node.api_products()
    assert len(prods) == 60 and all(p["sim_item"] for p in prods)

    s = node.api_trip({"items": [{"item_id": "FP001", "quantity": 1, "name": "Apple"},
                                 {"item_id": "BV001", "quantity": 2, "name": "Tea"}]})
    trip = s["trip"]
    assert trip["state"] == "NAVIGATING" and len([x for x in trip["stops"] if not x["is_checkout"]]) == 2
    assert trip["stops"][-1]["is_checkout"] and s["checkout"] == [-1.8, -1.25]
    goals = [m for t, m in LOG if t == "/shopping_goal"]
    first = trip["stops"][0]
    assert abs(goals[-1].pose.position.x - first["x"]) < 1e-9

    # stale latched arrival right after the goal: ignored
    node.on_reached(Bool(True))
    assert node.trip.state == "NAVIGATING" and node.trip.ignored_arrivals == 1
    # robot reaches the stop later: accepted
    clock[0] += 20
    Buffer.pose = (first["x"] + 0.2, first["y"], 0.0)
    node.on_reached(Bool(True))
    assert node.trip.state == "AT_STOP"

    node.api_control({"action": "next"})
    assert node.trip.state == "NAVIGATING"
    out = node.api_control({"action": "pause"})
    assert node.trip.state == "PAUSED" and "warning" not in out
    assert LOG[-1][1] == "cancel-all"
    hold = [m for t, m in LOG if t == "/shopping_goal"][-1]
    assert abs(hold.pose.position.x - Buffer.pose[0]) < 1e-9
    node.api_control({"action": "resume"})
    assert node.trip.state == "NAVIGATING"

    # a second publisher of /shopping_goal (shopping_manager) blocks new trips
    node.api_control({"action": "home"})
    clock[0] += 30
    Buffer.pose = (0.0, 0.0, 0.0)
    node.on_reached(Bool(True))
    assert node.trip.state == "DONE"
    node.publishers_count["/shopping_goal"] = 2
    try:
        node.api_trip({"items": [{"item_id": "FP001"}]})
    except ValueError as e:
        assert "shopping_manager" in str(e)
    else:
        raise AssertionError("conflict not detected")


def test_plan_and_route_choice():
    OVERRIDES.clear()
    OVERRIDES["catalog"] = "products"
    from arc_ui_bridge import route_order as ro
    from arc_ui_bridge import server
    Buffer = sys.modules["tf2_ros"].Buffer
    node = server.UiBridge()
    node.grid = ro.CostGrid([0] * (160 * 140), 160, 140, 0.05, -4.0, -3.0)
    clock = [100.0]
    node.now = lambda: clock[0]
    Buffer.pose = (3.0, 1.4, 0.0)
    items = [{"item_id": "FP001", "name": "Apple"}, {"item_id": "PC002", "name": "Soap"},
             {"item_id": "BV001", "name": "Tea"}, {"item_id": "DY001", "name": "Milk"}]
    plan = node.api_plan({"items": items})
    assert plan["ask"] is True and plan["checkout"] == [-1.8, -1.25]
    o, t = plan["options"]["optimised"], plan["options"]["typed"]
    assert o["stops"][-1] == t["stops"][-1] == "CHECKOUT"
    typed_items = [node.product_map[i["item_id"]] for i in items]
    assert t["stops"][:-1] == typed_items                       # typed keeps the list order
    assert sorted(o["stops"][:-1]) == sorted(typed_items)
    assert o["cost"] <= t["cost"] + 1e-6 and abs(plan["saved_m"] - (t["cost"] - o["cost"])) < 0.02

    for route, expect in (("typed", t["stops"]), ("optimised", o["stops"])):
        node.trip.state = "IDLE"
        s = node.api_trip({"items": items, "route": route})
        got = [x["label"] for x in s["trip"]["stops"]]
        assert got == expect, (route, got, expect)
        assert s["trip"]["info"]["route"] == route and s["trip"]["info"]["ends_at"] == "checkout"
        node.api_control({"action": "stop"})

    # without a map: no options, the typed order, still ends at the checkout
    node.trip.state = "IDLE"
    node.grid = None
    plan = node.api_plan({"items": items})
    assert plan["options"] is None and "no map" in plan["reason"]
    s = node.api_trip({"items": items})
    assert [x["label"] for x in s["trip"]["stops"]] == typed_items + ["CHECKOUT"]
    node.api_control({"action": "stop"})

    try:
        node.trip.state = "IDLE"
        node.api_trip({"items": items, "route": "shortest"})
    except ValueError as e:
        assert "unknown route" in str(e)
    else:
        raise AssertionError("bad route accepted")


def test_checkout_off_and_overrides():
    from arc_ui_bridge import server
    OVERRIDES.clear()
    OVERRIDES.update(catalog="products", end_at_checkout=False)
    node = server.UiBridge()
    assert node.checkout is None
    s = node.api_trip({"items": [{"item_id": "FP001"}]})
    assert not any(x["is_checkout"] for x in s["trip"]["stops"])
    OVERRIDES.clear()
    OVERRIDES.update(catalog="products", checkout_x=0.5, checkout_y=-2.4)
    node = server.UiBridge()
    assert (node.checkout["x"], node.checkout["y"]) == (0.5, -2.4)
    OVERRIDES.clear()
    OVERRIDES.update(catalog="products", destinations_file="/nonexistent.yaml")
    node = server.UiBridge()
    assert node.checkout is None                                   # warns and falls back to returning home
    OVERRIDES.clear()


def test_store_catalog_mode():
    """Default catalog: the UI lists the Gazebo store's items; a list of store item names behaves like
    a list typed into the shopping manager (typed order kept, checkout last)."""
    from arc_ui_bridge import route_order as ro
    from arc_ui_bridge import server
    OVERRIDES.clear()
    node = server.UiBridge()
    assert node.catalog == "store"
    prods = node.api_products()
    names = {p["id"] for p in prods}
    assert names == set(node.items) - {"checkout"} and len(prods) == len(names)
    assert all(p["sim_item"] == p["id"] for p in prods)
    node.grid = ro.CostGrid([0] * (160 * 140), 160, 140, 0.05, -4.0, -3.0)
    node.now = lambda: 500.0
    typed = ["pasta", "tea_coffee", "milk", "fruits"]
    items = [{"item_id": i, "quantity": 1} for i in typed]
    plan = node.api_plan({"items": items})
    assert plan["options"]["typed"]["stops"] == typed + ["CHECKOUT"]
    s = node.api_trip({"items": items, "route": "typed"})
    assert [x["label"] for x in s["trip"]["stops"]] == typed + ["CHECKOUT"]
    assert s["catalog"] == "store"
    node.api_control({"action": "stop"})
    node.trip.state = "IDLE"
    bad = node.api_plan({"items": items + [{"item_id": "caviar"}]})
    assert any("caviar" in p for p in bad["problems"])


if __name__ == "__main__":
    test_api_flow()
    test_plan_and_route_choice()
    test_checkout_off_and_overrides()
    test_store_catalog_mode()
    print("server API flow OK")
