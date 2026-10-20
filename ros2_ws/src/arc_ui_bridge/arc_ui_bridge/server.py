#!/usr/bin/env python3
"""ARC UI bridge: serves the ARC web UI and connects it to the running ARC stack.

    ros2 launch arc_ui_bridge arc_ui.launch.py          # then open http://localhost:8080

UI -> robot: the shopping list becomes stops in the simulated store (product_map.yaml ->
item_locations.yaml), ordered by Dijkstra path cost on the live map (exact Held-Karp up to
12 stops, the same route_order.py as the phase-3 patch), and sent one at a time on
/shopping_goal + /shopping_item - the topics ARC's shopping_manager uses, so the Dijkstra
planner, hand-over, controller, behaviour tree and motion gate are unchanged.

Route choice: POST /api/plan prices the optimised (Dijkstra path cost, Held-Karp) and the typed
order on the live map from the robot's pose; the UI shows both and the shopper picks one, which
POST /api/trip then drives ("route": "optimised" | "typed").

Checkout: every trip ends at the checkout counter (end_at_checkout, default true), read from the
store's destinations_map.yaml unless checkout_x/checkout_y are set. The optimised order is planned
with the checkout as the fixed end. Arriving at the checkout finishes the trip.

Robot -> UI: behaviour-tree mode, robot pose, trip progress, arrivals and ignored stale
arrivals, once per poll.

It replaces shopping_manager for UI trips: do not run both. Nothing in ARC is modified.
The UI's STOP button is a software stop (the robot is held at its current pose); the
hardware E-stop on the Uno remains the safety device.
"""
import json
import math
import os
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.srv import CancelGoal
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from . import route_order as ro
from .trip import Trip, resolve

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("arc_ui_bridge")
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class UiBridge(Node):
    def __init__(self):
        super().__init__("arc_ui_bridge")
        sd = share_dir()
        self.declare_parameter("port", 8080)
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("web_dir", os.path.join(sd, "web"))
        self.declare_parameter("product_map", os.path.join(sd, "config", "product_map.yaml"))
        self.declare_parameter("item_locations", "~/slam_ws/src/arc_store_sim/config/item_locations.yaml")
        self.declare_parameter("map_topic", "/rtabmap/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("strategy", "exact")          # exact | nearest | as_entered (when no choice is sent)
        self.declare_parameter("route_choice", "ask")         # ask: the UI shows both routes | fixed: use strategy
        self.declare_parameter("end_at_checkout", True)
        self.declare_parameter("destinations_file", "~/slam_ws/src/arc_store_sim/config/destinations_map.yaml")
        self.declare_parameter("checkout_x", float("nan"))
        self.declare_parameter("checkout_y", float("nan"))
        self.declare_parameter("checkout_yaw", 0.0)
        self.declare_parameter("arrive_m", 1.0)
        self.declare_parameter("min_leg_s", 3.0)   # > the planner's 2 s replan, so a stale latched arrival is never taken
        self.declare_parameter("follow_path_action", "/follow_path")
        g = lambda n: self.get_parameter(n).value
        self.web_dir = os.path.expanduser(g("web_dir"))
        self.product_map = yaml.safe_load(open(os.path.expanduser(g("product_map")))) or {}
        il = yaml.safe_load(open(os.path.expanduser(g("item_locations")))) or {}
        self.items = il.get("items", {})
        self.item_frame = il.get("frame_id", "map")
        self.map_frame, self.base_frame = g("map_frame"), g("base_frame")
        self.strategy = g("strategy")
        self.route_choice = str(g("route_choice"))
        self.checkout = self.find_checkout(bool(g("end_at_checkout")), float(g("checkout_x")), float(g("checkout_y")),
                                           float(g("checkout_yaw")), os.path.expanduser(str(g("destinations_file"))))
        self.trip = Trip(float(g("arrive_m")), float(g("min_leg_s")))
        self.lock = threading.Lock()
        self.grid, self.grid_stamp = None, None
        self.bt_mode, self.bt_t = None, None
        self.t0 = time.time()

        self.pub_goal = self.create_publisher(PoseStamped, "/shopping_goal", LATCHED)
        self.pub_item = self.create_publisher(String, "/shopping_item", LATCHED)
        self.create_subscription(Bool, "/navigation_goal_reached", self.on_reached, LATCHED)
        self.create_subscription(String, "/arc/behavior_mode", self.on_mode, 10)
        self.create_subscription(OccupancyGrid, g("map_topic"), self.on_map, LATCHED)
        self.cancel_cli = self.create_client(CancelGoal, g("follow_path_action").rstrip("/") + "/_action/cancel_goal")
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        missing = sorted({v for v in self.product_map.values() if v not in self.items})
        if missing:
            self.get_logger().warn(f"product_map names store items missing from item_locations.yaml: {missing}")
        self.get_logger().info(f"{len(self.product_map)} UI products -> {len(set(self.product_map.values()))} store items; "
                               f"open http://{g('host')}:{g('port')}")

    # ------------------------------------------------------------------ checkout
    def find_checkout(self, enabled, x, y, yaw, dest_file):
        """Checkout waiting point: parameters, else a 'checkout' item, else the destinations file."""
        if not enabled:
            self.get_logger().info("end_at_checkout is false: trips end at the last item and return home")
            return None
        src = "parameters"
        if math.isnan(x) or math.isnan(y):
            loc, src = self.items.get("checkout") or {}, "item_locations.yaml"
            if "x" in loc and "y" in loc:
                x, y, yaw = float(loc["x"]), float(loc["y"]), float(loc.get("yaw", yaw))
        if math.isnan(x) or math.isnan(y):
            src = dest_file
            try:
                with open(dest_file) as f:
                    d = yaml.safe_load(f) or {}
                if d.get("frame_id", "map") != self.item_frame:
                    raise ValueError(f"frame {d.get('frame_id')} != items' frame {self.item_frame}")
                c = (d.get("destinations") or {}).get("checkout") or {}
                x, y, yaw = float(c["x"]), float(c["y"]), float(c.get("yaw", 0.0))
            except Exception as e:
                self.get_logger().warn(f"no checkout position ({src}: {e!r}); trips will return home instead")
                return None
        self.get_logger().info(f"checkout counter ({x:.2f}, {y:.2f}) from {src}: every trip ends there")
        return {"item": "checkout", "items": ["checkout"], "x": x, "y": y, "yaw": yaw, "is_checkout": True}

    def plan(self, goals, start_xy, strategy):
        """Ordered stops (+ the checkout last) and the info dict, on the live map when there is one."""
        end_xy = (self.checkout["x"], self.checkout["y"]) if self.checkout else None
        if self.grid is not None:
            ordered, info = ro.plan_route(self.grid, start_xy, goals, strategy, end_xy=end_xy)
        else:
            ordered, info = ro.group_same_spot(goals), {"strategy": "as_entered (no map yet)"}
        if self.checkout:
            ordered = list(ordered) + [dict(self.checkout)]
        info = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in info.items() if k != "start_cell"}
        info["ends_at"] = "checkout" if self.checkout else "last stop, then home"
        return ordered, info

    # ------------------------------------------------------------------ ROS side
    def now(self):
        return time.time() - self.t0

    def on_map(self, msg):
        try:
            grid = ro.CostGrid.from_msg(msg)
        except Exception as e:
            self.get_logger().warn(f"map not usable for ordering: {e}")
            return
        self.grid, self.grid_stamp = grid, self.now()

    def on_mode(self, msg):
        self.bt_mode, self.bt_t = msg.data, self.now()

    def robot_pose(self):
        try:
            tr = self.tf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (tr.transform.translation.x, tr.transform.translation.y, yaw)

    def on_reached(self, msg):
        if not msg.data:
            return
        p = self.robot_pose()
        with self.lock:
            self.trip.arrival(p[:2] if p else None, self.now())

    def send(self, stop, label=None):
        if stop is None:
            return
        item = String()
        item.data = label or stop["label"]
        self.pub_item.publish(item)
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.item_frame
        m.pose.position.x, m.pose.position.y = float(stop["x"]), float(stop["y"])
        yaw = float(stop.get("yaw", 0.0))
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub_goal.publish(m)

    def hold(self):
        """Pause / software stop.

        ARC's hand-over ignores new paths while the controller is following one, so a new goal alone
        would not stop the robot mid-leg. We therefore (1) cancel every FollowPath goal on the
        controller (CancelGoal with an empty goal id = cancel all), and (2) make the robot's current
        pose the shopping goal, so the next plan the hand-over accepts keeps the robot where it is."""
        warn = []
        p = self.robot_pose()             # hold goal first, so the planner's next replan is not the old goal
        if p is None:
            warn.append("no robot pose, so the hold goal was not sent")
        else:
            self.send({"x": p[0], "y": p[1], "yaw": p[2]}, label="HOLD")
        if self.cancel_cli.service_is_ready():
            self.cancel_cli.call_async(CancelGoal.Request())
        else:
            warn.append("controller cancel service not available")
        return ("; ".join(warn) + " (the motion gate and hardware E-stop still apply)") if warn else None

    # ------------------------------------------------------------------ API
    def api_status(self):
        p = self.robot_pose()
        with self.lock:
            snap = self.trip.snapshot()
        others = max(0, self.count_publishers("/shopping_goal") - 1)
        return {"ros": True, "bt_mode": self.bt_mode, "route_choice": self.route_choice,
                "checkout": None if not self.checkout else [self.checkout["x"], self.checkout["y"]],
                "bt_age_s": None if self.bt_t is None else round(self.now() - self.bt_t, 1),
                "pose": None if p is None else [round(p[0], 3), round(p[1], 3), round(p[2], 3)],
                "localised": p is not None, "map": self.grid is not None,
                "other_goal_publishers": others, "trip": snap,
                "store_items": {k: [v["x"], v["y"]] for k, v in self.items.items()}}

    def api_products(self):
        with open(os.path.join(self.web_dir, "products.json")) as f:
            prods = json.load(f)
        for p in prods:
            it = self.product_map.get(p["id"])
            p["sim_item"] = it
            if it in self.items:
                p["sim_x"], p["sim_y"] = self.items[it]["x"], self.items[it]["y"]
        return prods

    def _goals(self, body):
        products = body.get("items", [])
        if not products:
            raise ValueError("the shopping list is empty")
        goals, problems = resolve(products, self.product_map, self.items)
        if not goals:
            raise ValueError("none of the products map to a store item: " + "; ".join(problems))
        p = self.robot_pose()
        if p is None:
            raise ValueError("no robot pose (map -> base_footprint); is the stack localised?")
        return goals, problems, p

    def api_plan(self, body):
        """Both routes the shopper can choose between, priced on the live map from the robot."""
        goals, problems, p = self._goals(body)
        out = {"ask": self.route_choice == "ask", "problems": problems,
               "checkout": None if not self.checkout else [self.checkout["x"], self.checkout["y"]]}
        if self.grid is None:
            out.update(options=None, reason="no map yet: the typed order will be used")
            return out
        strat = self.strategy if self.strategy in ("exact", "nearest") else "exact"
        end_xy = (self.checkout["x"], self.checkout["y"]) if self.checkout else None
        c = ro.compare_routes(self.grid, p[:2], goals, strat, end_xy=end_xy)
        tail = ["CHECKOUT"] if self.checkout else []

        def opt(stops, info):
            return {"strategy": info["strategy"], "cost": round(info["cost"], 2),
                    "stops": [" + ".join(dict.fromkeys(s.get("items", [s["item"]]))) for s in stops] + tail,
                    "unreachable": info["unreachable"]}
        out["options"] = {"optimised": opt(*c["optimised"]), "typed": opt(*c["typed"])}
        out["saved_m"], out["saved_pct"] = round(c["saved_m"], 2), round(c["saved_pct"], 1)
        return out

    def api_trip(self, body):
        if self.count_publishers("/shopping_goal") > 1:
            raise ValueError("another node publishes /shopping_goal (is shopping_manager running?); close it first")
        goals, problems, p = self._goals(body)
        route = body.get("route")
        if route not in (None, "optimised", "typed"):
            raise ValueError(f"unknown route '{route}' (use optimised or typed)")
        strategy = {"optimised": self.strategy if self.strategy in ("exact", "nearest") else "exact",
                    "typed": "as_entered"}.get(route, body.get("strategy", self.strategy))
        ordered, info = self.plan(goals, p[:2], strategy)
        info["route"] = route or "default"
        info["problems"] = problems
        with self.lock:
            goal = self.trip.start(ordered, goals, p, info, self.now())
        self.send(goal)
        return self.api_status()

    def api_control(self, body):
        action = body.get("action")
        t = self.now()
        warn = None
        with self.lock:
            tr = self.trip
            if action == "next":
                goal = tr.next(t)
            elif action == "skip":
                goal = tr.skip(t)
            elif action == "pause":
                goal = tr.pause(t, "PAUSED")
            elif action == "stop":
                goal = tr.pause(t, "STOPPED")
            elif action == "resume":
                goal = tr.resume(t)
            elif action == "home":
                goal = tr.go_home(t)
            elif action == "reset":
                goal = tr.reset(t)
            else:
                raise ValueError(f"unknown action '{action}'")
        if action in ("pause", "stop"):
            warn = self.hold()
        else:
            self.send(goal)
        out = self.api_status()
        if warn:
            out["warning"] = warn
        return out


def make_handler(node):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=node.web_dir, **kw)

        def log_message(self, *a):
            pass

        def reply(self, code, obj):
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.startswith("/api/status"):
                return self.reply(200, node.api_status())
            if self.path.startswith("/api/products"):
                return self.reply(200, node.api_products())
            return super().do_GET()

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length", 0) or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                if self.path.startswith("/api/plan"):
                    return self.reply(200, node.api_plan(body))
                if self.path.startswith("/api/trip"):
                    return self.reply(200, node.api_trip(body))
                if self.path.startswith("/api/control"):
                    return self.reply(200, node.api_control(body))
                return self.reply(404, {"error": "unknown endpoint"})
            except ValueError as e:
                return self.reply(409, {"error": str(e)})
            except Exception as e:   # keep the server alive; report the error to the UI
                node.get_logger().error(f"{self.path}: {e!r}")
                return self.reply(500, {"error": repr(e)})
    return Handler


def main():
    rclpy.init()
    node = UiBridge()
    host, port = node.get_parameter("host").value, int(node.get_parameter("port").value)
    httpd = ThreadingHTTPServer((host, port), make_handler(node))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
