#!/usr/bin/env python3
"""Mock of arc_ui_bridge for browser tests without ROS: same HTTP API, same Trip logic,
a fake robot that drives toward the current goal at 0.25 m/s, and a stale 'goal reached'
sent right after every new goal (the latched-message race) to exercise the guard.

    python3 test/mock_bridge.py [port]   then open http://127.0.0.1:<port>
"""
import json
import math
import os
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from arc_ui_bridge import route_order as ro  # noqa: E402
from arc_ui_bridge.trip import Trip, resolve  # noqa: E402

ITEMS = yaml.safe_load(open(os.environ.get(
    "ARC_ITEM_LOCATIONS", os.path.expanduser("~/slam_ws/src/arc_store_sim/config/item_locations.yaml"))))["items"]
PMAP = yaml.safe_load(open(os.path.join(ROOT, "config", "product_map.yaml")))
GRID = ro.CostGrid([0] * (160 * 140), 160, 140, 0.05, -4.0, -3.0)
_DEST = yaml.safe_load(open(os.environ.get(
    "ARC_DESTINATIONS", os.path.expanduser("~/slam_ws/src/arc_store_sim/config/destinations_map.yaml"))))
_C = _DEST["destinations"]["checkout"]
CHECKOUT = {"item": "checkout", "items": ["checkout"], "x": float(_C["x"]), "y": float(_C["y"]),
            "yaw": float(_C.get("yaw", 0.0)), "is_checkout": True}


class Mock:
    def __init__(self):
        self.trip = Trip()
        self.lock = threading.Lock()
        self.pose = [0.0, 0.0, 0.0]
        self.goal = None
        self.t0 = time.time()
        threading.Thread(target=self.robot, daemon=True).start()

    def now(self):
        return time.time() - self.t0

    def send(self, g, stale=True):
        if g is None:
            return
        self.goal = (g["x"], g["y"])
        if stale:   # previous goal's latched 'reached' arrives immediately
            with self.lock:
                self.trip.arrival(tuple(self.pose[:2]), self.now())

    def robot(self):
        while True:
            time.sleep(0.1)
            with self.lock:
                st = self.trip.state
            if self.goal and st in ("NAVIGATING", "RETURNING"):
                dx, dy = self.goal[0] - self.pose[0], self.goal[1] - self.pose[1]
                d = math.hypot(dx, dy)
                if d < 0.15:
                    with self.lock:
                        self.trip.arrival(tuple(self.pose[:2]), self.now())
                    continue
                step = min(d, 0.025 * 4)
                self.pose[0] += step * dx / d
                self.pose[1] += step * dy / d
                self.pose[2] = math.atan2(dy, dx)

    def status(self):
        with self.lock:
            snap = self.trip.snapshot()
        return {"ros": True, "bt_mode": "LEAD", "route_choice": "ask", "checkout": [CHECKOUT["x"], CHECKOUT["y"]], "bt_age_s": 0.2, "pose": [round(v, 3) for v in self.pose],
                "localised": True, "map": True, "other_goal_publishers": 0, "trip": snap,
                "store_items": {k: [v["x"], v["y"]] for k, v in ITEMS.items()}}

    def api_plan(self, body):
        goals, problems = resolve(body.get("items", []), PMAP, ITEMS)
        c = ro.compare_routes(GRID, tuple(self.pose[:2]), goals, "exact", end_xy=(CHECKOUT["x"], CHECKOUT["y"]))

        def opt(stops, info):
            return {"strategy": info["strategy"], "cost": round(info["cost"], 2), "unreachable": info["unreachable"],
                    "stops": [" + ".join(dict.fromkeys(s.get("items", [s["item"]]))) for s in stops] + ["CHECKOUT"]}
        return {"ask": True, "problems": problems, "checkout": [CHECKOUT["x"], CHECKOUT["y"]],
                "options": {"optimised": opt(*c["optimised"]), "typed": opt(*c["typed"])},
                "saved_m": round(c["saved_m"], 2), "saved_pct": round(c["saved_pct"], 1)}

    def api_trip(self, body):
        goals, problems = resolve(body.get("items", []), PMAP, ITEMS)
        strat = {"optimised": "exact", "typed": "as_entered"}.get(body.get("route"), "exact")
        ordered, info = ro.plan_route(GRID, tuple(self.pose[:2]), goals, strat, end_xy=(CHECKOUT["x"], CHECKOUT["y"]))
        ordered = list(ordered) + [dict(CHECKOUT)]
        info = {k: (round(v, 2) if k.startswith("cost") else v) for k, v in info.items() if k != "start_cell"}
        info["route"], info["ends_at"] = body.get("route") or "default", "checkout"
        info["problems"] = problems
        with self.lock:
            g = self.trip.start(ordered, goals, tuple(self.pose), info, self.now())
        self.send(g)
        return self.status()

    def api_control(self, body):
        a = body.get("action")
        with self.lock:
            g = {"next": self.trip.next, "skip": self.trip.skip, "resume": self.trip.resume,
                 "home": self.trip.go_home, "reset": self.trip.reset}.get(a)
            if a in ("pause", "stop"):
                self.trip.pause(self.now(), "PAUSED" if a == "pause" else "STOPPED")
                g = None
            elif g is None:
                raise ValueError(f"unknown action {a}")
            else:
                g = g(self.now())
        if a in ("pause", "stop"):
            self.goal = tuple(self.pose[:2])
        else:
            self.send(g)
        return self.status()


M = Mock()


class H(SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=os.path.join(ROOT, "web"), **k)

    def log_message(self, *a):
        pass

    def reply(self, code, obj):
        d = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(d)))
        self.end_headers()
        self.wfile.write(d)

    def do_GET(self):
        if self.path.startswith("/api/status"):
            return self.reply(200, M.status())
        return super().do_GET()

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        try:
            if self.path.startswith("/api/plan"):
                return self.reply(200, M.api_plan(body))
            if self.path.startswith("/api/trip"):
                return self.reply(200, M.api_trip(body))
            return self.reply(200, M.api_control(body))
        except ValueError as e:
            return self.reply(409, {"error": str(e)})


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8099
    ThreadingHTTPServer(("127.0.0.1", port), H).serve_forever()
