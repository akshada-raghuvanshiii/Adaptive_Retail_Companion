"""Shopping-trip logic behind the ARC UI. No ROS code here, so it can be unit-tested.

The UI sends product ids and quantities. Each product maps to an item spot of the simulated
store (config/product_map.yaml -> arc_store_sim item_locations.yaml); products on the same
spot become one stop. The trip then walks through the stops one at a time:

    IDLE -> NAVIGATING -> AT_STOP --(next)--> NAVIGATING ... -> CHECKOUT reached -> DONE
                 |  ^                                         \\-> RETURNING -> DONE
          pause  v  | resume                                 (RETURN HOME, or no checkout set)
                PAUSED          stop -> STOPPED (resume continues the same stop)

Checkout: when the trip has a checkout stop (is_checkout, always the last stop), arriving there
finishes the trip. Without one, the trip returns to where it started, as before.

Arrival guard (the goal-reached race found in the bench HIL test): ARC's hand-over
publishes /navigation_goal_reached = True for the previous goal as well, and the topic is
latched. An arrival therefore only counts if it comes at least `min_leg_s` after the goal
was sent AND the robot is within `arrive_m` of the current stop.
"""
import math

STATES = ("IDLE", "NAVIGATING", "AT_STOP", "PAUSED", "STOPPED", "RETURNING", "DONE")


def resolve(products, product_map, item_locations):
    """UI request [{item_id, quantity, name}] -> (goals for route_order, problems list)."""
    goals, problems = [], []
    for p in products:
        pid = p.get("item_id") or p.get("id")
        item = product_map.get(pid)
        if item is None:
            problems.append(f"{pid}: not in product_map.yaml")
            continue
        loc = item_locations.get(item)
        if loc is None:
            problems.append(f"{pid}: store item '{item}' not in item_locations.yaml")
            continue
        label = p.get("name") or pid
        goals.append({"item": item, "x": float(loc["x"]), "y": float(loc["y"]),
                      "yaw": float(loc.get("yaw", 0.0)), "shelf_name": str(loc.get("shelf", "")),
                      "products": [{"id": pid, "name": label, "qty": int(p.get("quantity", 1) or 1)}]})
    # same store item asked for by several products -> one goal carrying all products
    merged = {}
    for g in goals:
        if g["item"] in merged:
            merged[g["item"]]["products"] += g["products"]
        else:
            merged[g["item"]] = g
    return list(merged.values()), problems


def collect_products(stop, goals):
    """route_order merges goals on the same spot; gather the UI products of every merged item."""
    out = []
    for name in stop.get("items", [stop["item"]]):
        for g in goals:
            if g["item"] == name:
                out += g["products"]
    return out


class Trip:
    def __init__(self, arrive_m=1.0, min_leg_s=0.5):
        self.arrive_m, self.min_leg_s = arrive_m, min_leg_s
        self.state = "IDLE"
        self.stops, self.index = [], -1
        self.home = None
        self.goal_sent_t = None
        self.info = {}
        self.events = []
        self.ignored_arrivals = 0

    # ------------------------------------------------------------------ helpers
    def log(self, t, text):
        self.events.append({"t": round(t, 1), "text": text})
        self.events = self.events[-30:]

    def current(self):
        if self.state == "RETURNING":
            return self.home
        if 0 <= self.index < len(self.stops):
            return self.stops[self.index]
        return None

    def snapshot(self):
        cur = self.current()
        return {"state": self.state, "index": self.index, "info": self.info,
                "current": cur["label"] if cur else None,
                "stops": [dict({k: s[k] for k in ("label", "item", "x", "y", "status", "products")},
                               is_checkout=bool(s.get("is_checkout"))) for s in self.stops],
                "to_checkout": bool(cur and cur.get("is_checkout")),
                "home": self.home, "ignored_arrivals": self.ignored_arrivals, "events": self.events[-12:]}

    # ------------------------------------------------------------------ commands (return the goal to send, or None)
    def start(self, ordered_stops, goals, home_xy, info, t):
        if self.state in ("NAVIGATING", "AT_STOP", "PAUSED", "RETURNING"):
            raise ValueError("a trip is already running; finish or stop it first")
        self.stops = []
        for s in ordered_stops:
            co = bool(s.get("is_checkout"))
            prods = [] if co else collect_products(s, goals)
            self.stops.append({"label": "CHECKOUT" if co else " + ".join(dict.fromkeys(s.get("items", [s["item"]]))),
                               "item": s["item"], "x": s["x"], "y": s["y"], "yaw": s.get("yaw", 0.0),
                               "status": "pending", "products": prods, "is_checkout": co})
        if not any(not s["is_checkout"] for s in self.stops):
            raise ValueError("no reachable stops")
        if any(s["is_checkout"] for s in self.stops[:-1]):
            raise ValueError("the checkout must be the last stop")
        self.home = {"label": "HOME", "item": "home", "x": home_xy[0], "y": home_xy[1],
                     "yaw": home_xy[2] if len(home_xy) > 2 else 0.0, "status": "pending", "products": []}
        self.info = info
        self.index = 0
        self.ignored_arrivals = 0
        self.events = []
        n = sum(1 for s in self.stops if not s["is_checkout"])
        co = " + checkout" if self.stops[-1]["is_checkout"] else ""
        self.log(t, f"trip started: {n} stop{'s' if n != 1 else ''}{co} ({info.get('strategy', '?')} order)")
        return self._go(t)

    def _go(self, t):
        cur = self.current()
        if self.state != "RETURNING":
            self.state = "NAVIGATING"
        cur["status"] = "active"
        self.goal_sent_t = t
        self.log(t, f"goal -> {cur['label']} ({cur['x']:.2f}, {cur['y']:.2f})")
        return cur

    def arrival(self, robot_xy, t):
        """A /navigation_goal_reached=True arrived. Returns True if it is accepted."""
        cur = self.current()
        if self.state not in ("NAVIGATING", "RETURNING") or cur is None:
            return False
        too_soon = self.goal_sent_t is None or t - self.goal_sent_t < self.min_leg_s
        far = robot_xy is None or math.hypot(robot_xy[0] - cur["x"], robot_xy[1] - cur["y"]) > self.arrive_m
        if too_soon or far:
            self.ignored_arrivals += 1
            why = "too soon after the goal" if too_soon else (
                "no robot pose" if robot_xy is None else
                f"robot {math.hypot(robot_xy[0] - cur['x'], robot_xy[1] - cur['y']):.2f} m from the stop")
            self.log(t, f"ignored 'goal reached' ({why})")
            return False
        cur["status"] = "reached"
        if self.state == "RETURNING":
            self.state = "DONE"
            self.log(t, "back home: trip finished")
        elif cur.get("is_checkout"):
            self.state = "DONE"
            self.log(t, "at the checkout counter: trip finished")
        else:
            self.state = "AT_STOP"
            self.log(t, f"arrived at {cur['label']}")
        return True

    def next(self, t):
        if self.state != "AT_STOP":
            raise ValueError("NEXT only works while waiting at a stop")
        self.index += 1
        if self.index >= len(self.stops):
            self.state = "RETURNING"
            self.log(t, "all items collected: returning home")
        elif self.stops[self.index].get("is_checkout"):
            self.log(t, "all items collected: going to the checkout")
        return self._go(t)

    def skip(self, t):
        if self.state not in ("NAVIGATING", "PAUSED", "STOPPED") or self.current() is None or self.state == "RETURNING":
            raise ValueError("nothing to skip")
        cur = self.current()
        cur["status"] = "skipped"
        self.log(t, f"skipped {cur['label']}")
        if cur.get("is_checkout"):                 # skipping the checkout ends the trip where it is
            self.state = "DONE"
            self.log(t, "checkout skipped: trip finished")
            return None
        self.state = "AT_STOP"
        return self.next(t)

    def pause(self, t, kind="PAUSED"):
        if self.state not in ("NAVIGATING", "RETURNING", "AT_STOP"):
            raise ValueError(f"cannot {kind.lower()} while {self.state}")
        self._resume_to = self.state
        self.state = kind
        self.log(t, "paused" if kind == "PAUSED" else "software stop")
        return None   # caller holds the robot at its current pose

    def resume(self, t):
        if self.state not in ("PAUSED", "STOPPED"):
            raise ValueError("nothing to resume")
        prev = getattr(self, "_resume_to", "NAVIGATING")
        self.log(t, "resumed")
        if prev == "AT_STOP":
            self.state = "AT_STOP"
            return None
        self.state = prev
        return self._go(t)

    def go_home(self, t):
        if self.state in ("IDLE", "DONE") or self.home is None:
            raise ValueError("no trip to return from")
        for s in self.stops:
            if s["status"] in ("pending", "active"):
                s["status"] = "skipped"
        self.state = "RETURNING"
        self.log(t, "return home requested")
        return self._go(t)

    def reset(self, t):
        if self.state in ("NAVIGATING", "RETURNING"):
            raise ValueError("stop or finish the trip before clearing it")
        self.__init__(self.arrive_m, self.min_leg_s)
        self.log(t, "trip cleared")
