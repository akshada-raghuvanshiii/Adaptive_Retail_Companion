#!/usr/bin/env python3
"""Generate the ARC supermarket sandbox for Gazebo Harmonic (gz-sim 8, ROS 2 Jazzy).

    python3 scripts/generate_store.py            # from the package root

Reads config/store_layout.json and writes:
  worlds/arc_store.sdf             store + drivable test shopper + overhead camera
  worlds/arc_store_normal.sdf      + one scripted shopper who finds items normally
  worlds/arc_store_searching.sdf   + one scripted shopper who walks past the item,
                                     stops, turns back and revisits before finding it
  worlds/arc_store_busy.sdf        + both of the above and three background people
  config/item_locations.yaml       item -> pick pose (for the Shopping Manager)
  maps/store_map.pgm/.yaml         ground-truth occupancy map, 5 cm cells
  scenarios/<world>_truth.csv      scripted actor positions at 10 Hz (one loop)
  scenarios/<world>_labels.csv     behaviour intervals per actor
  scenarios/scenarios.yaml         what each scenario should and should not trigger
  docs/layout.png                  top-down plan (needs matplotlib; skipped otherwise)

Standard library only (matplotlib optional). Everything in the worlds is built
from primitives except the scripted people, whose walking mesh is the standard
Gazebo Fuel actor and is downloaded by Gazebo on first run (needs internet).
"""

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAYOUT = json.loads((ROOT / "config" / "store_layout.json").read_text())

ACTOR_MESH = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae"
WALK_SPEED = 0.8          # m/s, a relaxed (older) shopper's pace
TURN_RATE = math.pi / 2   # rad/s when turning in place
PERSON_RADIUS = 0.3       # clearance used for path checks

PALETTE = [  # muted product colours (RGB 0-1)
    (0.86, 0.36, 0.25), (0.95, 0.70, 0.20), (0.30, 0.62, 0.36), (0.20, 0.47, 0.75),
    (0.62, 0.40, 0.70), (0.90, 0.55, 0.65), (0.35, 0.70, 0.75), (0.75, 0.75, 0.30),
]


# --------------------------------------------------------------------------- geometry
def rect(name, kind, x0, x1, y0, y1, h, color=(0.6, 0.6, 0.6)):
    return {"name": name, "kind": kind, "x0": min(x0, x1), "x1": max(x0, x1),
            "y0": min(y0, y1), "y1": max(y0, y1), "h": h, "color": color}


def build_obstacles():
    s = LAYOUT["store"]
    L, W, H, T = s["length_x"], s["width_y"], s["wall_height"], s["wall_thickness"]
    ex0, ex1 = s["entrance_x"]
    obs = [
        rect("wall_south_w", "wall", -T, ex0, -T, 0, H),
        rect("wall_south_e", "wall", ex1, L + T, -T, 0, H),
        rect("wall_north", "wall", -T, L + T, W, W + T, H),
        rect("wall_west", "wall", -T, 0, 0, W, H),
        rect("wall_east", "wall", L, L + T, 0, W, H),
    ]
    g = LAYOUT["gondola"]
    for i, cx in enumerate(g["centers_x"], 1):
        obs.append(rect(f"gondola_{i}", "gondola", cx - g["depth"] / 2, cx + g["depth"] / 2,
                        g["y_start"], g["y_end"], g["height"]))
    ws = LAYOUT["wall_shelf"]
    cat = LAYOUT["categories"]
    ey0, ey1 = cat["east_wall"]["y_range"]
    obs.append(rect("shelf_east", "wall_shelf", L - ws["depth"], L, ey0, ey1, ws["height"]))
    nx0, nx1 = cat["north_wall"]["x_range"]
    obs.append(rect("shelf_north", "wall_shelf", nx0, nx1, W - ws["depth"], W, ws["height"]))
    wy0, wy1 = cat["west_wall"]["y_range"]
    obs.append(rect("shelf_west", "wall_shelf", 0, ws["depth"], wy0, wy1, ws["height"]))
    c = LAYOUT["checkout"]
    for i, cx in enumerate(c["centers_x"], 1):
        obs.append(rect(f"checkout_{i}", "checkout", cx - c["width_x"] / 2, cx + c["width_x"] / 2,
                        c["y_range"][0], c["y_range"][1], c["height"], (0.45, 0.45, 0.5)))
    for t in LAYOUT["produce_tables"]:
        (cx, cy), (sx, sy) = t["center"], t["size"]
        obs.append(rect(f"table_{t['name']}", "table", cx - sx / 2, cx + sx / 2,
                        cy - sy / 2, cy + sy / 2, t["height"], (0.55, 0.40, 0.25)))
    return obs


def build_items():
    """Item name -> dict(x, y, yaw, shelf, face_*). Pick pose faces the shelf."""
    so = LAYOUT["pick_standoff"]
    g = LAYOUT["gondola"]
    cat = LAYOUT["categories"]
    L, W = LAYOUT["store"]["length_x"], LAYOUT["store"]["width_y"]
    ws = LAYOUT["wall_shelf"]["depth"]
    items = {}

    def segments(a, b, names):
        step = (b - a) / len(names)
        return [(n, a + step * (k + 0.5), a + step * k, a + step * (k + 1)) for k, n in enumerate(names)]

    for f in cat["gondola_faces"]:
        cx = g["centers_x"][f["gondola"] - 1]
        if f["face"] == "west":
            face_x, px, yaw = cx - g["depth"] / 2, cx - g["depth"] / 2 - so, 0.0
        else:
            face_x, px, yaw = cx + g["depth"] / 2, cx + g["depth"] / 2 + so, math.pi
        for name, mid, a, b in segments(g["y_start"], g["y_end"], f["items"]):
            items[name] = {"x": px, "y": mid, "yaw": yaw, "shelf": f"gondola_{f['gondola']}",
                           "face": f["face"], "span": [face_x, a, face_x, b]}
    y0, y1 = cat["east_wall"]["y_range"]
    for name, mid, a, b in segments(y0, y1, cat["east_wall"]["items"]):
        items[name] = {"x": L - ws - so, "y": mid, "yaw": 0.0, "shelf": "shelf_east", "face": "west",
                       "span": [L - ws, a, L - ws, b]}
    x0, x1 = cat["north_wall"]["x_range"]
    for name, mid, a, b in segments(x0, x1, cat["north_wall"]["items"]):
        items[name] = {"x": mid, "y": W - ws - so, "yaw": math.pi / 2, "shelf": "shelf_north",
                       "face": "south", "span": [a, W - ws, b, W - ws]}
    y0, y1 = cat["west_wall"]["y_range"]
    for name, mid, a, b in segments(y0, y1, cat["west_wall"]["items"]):
        items[name] = {"x": ws + so, "y": mid, "yaw": math.pi, "shelf": "shelf_west", "face": "east",
                       "span": [ws, a, ws, b]}
    for t in LAYOUT["produce_tables"]:
        (cx, cy), (sx, sy) = t["center"], t["size"]
        items[t["name"]] = {"x": cx - sx / 2 - so, "y": cy, "yaw": 0.0, "shelf": f"table_{t['name']}",
                            "face": "west", "span": [cx - sx / 2, cy - sy / 2, cx - sx / 2, cy + sy / 2]}
    return items


def color_for(name):
    return PALETTE[sum(ord(c) for c in name) % len(PALETTE)]


# --------------------------------------------------------------------------- SDF helpers
def f(v):
    return f"{v:.3f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def pose(x, y, z, r=0.0, p=0.0, yw=0.0):
    return f"<pose>{f(x)} {f(y)} {f(z)} {f(r)} {f(p)} {f(yw)}</pose>"


def material(c, emissive=False):
    r, g, b = c
    em = f"<emissive>{r*0.15:.2f} {g*0.15:.2f} {b*0.15:.2f} 1</emissive>" if emissive else ""
    return (f"<material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>"
            f"<diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse><specular>0.1 0.1 0.1 1</specular>{em}</material>")


def box_visual(name, x, y, z, sx, sy, sz, c):
    return (f'<visual name="{name}">{pose(x, y, z)}<geometry><box><size>{f(sx)} {f(sy)} {f(sz)}</size>'
            f"</box></geometry>{material(c)}</visual>")


def box_collision(name, x, y, z, sx, sy, sz):
    return (f'<collision name="{name}">{pose(x, y, z)}<geometry><box><size>{f(sx)} {f(sy)} {f(sz)}'
            f"</size></box></geometry></collision>")


def static_model(name, body):
    return f'<model name="{name}"><static>true</static><link name="link">{body}</link></model>'


def obstacle_model(o, items):
    """Walls, counters, tables are single boxes; shelves get boards and coloured product blocks."""
    cx, cy = (o["x0"] + o["x1"]) / 2, (o["y0"] + o["y1"]) / 2
    sx, sy, h = o["x1"] - o["x0"], o["y1"] - o["y0"], o["h"]
    body = box_collision("col", 0.0, 0.0, h / 2, sx, sy, h)
    if o["kind"] in ("wall", "checkout", "table"):
        col = (0.88, 0.87, 0.84) if o["kind"] == "wall" else o["color"]
        body += box_visual("vis", 0.0, 0.0, h / 2, sx, sy, h, col)
        if o["kind"] == "table":  # produce heaped on top
            body += box_visual("produce", 0.0, 0.0, h + 0.08, sx * 0.9, sy * 0.9, 0.16,
                               color_for(o["name"]))
        return f'<model name="{o["name"]}"><static>true</static>{pose(cx, cy, 0)}<link name="link">{body}</link></model>'

    # shelving unit
    along_y = sy > sx
    length = sy if along_y else sx
    depth = sx if along_y else sy
    grey, board = (0.70, 0.71, 0.73), (0.82, 0.83, 0.85)
    panel = (0.06, length, h) if along_y else (length, 0.06, h)
    body += box_visual("plinth", 0.0, 0.0, 0.06, sx, sy, 0.12, (0.35, 0.36, 0.38))
    levels = LAYOUT["gondola"]["shelf_levels"]
    double = o["kind"] == "gondola"
    if o["name"] == "shelf_north":
        body += box_visual("panel", 0.0, depth / 2 - 0.03, h / 2, *panel, grey)
    elif o["name"] == "shelf_east":
        body += box_visual("panel", depth / 2 - 0.03, 0.0, h / 2, *panel, grey)
    elif o["name"] == "shelf_west":
        body += box_visual("panel", -(depth / 2 - 0.03), 0.0, h / 2, *panel, grey)
    else:
        body += box_visual("panel", 0.0, 0.0, h / 2, *panel, grey)
    for k, z in enumerate(levels):
        body += box_visual(f"board_{k}", 0.0, 0.0, z, sx, sy, 0.03, board)
    # product blocks for each item segment on this unit
    for name, it in items.items():
        if it["shelf"] != o["name"]:
            continue
        x_a, y_a, x_b, y_b = it["span"]
        seg = abs((y_b - y_a) if along_y else (x_b - x_a)) - 0.1
        mid = ((y_a + y_b) / 2 - cy) if along_y else ((x_a + x_b) / 2 - cx)
        pd = (depth / 2 if double else depth) - 0.12  # product depth from face
        face_off = {"west": -1, "east": 1, "south": -1, "north": 1}[it["face"]]
        for k, z in enumerate(levels):
            if z > h - 0.25:
                continue
            ph = 0.3 if k < len(levels) - 1 else 0.22
            if along_y:
                if double:
                    px = face_off * (depth / 2 - pd / 2 - 0.02)
                else:
                    px = face_off * (depth / 2 - pd / 2 - 0.02)
                body += box_visual(f"{name}_{k}", px, mid, z + 0.015 + ph / 2, pd, seg, ph, color_for(name))
            else:
                py = face_off * (depth / 2 - pd / 2 - 0.02)
                body += box_visual(f"{name}_{k}", mid, py, z + 0.015 + ph / 2, seg, pd, ph, color_for(name))
    return f'<model name="{o["name"]}"><static>true</static>{pose(cx, cy, 0)}<link name="link">{body}</link></model>'


def lights():
    out = ['<light type="directional" name="sun"><cast_shadows>true</cast_shadows>'
           '<pose>12 8 10 0 0 0</pose><diffuse>0.75 0.75 0.75 1</diffuse><specular>0.2 0.2 0.2 1</specular>'
           '<direction>-0.3 0.2 -0.9</direction></light>']
    k = 0
    for x in (5.0, 12.0, 19.0):
        for y in (4.0, 11.0):
            k += 1
            out.append(f'<light type="point" name="ceiling_{k}">{pose(x, y, 3.3)}'
                       '<diffuse>0.55 0.55 0.52 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
                       '<attenuation><range>14</range><constant>0.6</constant><linear>0.05</linear>'
                       '<quadratic>0.005</quadratic></attenuation><cast_shadows>false</cast_shadows></light>')
    return "".join(out)


def ground():
    L, W = LAYOUT["store"]["length_x"], LAYOUT["store"]["width_y"]
    return ('<model name="floor"><static>true</static><link name="link">'
            '<collision name="col"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision>'
            f'<visual name="outside">{pose(L / 2, W / 2, -0.002)}<geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>{material((0.5, 0.52, 0.5))}</visual>'
            f'<visual name="tiles">{pose(L / 2, W / 2, 0.001)}<geometry><box><size>{f(L)} {f(W)} 0.002</size></box></geometry>{material((0.86, 0.85, 0.80))}</visual>'
            '</link></model>')


def overhead_camera():
    c = LAYOUT["overhead_camera"]
    x, y, z = c["position"]
    return (f'<model name="overhead_cam"><static>true</static>{pose(x, y, z, 0.0, math.pi / 2, 0.0)}'
            '<link name="link"><sensor name="overhead_cam" type="camera"><always_on>1</always_on>'
            f'<update_rate>{c["rate_hz"]}</update_rate><visualize>true</visualize><topic>{c["topic"]}</topic>'
            f'<camera><horizontal_fov>{c["hfov"]}</horizontal_fov><image><width>{c["width"]}</width>'
            f'<height>{c["height"]}</height><format>R8G8B8</format></image>'
            '<clip><near>0.1</near><far>30</far></clip></camera></sensor></link></model>')


def drivable_shopper():
    """Primitive 'mannequin' you drive with /model/shopper/cmd_vel; publishes true odometry.
    It is for testing following/leading logic with known ground truth. YOLO is not
    expected to detect it reliably -- use the scripted actors for perception tests."""
    x, y, yaw = LAYOUT["shopper_spawn"]
    return (f'<model name="shopper">{pose(x, y, 0.0, 0, 0, yaw)}<link name="base">'
            '<inertial><mass>70</mass><pose>0 0 0.85 0 0 0</pose><inertia><ixx>60</ixx><iyy>60</iyy>'
            '<izz>2.2</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<collision name="col">{pose(0, 0, 0.85)}<geometry><cylinder><radius>0.25</radius><length>1.7</length></cylinder></geometry>'
            '<surface><friction><ode><mu>0</mu><mu2>0</mu2></ode><bullet><friction>0</friction><friction2>0</friction2></bullet></friction></surface></collision>'
            f'<visual name="legs">{pose(0, 0, 0.43)}<geometry><cylinder><radius>0.17</radius><length>0.86</length></cylinder></geometry>{material((0.15, 0.2, 0.35))}</visual>'
            f'<visual name="torso">{pose(0, 0, 1.17)}<geometry><box><size>0.28 0.46 0.62</size></box></geometry>{material((0.75, 0.25, 0.2))}</visual>'
            f'<visual name="head">{pose(0, 0, 1.6)}<geometry><sphere><radius>0.12</radius></sphere></geometry>{material((0.85, 0.68, 0.55))}</visual>'
            f'<visual name="nose">{pose(0.12, 0, 1.6)}<geometry><box><size>0.04 0.04 0.04</size></box></geometry>{material((0.2, 0.2, 0.2))}</visual>'
            '</link>'
            '<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>'
            '<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
            '<odom_frame>shopper_odom</odom_frame><robot_base_frame>shopper</robot_base_frame>'
            '<odom_publish_frequency>30</odom_publish_frequency><dimensions>2</dimensions></plugin>'
            '</model>')


# --------------------------------------------------------------------------- scripted shoppers
class Script:
    """Build an actor trajectory as straight segments, turns in place and pauses,
    while recording labelled time intervals for ground truth."""

    def __init__(self, name, x, y, yaw):
        self.name = name
        self.t, self.x, self.y, self.yaw = 0.0, x, y, yaw
        self.wps = [(0.0, x, y, yaw)]
        self.labels = []
        self.phase = "walk"

    def _add(self, dt, x, y, yaw, label):
        t0 = self.t
        self.t += dt
        self.x, self.y, self.yaw = x, y, yaw
        self.wps.append((self.t, x, y, yaw))
        if self.labels and self.labels[-1][2] == label and abs(self.labels[-1][1] - t0) < 1e-9:
            self.labels[-1] = (self.labels[-1][0], self.t, label)
        else:
            self.labels.append((t0, self.t, label))

    def set_phase(self, label):
        self.phase = label
        return self

    def face(self, yaw):
        d = (yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(d) > 0.05:
            self._add(abs(d) / TURN_RATE, self.x, self.y, self.yaw + d, self.phase)
        return self

    def go(self, x, y, speed=WALK_SPEED):
        dist = math.hypot(x - self.x, y - self.y)
        if dist < 1e-6:
            return self
        self.face(math.atan2(y - self.y, x - self.x))
        self._add(dist / speed, x, y, self.yaw, self.phase)
        return self

    def go_item(self, item):
        self.go(item["x"], item["y"])
        return self.face(item["yaw"])

    def pause(self, seconds, label):
        self._add(seconds, self.x, self.y, self.yaw, label)
        return self

    def close_loop(self, x, y, yaw):
        self.go(x, y)
        return self.face(yaw)

    @property
    def cycle(self):
        return self.wps[-1][0]

    def sdf(self):
        w = "".join(f"<waypoint><time>{t:.2f}</time>{pose(x, y, 1.0, 0, 0, yaw)}</waypoint>"
                    for t, x, y, yaw in self.wps)
        return (f'<actor name="{self.name}"><skin><filename>{ACTOR_MESH}</filename><scale>1.0</scale></skin>'
                f'<animation name="walking"><filename>{ACTOR_MESH}</filename><interpolate_x>true</interpolate_x></animation>'
                '<script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>'
                f'<trajectory id="0" type="walking" tension="1">{w}</trajectory></script></actor>')

    def sample(self, hz=10.0):
        """Positions as Gazebo plays them: tension=1 gives straight segments with
        ease-in/out (cubic Hermite, zero tangents) between waypoints."""
        out = []
        n = int(self.cycle * hz) + 1
        k = 0
        for i in range(n):
            t = i / hz
            while k < len(self.wps) - 2 and self.wps[k + 1][0] < t:
                k += 1
            t0, x0, y0, a0 = self.wps[k]
            t1, x1, y1, a1 = self.wps[k + 1]
            u = 0.0 if t1 <= t0 else min(1.0, max(0.0, (t - t0) / (t1 - t0)))
            h = 3 * u * u - 2 * u * u * u
            label = next((lb for s, e, lb in self.labels if s <= t <= e), self.labels[-1][2])
            out.append((round(t, 2), x0 + (x1 - x0) * h, y0 + (y1 - y0) * h, a0 + (a1 - a0) * h, label))
        return out


def scenario_actors(items):
    start = (2.5, 0.5, math.pi / 2)
    front_y, back_y = 3.8, 14.2
    aisle = {0: 4.6, 1: 7.8, 2: 11.4, 3: 15.0, 4: 18.6, 5: 22.2}
    c = LAYOUT["checkout"]
    lanes = [(c["centers_x"][i] + c["centers_x"][i + 1]) / 2 for i in range(len(c["centers_x"]) - 1)]
    lane_y = (c["y_range"][0] + c["y_range"][1]) / 2

    def checkout_and_leave(s, lane_x):
        s.set_phase("exit").go(lane_x, front_y).go(lane_x, lane_y).pause(8, "pause_checkout")
        s.go(lane_x, 0.5).go(start[0], 0.5)
        return s.close_loop(*start)

    normal = Script("shopper_normal", *start).set_phase("walk_to_item")
    normal.go(start[0], front_y).go(aisle[1], front_y).go_item(items["biscuits"]).pause(6, "pause_at_item")
    normal.set_phase("walk_to_item").go(aisle[1], back_y).go_item(items["milk"]).pause(5, "pause_at_item")
    normal.set_phase("walk_to_item").go(aisle[2], back_y).go(aisle[2], front_y)
    checkout_and_leave(normal, lanes[0])

    target = items["pasta"]
    srch = Script("shopper_searching", *start).set_phase("walk_to_item")
    srch.go(start[0], front_y).go(aisle[3], front_y).go(aisle[3], target["y"])
    srch.set_phase("searching_walk").go(aisle[3], 12.4).pause(9, "searching_pause")
    srch.set_phase("searching_walk").go(aisle[3], 9.6).pause(6, "searching_pause")
    srch.set_phase("searching_walk").go(aisle[3], back_y).go(aisle[4], back_y).go(aisle[4], 8.0)
    srch.pause(7, "searching_pause")
    srch.set_phase("searching_walk").go(aisle[4], back_y).go(aisle[3], back_y).go(aisle[3], target["y"])
    srch.go_item(target).pause(6, "pause_at_item")
    srch.set_phase("walk_to_item").go(aisle[3], target["y"]).go(aisle[3], front_y)
    checkout_and_leave(srch, lanes[1])

    browse = Script("shopper_browsing", aisle[2], 4.4, math.pi / 2).set_phase("browsing_walk")
    browse.go_item(items["rice"]).pause(5, "pause_at_item")
    browse.set_phase("browsing_walk").go(aisle[2], items["atta_flour"]["y"]).go_item(items["atta_flour"])
    browse.pause(6, "pause_at_item")
    browse.set_phase("browsing_walk").go(aisle[2], items["dal_pulses"]["y"]).go_item(items["dal_pulses"])
    browse.pause(4, "pause_at_item").set_phase("browsing_walk").close_loop(aisle[2], 4.4, math.pi / 2)

    walker = Script("passer_by", aisle[0], front_y, math.pi / 2).set_phase("walk_through")
    walker.go(aisle[0], back_y).go(aisle[5], back_y).go(aisle[5], front_y).close_loop(aisle[0], front_y, math.pi / 2)

    sx = c["centers_x"][0] - c["width_x"] / 2 - 0.4
    staff = Script("staff_checkout", sx, lane_y - 0.2, 0.0).set_phase("staff_idle")
    staff.pause(20, "staff_idle").go(sx, lane_y + 0.3).pause(20, "staff_idle").close_loop(sx, lane_y - 0.2, 0.0)

    return {"normal": [normal], "searching": [srch],
            "busy": [normal, srch, browse, walker, staff]}


# --------------------------------------------------------------------------- outputs
def world_sdf(name, obstacles, items, actors):
    parts = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.9">',
        f'<world name="{name}">',
        '<physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size>'
        '<real_time_factor>1.0</real_time_factor></physics>',
        '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
        '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
        '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
        '<render_engine>ogre2</render_engine></plugin>',
        '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>',
        '<gravity>0 0 -9.8</gravity>',
        '<scene><ambient>0.55 0.55 0.55 1</ambient><background>0.75 0.8 0.85 1</background>'
        '<shadows>true</shadows><grid>false</grid></scene>',
        lights(), ground(),
    ]
    parts += [obstacle_model(o, items) for o in obstacles]
    parts += [overhead_camera(), drivable_shopper()]
    parts += [a.sdf() for a in actors]
    parts += ["</world>", "</sdf>"]
    return "\n".join(parts) + "\n"


def write_items_yaml(items, path):
    lines = ["# Generated by scripts/generate_store.py from config/store_layout.json.",
             "# Pick pose = where a shopper (or the cart) stands to take the item, facing the shelf.",
             "# Frame: world (x east, y north, origin at the store's south-west inner corner).",
             "frame_id: world", "items:"]
    for name in sorted(items):
        it = items[name]
        lines.append(f"  {name}: {{x: {it['x']:.2f}, y: {it['y']:.2f}, yaw: {it['yaw']:.4f}, "
                     f"shelf: {it['shelf']}, face: {it['face']}}}")
    path.write_text("\n".join(lines) + "\n")


def write_map(obstacles, res=0.05, pad=0.5):
    L, W = LAYOUT["store"]["length_x"], LAYOUT["store"]["width_y"]
    nx, ny = int(round((L + 2 * pad) / res)), int(round((W + 2 * pad) / res))
    grid = bytearray([254]) * (nx * ny)
    for o in obstacles:
        i0, i1 = int((o["x0"] + pad) / res), int(math.ceil((o["x1"] + pad) / res))
        j0, j1 = int((o["y0"] + pad) / res), int(math.ceil((o["y1"] + pad) / res))
        for j in range(max(0, j0), min(ny, j1)):
            row = (ny - 1 - j) * nx  # PGM rows go top-down
            for i in range(max(0, i0), min(nx, i1)):
                grid[row + i] = 0
    mdir = ROOT / "maps"
    mdir.mkdir(exist_ok=True)
    (mdir / "store_map.pgm").write_bytes(f"P5\n{nx} {ny}\n255\n".encode() + bytes(grid))
    (mdir / "store_map.yaml").write_text(
        "image: store_map.pgm\nmode: trinary\n"
        f"resolution: {res}\norigin: [{-pad}, {-pad}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return nx, ny


def write_scenarios(actor_sets):
    sdir = ROOT / "scenarios"
    sdir.mkdir(exist_ok=True)
    for key, actors in actor_sets.items():
        with (sdir / f"arc_store_{key}_truth.csv").open("w") as fh:
            fh.write("actor,t,x,y,yaw,label\n")
            for a in actors:
                for t, x, y, yaw, lb in a.sample():
                    fh.write(f"{a.name},{t:.2f},{x:.3f},{y:.3f},{yaw:.3f},{lb}\n")
        with (sdir / f"arc_store_{key}_labels.csv").open("w") as fh:
            fh.write("actor,t_start,t_end,label,cycle_s\n")
            for a in actors:
                for s, e, lb in a.labels:
                    fh.write(f"{a.name},{s:.2f},{e:.2f},{lb},{a.cycle:.2f}\n")
    srch = actor_sets["searching"][0]
    first = next(s for s, e, lb in srch.labels if lb.startswith("searching"))
    found = next(s for s, e, lb in srch.labels if lb == "pause_at_item" and s > first)
    cycles = {a.name: round(a.cycle, 2) for a in actor_sets["busy"]}
    (sdir / "scenarios.yaml").write_text(
        "# Expected outcomes for the help-offer trigger. Times are sim seconds from world start;\n"
        "# each actor loops with the period given in cycle_s, so use t mod cycle_s.\n"
        "# These are scripted, observable movement patterns, not a model of anyone's state of mind.\n"
        "actors:\n"
        f"  shopper_normal:    {{cycle_s: {cycles['shopper_normal']}, target_items: [biscuits, milk], "
        "help_offer_expected: false}\n"
        f"  shopper_searching: {{cycle_s: {cycles['shopper_searching']}, target_items: [pasta], "
        f"help_offer_expected: true, no_progress_window_s: [{first:.2f}, {found:.2f}]}}\n"
        f"  shopper_browsing:  {{cycle_s: {cycles['shopper_browsing']}, target_items: [rice, atta_flour, dal_pulses], "
        "help_offer_expected: false, note: 'hard negative: walks back and forth but stops at items'}\n"
        f"  passer_by:         {{cycle_s: {cycles['passer_by']}, help_offer_expected: false}}\n"
        f"  staff_checkout:    {{cycle_s: {cycles['staff_checkout']}, help_offer_expected: false, "
        "note: 'long stops by design; not a shopper'}\n"
        "worlds:\n"
        "  arc_store.sdf: [drivable shopper only]\n"
        "  arc_store_normal.sdf: [shopper_normal]\n"
        "  arc_store_searching.sdf: [shopper_searching]\n"
        "  arc_store_busy.sdf: [shopper_normal, shopper_searching, shopper_browsing, passer_by, staff_checkout]\n")


def draw_layout(obstacles, items, actor_sets, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        print("matplotlib not available; skipping docs/layout.png")
        return
    fig, ax = plt.subplots(figsize=(12, 8.6))
    kc = {"wall": "#9a9994", "gondola": "#b9b8b2", "wall_shelf": "#b9b8b2",
          "checkout": "#7a7f8f", "table": "#a8865f"}
    for o in obstacles:
        ax.add_patch(Rectangle((o["x0"], o["y0"]), o["x1"] - o["x0"], o["y1"] - o["y0"],
                               color=kc[o["kind"]], zorder=1))
    for name, it in items.items():
        ax.plot(it["x"], it["y"], "o", color="#0b0b0b", ms=2.5, zorder=4)
        if abs(it["yaw"] - math.pi) < 0.1:      # shelf to the west: label toward it
            tx, ha = it["x"] - 0.12, "right"
        elif abs(it["yaw"]) < 0.1:              # shelf to the east
            tx, ha = it["x"] + 0.12, "left"
        else:
            tx, ha = it["x"], "center"
        ax.text(tx, it["y"] + 0.15, name.replace("_", " "), fontsize=5.5, ha=ha, color="#52514e", zorder=4)
    cols = {"shopper_normal": "#2a78d6", "shopper_searching": "#eb6834", "shopper_browsing": "#1baf7a",
            "passer_by": "#6250d6", "staff_checkout": "#eda100"}
    for a in actor_sets["busy"]:
        pts = a.sample(5)
        ax.plot([p[1] for p in pts], [p[2] for p in pts], "-", lw=1.4, color=cols[a.name],
                label=a.name.replace("_", " "), zorder=3, alpha=0.9)
        for s, e, lb in a.labels:
            if "pause" in lb or lb == "staff_idle":
                p = next(q for q in pts if q[0] >= s)
                ax.plot(p[1], p[2], "s", color=cols[a.name], ms=5, zorder=5,
                        mec="white" if lb != "searching_pause" else "#0b0b0b")
    c = LAYOUT["overhead_camera"]
    x, y, z = c["position"]
    hw = z * math.tan(c["hfov"] / 2)
    hh = hw * c["height"] / c["width"]
    ax.add_patch(Rectangle((x - hh, y - hw), 2 * hh, 2 * hw, fill=False, ls="--", ec="#e34948", lw=1, zorder=2))
    ax.text(x - hh + 0.1, y + hw - 0.35, "overhead camera view", fontsize=7, color="#e34948")
    sx, sy, _ = LAYOUT["shopper_spawn"]
    ax.plot(sx, sy, "^", color="#e34948", ms=8, zorder=6)
    ax.text(sx + 0.25, sy, "drivable shopper", fontsize=7, color="#e34948", va="center")
    for k, ax_x in enumerate((4.6, 7.8, 11.4, 15.0, 18.6, 22.2)):
        ax.text(ax_x, 4.55, f"A{k}", fontsize=8, ha="center", weight="bold", color="#52514e")
    ax.text(2.5, -0.6, "entrance", ha="center", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlim(-0.8, 24.8)
    ax.set_ylim(-1.0, 16.6)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("ARC store sandbox: layout, pick poses (dots), scripted shoppers (busy world); "
                 "squares = pauses, black-edged = searching stops", fontsize=9, loc="left")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8, frameon=False)
    ax.grid(color="#e6e5e0", lw=0.5)
    fig.tight_layout()
    path.parent.mkdir(exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    obstacles = build_obstacles()
    items = build_items()
    actor_sets = scenario_actors(items)
    wdir = ROOT / "worlds"
    wdir.mkdir(exist_ok=True)
    worlds = {"arc_store": [], "arc_store_normal": actor_sets["normal"],
              "arc_store_searching": actor_sets["searching"], "arc_store_busy": actor_sets["busy"]}
    for name, actors in worlds.items():
        (wdir / f"{name}.sdf").write_text(world_sdf(name, obstacles, items, actors))
    write_items_yaml(items, ROOT / "config" / "item_locations.yaml")
    nx, ny = write_map(obstacles)
    write_scenarios(actor_sets)
    draw_layout(obstacles, items, actor_sets, ROOT / "docs" / "layout.png")
    print(f"worlds: {', '.join(worlds)}; items: {len(items)}; map: {nx}x{ny} cells")
    for a in actor_sets["busy"]:
        print(f"  {a.name}: {len(a.wps)} waypoints, loop {a.cycle:.1f} s")


if __name__ == "__main__":
    main()
