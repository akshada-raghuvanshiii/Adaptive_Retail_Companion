"""Shared, ROS-free logic for the ARC test kit.

Item loading, occupancy-grid Dijkstra, visit-order strategies and the exact
(Held-Karp) solver. Nothing here imports rclpy, so it can be unit-tested and
used by the offline benchmark without a running ROS graph.
"""
import glob
import heapq
import json
import math
import os
import random

import numpy as np
import yaml

# --------------------------------------------------------------------------
# Item locations
# --------------------------------------------------------------------------

# The Shopping Manager reads exactly this file, so the tests use it too.
ARC_ITEM_FILE = "~/slam_ws/src/arc_store_sim/config/item_locations.yaml"

ITEM_FILE_CANDIDATES = [
    "~/slam_ws/src/**/item_locations.yaml",
    "~/arc_compact_store_kit/**/item_locations.yaml",
]


def find_item_file():
    """The file shopping_manager.py loads; else the newest item_locations.yaml found."""
    if os.path.exists(os.path.expanduser(ARC_ITEM_FILE)):
        return os.path.expanduser(ARC_ITEM_FILE)
    hits = []
    for pat in ITEM_FILE_CANDIDATES:
        hits += glob.glob(os.path.expanduser(pat), recursive=True)
    hits = [h for h in hits if "backup" not in h and "/build/" not in h
            and "/install/" not in h]
    if not hits:
        return None
    return max(hits, key=os.path.getmtime)


def _xy_from(entry):
    """Pull (x, y, yaw) out of an item entry, whatever its layout."""
    if isinstance(entry, (list, tuple)) and len(entry) >= 2 and all(
            isinstance(v, (int, float)) for v in entry[:2]):
        yaw = float(entry[2]) if len(entry) > 2 and isinstance(entry[2], (int, float)) else 0.0
        return float(entry[0]), float(entry[1]), yaw
    if not isinstance(entry, dict):
        return None
    if "x" in entry and "y" in entry:
        yaw = entry.get("yaw", entry.get("theta", 0.0)) or 0.0
        return float(entry["x"]), float(entry["y"]), float(yaw)
    for key in ("position", "pose", "goal", "location", "xy"):
        if key in entry:
            got = _xy_from(entry[key])
            if got:
                if "yaw" in entry and isinstance(entry["yaw"], (int, float)):
                    got = (got[0], got[1], float(entry["yaw"]))
                return got
    return None


def load_items(path):
    """Return a list of dicts: name, x, y, yaw, stop (stop may be None).

    Accepts a mapping name -> entry, a list of entries with a 'name' field,
    or either of those under a top-level key such as 'items'.
    """
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f)
    # unwrap a single wrapper key such as {"items": ...} or {"frame": "map", "items": ...}
    if isinstance(data, dict):
        for key in ("items", "item_locations", "products", "locations"):
            if key in data and isinstance(data[key], (dict, list)):
                data = data[key]
                break
    items = []
    if isinstance(data, dict):
        iterable = [(k, v) for k, v in data.items()]
    elif isinstance(data, list):
        iterable = [(e.get("name", f"item{i}") if isinstance(e, dict) else f"item{i}", e)
                    for i, e in enumerate(data)]
    else:
        raise ValueError(f"Unrecognised item file layout in {path}")
    for name, entry in iterable:
        xy = _xy_from(entry)
        if xy is None:
            continue
        stop = entry.get("stop") if isinstance(entry, dict) else None
        aisle = entry.get("aisle") if isinstance(entry, dict) else None
        items.append({"name": str(name), "x": xy[0], "y": xy[1], "yaw": xy[2],
                      "stop": stop, "aisle": aisle})
    if not items:
        raise ValueError(f"No items with coordinates found in {path}")
    return items


def group_stops(items, merge_radius=0.30, same_place=0.10):
    """Collapse items to unique driving stops.

    Uses the 'stop' field when present; otherwise merges items closer than
    merge_radius. Stops that sit on the same spot (within `same_place` m, e.g.
    the two faces of one aisle) are then merged, because the robot drives to
    the same point for both; their ids are joined with '+'.
    Returns a list of dicts: id, x, y, yaw, items (names).
    """
    stops = {}
    if all(it["stop"] is not None for it in items):
        for it in items:
            s = stops.setdefault(str(it["stop"]), {"id": str(it["stop"]), "xs": [], "ys": [],
                                                   "yaw": it["yaw"], "items": []})
            s["xs"].append(it["x"]); s["ys"].append(it["y"]); s["items"].append(it["name"])
    else:
        for it in items:
            for s in stops.values():
                if math.hypot(np.mean(s["xs"]) - it["x"], np.mean(s["ys"]) - it["y"]) < merge_radius:
                    s["xs"].append(it["x"]); s["ys"].append(it["y"]); s["items"].append(it["name"])
                    break
            else:
                sid = f"s{len(stops)}"
                stops[sid] = {"id": sid, "xs": [it["x"]], "ys": [it["y"]], "yaw": it["yaw"],
                              "items": [it["name"]]}
    out = []
    for s in stops.values():
        x, y = float(np.mean(s["xs"])), float(np.mean(s["ys"]))
        for o in out:
            if math.hypot(o["x"] - x, o["y"] - y) <= same_place:
                o["id"] += "+" + s["id"]
                o["items"] += s["items"]
                break
        else:
            out.append({"id": s["id"], "x": x, "y": y, "yaw": float(s["yaw"]), "items": list(s["items"])})
    return out


def stops_for_list(item_names, items, stops):
    """Unique stops (in first-seen order) needed for a shopping list."""
    by_name = {it["name"]: it for it in items}
    need = []
    for n in item_names:
        it = by_name[n]
        for s in stops:
            if n in s["items"]:
                if s not in need:
                    need.append(s)
                break
    return need


def make_lists(items, sizes, per_size, seed=7):
    """Reproducible random shopping lists: {size: [[names...], ...]}."""
    rng = random.Random(seed)
    names = [it["name"] for it in items]
    out = {}
    for n in sizes:
        n_eff = min(n, len(names))
        out[n] = [rng.sample(names, n_eff) for _ in range(per_size)]
    return out


def make_random_size_lists(items, n_lists, lo, hi, seed=7):
    """n_lists reproducible lists whose sizes are uniform in [lo, hi].

    Returns [(list_id, size, [names...]), ...]. List k depends only on the
    seed and the lists before it, so the first 10 of a 30-list run are the
    same 10 lists as a 10-list run with the same seed (used for B1 pairing).
    """
    rng = random.Random(seed)
    names = [it["name"] for it in items]
    out = []
    for k in range(n_lists):
        n = rng.randint(lo, hi)
        out.append((f"s{seed}_r{k:02d}_n{n}", n, rng.sample(names, min(n, len(names)))))
    return out


def make_pairs(stops, n_pairs, seed=5, min_dist=2.0, home=(0.0, 0.0)):
    """Fixed start-goal pairs between stops (and home), straight-line >= min_dist apart."""
    rng = random.Random(seed)
    nodes = [{"id": "home", "x": home[0], "y": home[1], "yaw": 0.0}] + list(stops)
    cand = [(a, b) for a in nodes for b in nodes
            if a is not b and math.hypot(a["x"] - b["x"], a["y"] - b["y"]) >= min_dist]
    if not cand:
        raise ValueError("no stop pairs that far apart")
    rng.shuffle(cand)
    # cycle through the candidates if more pairs are asked for than exist
    return [(f"pair{k:02d}", cand[k % len(cand)][0], cand[k % len(cand)][1]) for k in range(n_pairs)]


def parse_range(text):
    """'2' -> (2, 2); '1-3' -> (1, 3)."""
    text = str(text)
    if "-" in text:
        lo, hi = text.split("-", 1)
        return int(lo), int(hi)
    return int(text), int(text)


# --------------------------------------------------------------------------
# Occupancy grid
# --------------------------------------------------------------------------

class Grid:
    """Occupancy grid with world<->cell conversion and an inflated free mask."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 robot_radius=0.15, occ_threshold=65, unknown_is_free=False):
        self.w, self.h, self.res = int(width), int(height), float(resolution)
        self.ox, self.oy = float(origin_x), float(origin_y)
        raw = np.asarray(data, dtype=np.int16).reshape(self.h, self.w)
        self.raw = raw
        blocked = raw >= occ_threshold
        if not unknown_is_free:
            blocked |= raw < 0
        self.blocked_raw = blocked
        self.free = ~self._inflate(blocked, robot_radius)

    def _inflate(self, blocked, radius):
        r = int(math.ceil(radius / self.res))
        if r <= 0:
            return blocked.copy()
        out = blocked.copy()
        ys, xs = np.nonzero(blocked)
        offsets = [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1)
                   if dx * dx + dy * dy <= r * r]
        for dy, dx in offsets:
            yy, xx = ys + dy, xs + dx
            ok = (yy >= 0) & (yy < self.h) & (xx >= 0) & (xx < self.w)
            out[yy[ok], xx[ok]] = True
        return out

    def to_cell(self, x, y):
        return (int(math.floor((y - self.oy) / self.res)),
                int(math.floor((x - self.ox) / self.res)))

    def to_world(self, cell):
        r, c = cell
        return (self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res)

    def inside(self, cell):
        return 0 <= cell[0] < self.h and 0 <= cell[1] < self.w

    def nearest_free(self, cell, max_r=40):
        """Closest free cell to `cell` (goals often sit inside shelf inflation)."""
        if self.inside(cell) and self.free[cell]:
            return cell
        for rad in range(1, max_r + 1):
            best = None
            for dr in range(-rad, rad + 1):
                for dc in (-rad, rad) if abs(dr) != rad else range(-rad, rad + 1):
                    c = (cell[0] + dr, cell[1] + dc)
                    if self.inside(c) and self.free[c]:
                        d = dr * dr + dc * dc
                        if best is None or d < best[0]:
                            best = (d, c)
            if best:
                return best[1]
        return None

    def occupied_points(self):
        """World (x, y) of every cell marked occupied (not unknown)."""
        rows, cols = np.nonzero(self.raw >= 65)
        return np.column_stack([self.ox + (cols + 0.5) * self.res, self.oy + (rows + 0.5) * self.res])

    @classmethod
    def from_npz(cls, path, **kw):
        z = np.load(os.path.expanduser(path))
        return cls(z["data"], int(z["width"]), int(z["height"]), float(z["resolution"]),
                   float(z["origin_x"]), float(z["origin_y"]), **kw)

    def save_npz(self, path):
        np.savez_compressed(path, data=self.raw.ravel(), width=self.w, height=self.h,
                            resolution=self.res, origin_x=self.ox, origin_y=self.oy)


SQ2 = math.sqrt(2.0)
NEIGH = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2)]


def dijkstra_from(grid, src):
    """Distances (metres) from src cell to every free cell; inf where unreachable."""
    dist = np.full((grid.h, grid.w), np.inf)
    if src is None:
        return dist
    dist[src] = 0.0
    pq = [(0.0, src[0], src[1])]
    free = grid.free
    while pq:
        d, r, c = heapq.heappop(pq)
        if d > dist[r, c]:
            continue
        for dr, dc, w in NEIGH:
            rr, cc = r + dr, c + dc
            if 0 <= rr < grid.h and 0 <= cc < grid.w and free[rr, cc]:
                # no corner cutting through blocked diagonals
                if dr and dc and not (free[r + dr, c] and free[r, c + dc]):
                    continue
                nd = d + w * grid.res
                if nd < dist[rr, cc]:
                    dist[rr, cc] = nd
                    heapq.heappush(pq, (nd, rr, cc))
    return dist


def cost_matrix(grid, points):
    """Pairwise Dijkstra path cost (m) between points [(x, y), ...].

    Also returns the straight-line matrix and the snapped cells.
    """
    cells = [grid.nearest_free(grid.to_cell(x, y)) for x, y in points]
    n = len(points)
    D = np.full((n, n), np.inf)
    E = np.zeros((n, n))
    for i in range(n):
        dist = dijkstra_from(grid, cells[i])
        for j in range(n):
            E[i, j] = math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1])
            if cells[j] is not None:
                D[i, j] = dist[cells[j]]
    return D, E, cells


# --------------------------------------------------------------------------
# Visit-order strategies. Node 0 is the start; 1..n are stops. Open path.
# --------------------------------------------------------------------------

def route_cost(D, order, end=None):
    seq = [0] + list(order) + ([end] if end is not None else [])
    return float(sum(D[a, b] for a, b in zip(seq, seq[1:])))


def order_as_entered(n):
    return list(range(1, n + 1))


def order_nearest(M, n):
    """Greedy nearest-remaining using matrix M (Euclidean or path cost)."""
    left = set(range(1, n + 1))
    cur, out = 0, []
    while left:
        nxt = min(left, key=lambda j: (M[cur, j], j))
        out.append(nxt)
        left.remove(nxt)
        cur = nxt
    return out


def order_exact(D, n, end=None):
    """Held-Karp from node 0 through 1..n (then to node `end`, if given). Exact for n <= ~14."""
    if n == 0:
        return []
    INF = float("inf")
    size = n + 1 if end is None else max(n + 1, end + 1)
    Dl = [[float(D[i][j]) for j in range(size)] for i in range(size)]
    full = 1 << n
    dp = [[INF] * n for _ in range(full)]
    parent = [[-1] * n for _ in range(full)]
    for j in range(n):
        dp[1 << j][j] = Dl[0][j + 1]
    for mask in range(1, full):
        row = dp[mask]
        for j in range(n):
            cur = row[j]
            if cur == INF or not (mask >> j) & 1:
                continue
            dj = Dl[j + 1]
            for k in range(n):
                if (mask >> k) & 1:
                    continue
                nm = mask | (1 << k)
                val = cur + dj[k + 1]
                if val < dp[nm][k]:
                    dp[nm][k] = val
                    parent[nm][k] = j
    last_row = dp[full - 1]
    last = min(range(n), key=lambda j: last_row[j] + (Dl[j + 1][end] if end is not None else 0.0))
    order, mask = [], full - 1
    while last != -1:
        order.append(last + 1)
        prev = parent[mask][last]
        mask &= ~(1 << last)
        last = prev
    return order[::-1]


STRATEGIES = ("as_entered", "nn_euclid", "nn_dijkstra", "exact")


def order_for(strategy, D, E, n):
    if strategy == "as_entered":
        return order_as_entered(n)
    if strategy == "nn_euclid":
        return order_nearest(E, n)
    if strategy == "nn_dijkstra":
        return order_nearest(D, n)
    if strategy == "exact":
        return order_exact(D, n)
    raise ValueError(strategy)


def save_json(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, default=float)
