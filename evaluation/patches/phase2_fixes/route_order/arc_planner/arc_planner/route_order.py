"""Order a shopping list by Dijkstra path cost between the items (ARC N3).

ROS-free, so it can be unit-tested and reused. The cost model mirrors
dijkstra_planner.py exactly, so the order is chosen on the same costs the
planner will then drive:

- a cell is passable when no occupied cell (value >= 50) lies within the
  4-cell (0.20 m) inflation radius and it is inside the map;
- entering a cell costs move * terrain, with move 1 or sqrt(2) (8-connected)
  and terrain 1.0 for free, 5.0 for unknown;
- the robot start and each stop snap to the nearest passable cell, as the
  planner does (search radius 20 cells).

Items on the same spot (within 0.10 m, e.g. both faces of one aisle) are merged
into one stop, so the robot never gets a goal at the place it already is.

Strategies: 'exact' (Held-Karp optimum from the robot; used up to 12 stops, the
compact store has 9 spots), 'nearest' (greedy nearest by path cost) and
'as_entered' (the old behaviour).

Optional fixed end (end_xy, e.g. the checkout counter): the route is then
start -> all stops -> end, and 'exact' optimises it with the end in mind, which
can differ from the best open route followed by a drive to the end.
"""
import heapq
import math

OCCUPIED = 50
INFLATION_CELLS = 4
UNKNOWN_COST = 5.0
SEARCH_CELLS = 20
SAME_SPOT_M = 0.10
EXACT_MAX_STOPS = 12
SQ2 = math.sqrt(2.0)
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2)]


class CostGrid:
    """Planner-equivalent costs on an OccupancyGrid (data row-major, origin bottom-left)."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 inflation=INFLATION_CELLS, unknown_cost=UNKNOWN_COST):
        self.w, self.h, self.res = int(width), int(height), float(resolution)
        self.ox, self.oy = float(origin_x), float(origin_y)
        self.data = list(data)
        self.unknown_cost = unknown_cost
        occ = [i for i, v in enumerate(self.data) if v >= OCCUPIED]
        offs = [(dx, dy) for dx in range(-inflation, inflation + 1) for dy in range(-inflation, inflation + 1)
                if dx * dx + dy * dy <= inflation * inflation]
        blocked = bytearray(self.w * self.h)
        for i in occ:
            gx, gy = i % self.w, i // self.w
            for dx, dy in offs:
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    blocked[ny * self.w + nx] = 1
        # the planner also treats cells whose inflation disc leaves the map as unsafe
        for gy in range(self.h):
            for gx in range(self.w):
                if gx < inflation or gy < inflation or gx >= self.w - inflation or gy >= self.h - inflation:
                    blocked[gy * self.w + gx] = 1
        self.blocked = blocked

    @classmethod
    def from_msg(cls, msg, **kw):
        i = msg.info
        return cls(msg.data, i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y, **kw)

    def to_cell(self, x, y):
        return int((x - self.ox) / self.res), int((y - self.oy) / self.res)

    def free(self, gx, gy):
        return 0 <= gx < self.w and 0 <= gy < self.h and not self.blocked[gy * self.w + gx]

    def nearest_free(self, cell, radius=SEARCH_CELLS):
        gx, gy = cell
        if self.free(gx, gy):
            return cell
        best = None
        for r in range(1, radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    if self.free(gx + dx, gy + dy):
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, (gx + dx, gy + dy))
            if best:
                return best[1]
        return None

    def costs_from(self, src, targets):
        """Planner cost (metres x terrain) from src cell to each target cell; inf if unreachable."""
        want = {t for t in targets if t is not None}
        dist = {src: 0.0}
        pq = [(0.0, src)]
        found = {}
        while pq and len(found) < len(want):
            d, (cx, cy) = heapq.heappop(pq)
            if d > dist.get((cx, cy), math.inf):
                continue
            if (cx, cy) in want:
                found[(cx, cy)] = d
            for dx, dy, mc in MOVES:
                nx, ny = cx + dx, cy + dy
                if not self.free(nx, ny):
                    continue
                v = self.data[ny * self.w + nx]
                terrain = self.unknown_cost if v < 0 else 1.0
                nd = d + mc * terrain
                if nd < dist.get((nx, ny), math.inf):
                    dist[(nx, ny)] = nd
                    heapq.heappush(pq, (nd, (nx, ny)))
        return [found.get(t, math.inf) * self.res if t is not None else math.inf for t in targets]


def group_same_spot(goals, same=SAME_SPOT_M):
    """Merge goals within `same` metres into one stop; keeps first-seen order."""
    stops = []
    for g in goals:
        for s in stops:
            if math.hypot(s["x"] - g["x"], s["y"] - g["y"]) <= same:
                s["item"] += " + " + g["item"]
                s["items"].append(g["item"])
                if g.get("shelf_name") and g["shelf_name"] not in s["shelf_name"]:
                    s["shelf_name"] += " / " + g["shelf_name"]
                break
        else:
            s = dict(g)
            s["items"] = [g["item"]]
            stops.append(s)
    return stops


def cost_matrix(grid, start_xy, stops):
    """D[i][j] for nodes 0 = robot, 1..n = stops."""
    pts = [start_xy] + [(s["x"], s["y"]) for s in stops]
    cells = [grid.nearest_free(grid.to_cell(x, y)) for x, y in pts]
    D = []
    for c in cells:
        D.append(grid.costs_from(c, cells) if c is not None else [math.inf] * len(cells))
    for i in range(len(D)):
        D[i][i] = 0.0
    return D, cells


def route_cost(D, order, end=None):
    seq = [0] + list(order) + ([end] if end is not None else [])
    return sum(D[a][b] for a, b in zip(seq, seq[1:]))


def order_nearest(D, n):
    left, cur, out = set(range(1, n + 1)), 0, []
    while left:
        nxt = min(left, key=lambda j: (D[cur][j], j))
        out.append(nxt)
        left.remove(nxt)
        cur = nxt
    return out


def order_exact(D, n, end=None):
    """Held-Karp optimum of the path from node 0 through nodes 1..n (then to node `end` if given)."""
    if n == 0:
        return []
    INF = math.inf
    full = 1 << n
    dp = [[INF] * n for _ in range(full)]
    par = [[-1] * n for _ in range(full)]
    for j in range(n):
        dp[1 << j][j] = D[0][j + 1]
    for mask in range(1, full):
        row = dp[mask]
        for j in range(n):
            c = row[j]
            if c == INF or not (mask >> j) & 1:
                continue
            dj = D[j + 1]
            for k in range(n):
                if (mask >> k) & 1:
                    continue
                nm = mask | (1 << k)
                v = c + dj[k + 1]
                if v < dp[nm][k]:
                    dp[nm][k] = v
                    par[nm][k] = j
    tail = (lambda j: D[j + 1][end]) if end is not None else (lambda j: 0.0)
    last = min(range(n), key=lambda j: dp[full - 1][j] + tail(j))
    order, mask = [], full - 1
    while last != -1:
        order.append(last + 1)
        prev = par[mask][last]
        mask &= ~(1 << last)
        last = prev
    return order[::-1]


def plan_route(grid, start_xy, goals, strategy="exact", end_xy=None):
    """Return (ordered stops, info dict). Unreachable stops go last, in entered order.

    end_xy: optional fixed final destination (not returned as a stop; the caller appends it)."""
    stops = group_same_spot(goals)
    n = len(stops)
    pts_stops = stops + ([{"x": end_xy[0], "y": end_xy[1]}] if end_xy is not None else [])
    D, cells = cost_matrix(grid, start_xy, pts_stops)
    end = n + 1 if end_xy is not None and math.isfinite(D[0][n + 1]) else None
    reach = [j for j in range(1, n + 1) if math.isfinite(D[0][j])]
    unreach = [j for j in range(1, n + 1) if j not in reach]
    entered = list(range(1, n + 1))
    if strategy == "as_entered" or not reach:
        order, used = entered, "as_entered"
    else:
        # solve on the reachable sub-problem
        idx = [0] + reach + ([end] if end is not None else [])
        sub = [[D[a][b] for b in idx] for a in idx]
        m = len(reach)
        sub_end = m + 1 if end is not None else None
        if strategy == "exact" and m <= EXACT_MAX_STOPS:
            o, used = order_exact(sub, m, sub_end), "exact"
        else:
            o, used = order_nearest(sub, m), "nearest"
        order = [reach[i - 1] for i in o] + unreach
    finite = [j for j in order if j not in unreach]
    info = {"strategy": used, "stops": n, "merged_items": len(goals) - n,
            "cost": route_cost(D, finite, end) if finite else 0.0,
            "cost_as_entered": route_cost(D, [j for j in entered if j not in unreach], end) if reach else 0.0,
            "ends_at": "end" if end is not None else ("end unreachable" if end_xy is not None else "last stop"),
            "unreachable": [stops[j - 1]["item"] for j in unreach],
            "start_cell": cells[0]}
    return [stops[j - 1] for j in order], info
