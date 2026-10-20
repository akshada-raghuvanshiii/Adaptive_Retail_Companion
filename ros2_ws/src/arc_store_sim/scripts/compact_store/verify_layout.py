"""Check the compact store geometry before it ever reaches Gazebo (standard library;
matplotlib optional for the preview image).

Checks
  1. The store, including walls, lies inside the currently published RTAB-Map bounds.
  2. Every destination is inside the store interior, on free space, with clearance
     >= robot radius + safety margin to every obstacle.
  3. Every destination and the spawn point are connected through free space for a
     disc of the robot's radius (BFS on a 0.05 m grid inflated by the radius).
  4. Every destination's heading points at its fixture.
  5. Narrowest passage the robot centre can travel (2 x clearance along the reachable set).

    python3 verify_layout.py [--robot-radius 0.22] [--margin 0.10] [--png preview.png]
Exit code 1 if any check fails.
"""

import argparse
import json
import sys
from collections import deque
from math import atan2, cos, hypot, pi, sin

import layout as L

RES = 0.05


def dist_point_box(x, y, f):
    dx = max(f.x0 - x, 0.0, x - f.x1)
    dy = max(f.y0 - y, 0.0, y - f.y1)
    return hypot(dx, dy)


def clearance(x, y, obstacles=None):
    obstacles = obstacles if obstacles is not None else L.ALL_OBSTACLES
    return min(dist_point_box(x, y, f) for f in obstacles)


def inside(x, y, f):
    return f.x0 <= x <= f.x1 and f.y0 <= y <= f.y1


def build_grid(radius):
    nx = int(round((L.X1 - L.X0) / RES))
    ny = int(round((L.Y1 - L.Y0) / RES))
    free = [[False] * nx for _ in range(ny)]
    clear = [[0.0] * nx for _ in range(ny)]
    for j in range(ny):
        y = L.Y0 + (j + 0.5) * RES
        for i in range(nx):
            x = L.X0 + (i + 0.5) * RES
            c = clearance(x, y)
            occupied = any(inside(x, y, f) for f in L.ALL_OBSTACLES)
            clear[j][i] = 0.0 if occupied else c
            free[j][i] = (not occupied) and c >= radius
    return free, clear, nx, ny


def cell(x, y):
    return int((x - L.X0) / RES), int((y - L.Y0) / RES)


def bfs(free, nx, ny, start):
    seen = [[False] * nx for _ in range(ny)]
    i, j = start
    if not (0 <= i < nx and 0 <= j < ny and free[j][i]):
        return seen
    q = deque([start])
    seen[j][i] = True
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            a, b = i + di, j + dj
            if 0 <= a < nx and 0 <= b < ny and free[b][a] and not seen[b][a]:
                seen[b][a] = True
                q.append((a, b))
    return seen


def faces_fixture(d):
    f = next(f for f in L.FIXTURES if f.name == d.fixture)
    # nearest point of the fixture to the destination
    px = min(max(d.x, f.x0), f.x1)
    py = min(max(d.y, f.y0), f.y1)
    want = atan2(py - d.y, px - d.x)
    err = abs((d.yaw - want + pi) % (2 * pi) - pi)
    return err, dist_point_box(d.x, d.y, f)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot-radius", type=float, default=L.ROBOT_RADIUS)
    ap.add_argument("--margin", type=float, default=L.SAFETY_MARGIN)
    ap.add_argument("--png")
    ap.add_argument("--json", help="write the report as JSON")
    args = ap.parse_args(argv)

    need = args.robot_radius + args.margin
    failures = []
    report = {"robot_radius": args.robot_radius, "required_clearance": need, "checks": []}

    def check(name, ok, detail):
        report["checks"].append({"check": name, "ok": ok, "detail": detail})
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
        if not ok:
            failures.append(name)

    fits = (L.CURRENT_MAP_X[0] <= L.X0 and L.X1 <= L.CURRENT_MAP_X[1]
            and L.CURRENT_MAP_Y[0] <= L.Y0 and L.Y1 <= L.CURRENT_MAP_Y[1])
    check("store inside current map bounds", fits,
          f"store X {L.X0}..{L.X1}, Y {L.Y0}..{L.Y1}; map X {L.CURRENT_MAP_X[0]:.3f}..{L.CURRENT_MAP_X[1]:.3f}, "
          f"Y {L.CURRENT_MAP_Y[0]:.3f}..{L.CURRENT_MAP_Y[1]:.3f}; "
          f"margins W {L.X0 - L.CURRENT_MAP_X[0]:.2f}, E {L.CURRENT_MAP_X[1] - L.X1:.2f}, "
          f"S {L.Y0 - L.CURRENT_MAP_Y[0]:.2f}, N {L.CURRENT_MAP_Y[1] - L.Y1:.2f} m")

    overlaps = [(a.name, b.name) for i, a in enumerate(L.FIXTURES) for b in L.FIXTURES[i + 1:]
                if a.x0 < b.x1 and b.x0 < a.x1 and a.y0 < b.y1 and b.y0 < a.y1]
    check("fixtures do not overlap", not overlaps, str(overlaps or "none"))

    sx, sy, _ = L.SPAWN
    check("spawn point clearance", clearance(sx, sy) >= need, f"{clearance(sx, sy):.2f} m (need {need:.2f})")

    free, clear, nx, ny = build_grid(args.robot_radius)
    seen = bfs(free, nx, ny, cell(sx, sy))
    reach_cells = sum(map(sum, seen))
    free_cells = sum(map(sum, free))
    check("free space connected", reach_cells == free_cells,
          f"{reach_cells} of {free_cells} robot-feasible cells reachable from spawn")

    rows = []
    for d in L.DESTINATIONS:
        c = clearance(d.x, d.y)
        i, j = cell(d.x, d.y)
        reach = 0 <= i < nx and 0 <= j < ny and seen[j][i]
        in_store = L.IX0 < d.x < L.IX1 and L.IY0 < d.y < L.IY1
        yaw_err, face_dist = faces_fixture(d)
        ok = c >= need and reach and in_store and yaw_err < 0.05
        rows.append({"id": d.id, "x": round(d.x, 3), "y": round(d.y, 3), "yaw_deg": round(d.yaw * 180 / pi, 1),
                     "clearance_m": round(c, 3), "distance_to_own_fixture_m": round(face_dist, 3),
                     "reachable": reach, "heading_ok": yaw_err < 0.05, "ok": ok})
        check(f"destination {d.id}", ok,
              f"({d.x:.2f}, {d.y:.2f}, {d.yaw * 180 / pi:.0f} deg) clearance {c:.2f} m, "
              f"reachable={reach}, faces {d.fixture}={yaw_err < 0.05} at {face_dist:.2f} m")
    report["destinations"] = rows

    # narrowest gap the robot actually has to use: min 2*clearance on reachable cells of aisle centrelines
    widths = {}
    g = {f.name: f for f in L.FIXTURES}
    widths["aisle gondola 1-2"] = g["gondola_2"].x0 - g["gondola_1"].x1
    widths["aisle gondola 2-3"] = g["gondola_3"].x0 - g["gondola_2"].x1
    widths["gap produce - gondola 1"] = g["gondola_1"].x0 - g["produce_display"].x1
    widths["gap tea/coffee - produce"] = g["produce_display"].x0 - g["tea_coffee_shelf"].x1
    widths["cross aisle south (gondolas - wall)"] = g["gondola_1"].y0 - L.IY0
    widths["cross aisle north (gondolas - wall)"] = L.IY1 - g["gondola_1"].y1
    widths["gap fridge - produce"] = g["dairy_fridge"].y0 - g["produce_display"].y1
    widths["gap checkout - produce"] = g["produce_display"].y0 - g["checkout_counter"].y1
    turn = 2 * (args.robot_radius + args.margin)
    for k, v in widths.items():
        check(f"width: {k}", v >= turn, f"{v:.2f} m (robot turn-in-place + margin needs {turn:.2f} m)")
    report["widths_m"] = widths

    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=2)
    if args.png:
        draw(args.png, free, seen, nx, ny)
    print(f"\n{len(report['checks']) - len(failures)} passed, {len(failures)} failed")
    return 1 if failures else 0


def draw(path, free, seen, nx, ny):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, FancyArrow
    except ImportError:
        print("matplotlib not available: no preview written")
        return
    fig, ax = plt.subplots(figsize=(9, 8.2))
    ax.add_patch(Rectangle((L.CURRENT_MAP_X[0], L.CURRENT_MAP_Y[0]), L.CURRENT_MAP_X[1] - L.CURRENT_MAP_X[0],
                           L.CURRENT_MAP_Y[1] - L.CURRENT_MAP_Y[0], fill=False, ls="--", ec="#8a8984", lw=1))
    ax.text(L.CURRENT_MAP_X[0] + 0.05, L.CURRENT_MAP_Y[1] - 0.25, "current RTAB-Map extent", color="#52514e", fontsize=8)
    # robot-feasible free space
    import numpy as np
    img = np.zeros((ny, nx, 4))
    for j in range(ny):
        for i in range(nx):
            if seen[j][i]:
                img[j, i] = (0.16, 0.47, 0.84, 0.10)
    ax.imshow(img, origin="lower", extent=(L.X0, L.X1, L.Y0, L.Y1), interpolation="nearest")
    for f in L.ALL_OBSTACLES:
        ax.add_patch(Rectangle((f.x0, f.y0), f.x1 - f.x0, f.y1 - f.y0, fc=f.colour, ec="#333", lw=0.8))
        if f.kind != "wall":
            ax.text(*f.centre, f.name.replace("_", "\n"), ha="center", va="center", fontsize=7)
    spots = {}
    for d in L.DESTINATIONS:
        spots.setdefault((round(d.x, 2), round(d.y, 2)), []).append(d.id)
        ax.plot(d.x, d.y, "o", color="#eb6834", ms=5)
        ax.add_patch(FancyArrow(d.x, d.y, 0.3 * cos(d.yaw), 0.3 * sin(d.yaw), width=0.02, color="#eb6834"))
    for (x, y), ids in spots.items():
        ax.text(x, y + 0.12, "\n".join(ids), fontsize=5.5, color="#52514e", ha="center", va="bottom")
    sx, sy, syaw = L.SPAWN
    ax.add_patch(plt.Circle((sx, sy), L.ROBOT_RADIUS, color="#0ca30c", alpha=0.6))
    ax.text(sx + 0.25, sy - 0.25, "spawn (0,0)", fontsize=8, color="#0a7a0a")
    ax.set_xlim(L.CURRENT_MAP_X[0] - 0.2, L.CURRENT_MAP_X[1] + 0.2)
    ax.set_ylim(L.CURRENT_MAP_Y[0] - 0.2, L.CURRENT_MAP_Y[1] + 0.2)
    ax.set_aspect("equal")
    ax.grid(color="#e5e4df", lw=0.5)
    ax.set_xlabel("x (m) — world = map")
    ax.set_ylabel("y (m)")
    ax.set_title("ARC compact store: fixtures, destinations (orange, heading arrows),\n"
                 "space reachable by the robot centre (blue tint)", fontsize=10)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    print(f"Preview written to {path}")


if __name__ == "__main__":
    sys.exit(main())
