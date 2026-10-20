#!/usr/bin/env python3
"""Would a real shopping cart fit this store? Offline, from the saved map (no ROS, no driving).

    python3 arc_tests/footprint_check.py --map results/map.npz --out results

For each robot radius, every cell closer than that radius to a mapped obstacle is blocked, and the
Dijkstra path from home to each of the 9 driving stops is computed on what is left. Radii checked:
  0.15 m  the test kit's own grid (C1/C2)
  0.20 m  ARC's planner inflation, used with the TurtleBot stand-in in all tests
  0.35 m  cart, straight driving: half its 0.61 m width + 0.05 m margin (lenient bound)
  0.59 m  cart, turning on the spot: half its 1.00 x 0.61 m diagonal (strict bound)
Cart size: a standard 180 L supermarket trolley, 1000 x 610 mm.
A stop counts as reachable only if the robot can stand within 1.0 m of it (ARC's planner snaps a goal
at most 20 cells = 1.0 m to free space) and a path exists; the snap distance is reported too.
A circle is only a bound for a rectangular cart: the truth lies between the two cart rows.
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arc_tests.common import Grid, cost_matrix, find_item_file, group_stops, load_items  # noqa: E402

CART_L, CART_W, MARGIN = 1.00, 0.61, 0.05
RADII = [("kit grid (C1/C2)", 0.15), ("ARC planner, TurtleBot tests", 0.20),
         ("cart, straight (lenient)", CART_W / 2 + MARGIN),
         ("cart, turning (strict)", math.hypot(CART_L, CART_W) / 2)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--items", default=None)
    ap.add_argument("--out", default="results")
    ap.add_argument("--start", default="0,0")
    ap.add_argument("--snap-max", type=float, default=1.0, help="ARC planner snap radius (m)")
    a = ap.parse_args()
    path = a.items or find_item_file()
    if not path:
        sys.exit("item_locations.yaml not found; pass --items")
    stops = group_stops(load_items(path))
    sx, sy = (float(v) for v in a.start.split(","))
    pts = [(sx, sy)] + [(s["x"], s["y"]) for s in stops]
    rows, base = [], None
    for name, r in RADII:
        g = Grid.from_npz(a.map, robot_radius=r)
        D, _, cells = cost_matrix(g, pts)
        snap = []
        for (x, y), c in zip(pts, cells):
            c0 = g.to_cell(x, y)
            snap.append(math.hypot(c[0] - c0[0], c[1] - c0[1]) * g.res if c is not None else float("inf"))
        start_ok = snap[0] <= a.snap_max
        reach = [j for j in range(1, len(pts))
                 if start_ok and snap[j] <= a.snap_max and math.isfinite(D[0, j])]
        lengths = {j: float(D[0, j]) for j in reach}
        if base is None:
            base = lengths
        common = [j for j in reach if j in base]
        longer = (100 * (sum(lengths[j] for j in common) - sum(base[j] for j in common))
                  / sum(base[j] for j in common)) if common else float("nan")
        lost = [stops[j - 1]["id"] for j in range(1, len(pts)) if j not in reach]
        rows.append({"footprint": name, "radius_m": round(r, 3), "start_free": int(start_ok),
                     "stops_reachable": f"{len(reach)}/{len(stops)}",
                     "max_snap_m": round(max(snap[j] for j in reach), 2) if reach else "",
                     "path_longer_than_kit_grid_pct": round(longer, 1) if math.isfinite(longer) else "",
                     "unreachable": " ".join(lost)})
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "footprint_check.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, rows[0].keys())
        w.writeheader()
        w.writerows(rows)
    print(f"Cart: standard 180 L trolley {CART_L * 1000:.0f} x {CART_W * 1000:.0f} mm. Map: {a.map}")
    print(f"{'footprint':30s} {'radius':>7s} {'start':>6s} {'stops':>6s} {'snap':>6s} {'longer':>7s}  unreachable")
    for r in rows:
        print(f"{r['footprint']:30s} {r['radius_m']:7.2f} {'ok' if r['start_free'] else 'BLOCK':>6s} "
              f"{r['stops_reachable']:>6s} {str(r['max_snap_m']):>6s} {str(r['path_longer_than_kit_grid_pct']):>6s}%  {r['unreachable'] or '-'}")
    print(f"wrote {os.path.join(a.out, 'footprint_check.csv')}")


if __name__ == "__main__":
    main()
