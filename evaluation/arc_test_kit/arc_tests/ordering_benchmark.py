#!/usr/bin/env python3
"""C1: offline shopping-list ordering benchmark. No ROS needed.

Uses the store map saved by the mission runner (results/map.npz) or by
`save_map.py`, plus item_locations.yaml. For many random lists it computes
the Dijkstra path-cost matrix between the start and the stops, orders the
stops four ways and reports the route cost of each against the exact optimum.

    python3 arc_tests/ordering_benchmark.py --map results/map.npz --out results
"""
import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arc_tests.common import (STRATEGIES, Grid, cost_matrix, find_item_file, group_stops,  # noqa: E402
                              load_items, make_lists, order_exact, order_for, route_cost, stops_for_list)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--items", default=None)
    ap.add_argument("--out", default="results")
    ap.add_argument("--sizes", default="3,5,8,10,12")
    ap.add_argument("--per-size", type=int, default=50)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--start", default="0,0", help="x,y where every route starts")
    ap.add_argument("--inflation", type=float, default=0.15)
    ap.add_argument("--end", default=None,
                    help="x,y of a fixed final destination (the checkout); writes ordering_benchmark_checkout.csv")
    a = ap.parse_args()
    if a.end:
        return checkout_benchmark(a)

    path = a.items or find_item_file()
    if not path:
        sys.exit("item_locations.yaml not found; pass --items")
    items = load_items(path)
    stops = group_stops(items)
    grid = Grid.from_npz(a.map, robot_radius=a.inflation)
    sx, sy = (float(v) for v in a.start.split(","))
    sizes = [int(s) for s in a.sizes.split(",")]
    lists = make_lists(items, sizes, a.per_size, seed=a.seed)
    os.makedirs(a.out, exist_ok=True)
    out_path = os.path.join(a.out, "ordering_benchmark.csv")
    cols = ["n_items", "list", "n_stops"] + [f"J_{s}" for s in STRATEGIES] + \
           [f"gap_{s}_pct" for s in STRATEGIES] + ["t_exact_ms", "t_nn_ms", "unreachable"]
    print(f"{len(items)} items, {len(stops)} stops; map {grid.w}x{grid.h} @ {grid.res} m")
    with open(out_path, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=cols)
        wr.writeheader()
        for n in sizes:
            for li, names in enumerate(lists[n]):
                need = stops_for_list(names, items, stops)
                pts = [(sx, sy)] + [(s["x"], s["y"]) for s in need]
                D, E, cells = cost_matrix(grid, pts)
                m = len(need)
                row = {"n_items": n, "list": li, "n_stops": m,
                       "unreachable": int(any(c is None for c in cells) or not (D < float("inf")).all())}
                if row["unreachable"]:
                    wr.writerow(row)
                    continue
                t = time.perf_counter()
                exact = order_for("exact", D, E, m)
                row["t_exact_ms"] = round((time.perf_counter() - t) * 1000, 2)
                jopt = route_cost(D, exact)
                for s in STRATEGIES:
                    t = time.perf_counter()
                    o = exact if s == "exact" else order_for(s, D, E, m)
                    if s == "nn_dijkstra":
                        row["t_nn_ms"] = round((time.perf_counter() - t) * 1000, 3)
                    j = route_cost(D, o)
                    row[f"J_{s}"] = round(j, 4)
                    row[f"gap_{s}_pct"] = round(100 * (j - jopt) / jopt, 3) if jopt > 0 else 0.0
                wr.writerow(row)
            print(f"  n={n}: {a.per_size} lists done")
    print(f"wrote {out_path}")



def checkout_benchmark(a):
    """Every route ends at a fixed destination (the checkout). Compares, per list:
    typed order + checkout, nearest-next + checkout, the best OPEN order + checkout, and the
    exact order planned WITH the checkout as the fixed end (the optimum)."""
    path = a.items or find_item_file()
    if not path:
        sys.exit("item_locations.yaml not found; pass --items")
    items = load_items(path)
    stops = group_stops(items)
    grid = Grid.from_npz(a.map, robot_radius=a.inflation)
    sx, sy = (float(v) for v in a.start.split(","))
    ex, ey = (float(v) for v in a.end.split(","))
    sizes = [int(s) for s in a.sizes.split(",")]
    lists = make_lists(items, sizes, a.per_size, seed=a.seed)
    os.makedirs(a.out, exist_ok=True)
    out_path = os.path.join(a.out, "ordering_benchmark_checkout.csv")
    names = ["as_entered", "nn_dijkstra", "exact_open_then_end", "exact_with_end"]
    cols = ["n_items", "list", "n_stops"] + [f"J_{s}" for s in names] + [f"gap_{s}_pct" for s in names] + ["unreachable"]
    summary = {n: {s: [] for s in names} for n in sizes}
    with open(out_path, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=cols)
        wr.writeheader()
        for n in sizes:
            for li, nm in enumerate(lists[n]):
                need = stops_for_list(nm, items, stops)
                pts = [(sx, sy)] + [(s_["x"], s_["y"]) for s_ in need] + [(ex, ey)]
                D, E, cells = cost_matrix(grid, pts)
                m = len(need)
                end = m + 1
                row = {"n_items": n, "list": li, "n_stops": m,
                       "unreachable": int(any(c is None for c in cells) or not (D < float("inf")).all())}
                if row["unreachable"]:
                    wr.writerow(row)
                    continue
                orders = {"as_entered": order_for("as_entered", D, E, m),
                          "nn_dijkstra": order_for("nn_dijkstra", D, E, m),
                          "exact_open_then_end": order_exact(D, m),
                          "exact_with_end": order_exact(D, m, end)}
                jopt = route_cost(D, orders["exact_with_end"], end)
                for s_, o in orders.items():
                    j = route_cost(D, o, end)
                    row[f"J_{s_}"] = round(j, 4)
                    g = 100 * (j - jopt) / jopt if jopt > 0 else 0.0
                    row[f"gap_{s_}_pct"] = round(g, 3)
                    summary[n][s_].append(g)
                wr.writerow(row)
    print(f"Routes start at ({sx}, {sy}) and end at the checkout ({ex}, {ey}). Mean excess over the optimum, %:")
    print(f"{'items':>5} " + " ".join(f"{s_:>22}" for s_ in names))
    for n in sizes:
        print(f"{n:5d} " + " ".join(f"{(sum(v) / len(v) if v else float('nan')):22.1f}" for v in summary[n].values()))
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
