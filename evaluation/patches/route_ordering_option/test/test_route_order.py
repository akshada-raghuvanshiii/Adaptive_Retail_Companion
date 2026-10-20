"""Offline tests for route_order.py (no ROS):  python3 -m unittest discover -s test -v"""
import itertools, math, os, random, sys, time, unittest
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "arc_planner"))
from arc_planner import route_order as ro  # noqa: E402

RES, OX, OY, W, H = 0.05, -3.5, -3.0, 160, 140


def store_grid():
    """The compact store's walls and shelves (positions from arc_store.sdf), as an occupancy grid."""
    d = [0] * (W * H)
    def box(cx, cy, sx, sy):
        for gy in range(int((cy - sy / 2 - OY) / RES), int((cy + sy / 2 - OY) / RES)):
            for gx in range(int((cx - sx / 2 - OX) / RES), int((cx + sx / 2 - OX) / RES)):
                if 0 <= gx < W and 0 <= gy < H:
                    d[gy * W + gx] = 100
    for b in [(0.5, -2.95, 8, 0.1), (0.5, 3.95, 8, 0.1), (-3.45, 0.5, 0.1, 7), (4.45, 0.5, 0.1, 7),
              (0.95, 0.5, 0.6, 2.8), (2.65, 0.5, 0.6, 2.8), (4.225, 0.5, 0.4, 2.8), (-1.4, 1.1, 0.8, 0.8),
              (-2.2, 3.6, 1.2, 0.5), (-3.2, 0.4, 0.4, 1.2), (-1.8, -2.05, 1.2, 0.5)]:
        box(*b)
    return ro.CostGrid(d, W, H, RES, OX, OY)


SPOTS = [(0.15, -0.4), (1.8, -0.4), (3.48, -0.4), (0.15, 1.4), (1.8, 1.4), (3.48, 1.4),
         (-2.2, 2.75), (-1.4, 0.05), (-2.45, 0.4)]


def goals(names_xy):
    return [{"item": n, "shelf_id": "s", "shelf_name": "s", "x": x, "y": y, "yaw": 0.0} for n, (x, y) in names_xy]


class RouteOrderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.g = store_grid()

    def test_compare_routes_prices_both_options(self):
        names = ["pasta", "tea_coffee", "fruits", "milk"]
        gl = goals(list(zip(names, [SPOTS[5], SPOTS[8], SPOTS[7], SPOTS[6]])))
        c = ro.compare_routes(self.g, (-0.5, -2.0), gl)
        (opt, oi), (typ, ti) = c["optimised"], c["typed"]
        self.assertEqual([s["item"] for s in typ], names)               # typed keeps the typed order
        self.assertEqual(sorted(s["item"] for s in opt), sorted(names))  # same items, new order
        self.assertLessEqual(oi["cost"], ti["cost"] + 1e-9)
        self.assertAlmostEqual(c["saved_m"], ti["cost"] - oi["cost"], places=9)
        self.assertGreater(c["saved_pct"], 0.0)
        # the typed price is the same as plan_route reports for 'as_entered'
        self.assertAlmostEqual(ti["cost"], ro.plan_route(self.g, (-0.5, -2.0), gl, "as_entered")[1]["cost"])

    def test_exact_is_optimal(self):
        rng = random.Random(1)
        for _ in range(8):
            pick = rng.sample(SPOTS, rng.randint(2, 6))
            st = goals([(f"i{k}", p) for k, p in enumerate(pick)])
            stops = ro.group_same_spot(st)
            D, _ = ro.cost_matrix(self.g, (0.0, 0.0), stops)
            n = len(stops)
            best = min(ro.route_cost(D, p) for p in itertools.permutations(range(1, n + 1)))
            self.assertAlmostEqual(ro.route_cost(D, ro.order_exact(D, n)), best, places=9)

    def test_same_spot_items_merge(self):
        st = goals([("atta_flour", (0.15, -0.4)), ("dairy_milk", (-2.2, 2.75)), ("bread", (0.15, -0.4))])
        ordered, info = ro.plan_route(self.g, (0.0, 0.0), st)
        self.assertEqual(info["stops"], 2)
        self.assertEqual(info["merged_items"], 1)
        self.assertIn("atta_flour + bread", [s["item"] for s in ordered])

    def test_never_worse_than_entered_and_fast(self):
        rng = random.Random(7)
        for _ in range(10):
            pick = [rng.choice(SPOTS) for _ in range(rng.randint(3, 12))]
            st = goals([(f"i{k}", p) for k, p in enumerate(pick)])
            t = time.time()
            ordered, info = ro.plan_route(self.g, (0.0, 0.0), st, "exact")
            self.assertLess(time.time() - t, 15.0)
            self.assertLessEqual(info["cost"], info["cost_as_entered"] + 1e-9)
            self.assertEqual(sorted(sum((s["items"] for s in ordered), [])), sorted(g["item"] for g in st))

    def test_costs_are_path_lengths_not_straight_lines(self):
        # 0.15,-0.4 -> 1.8,-0.4 crosses gondola_1, so the path must go round it
        D, _ = ro.cost_matrix(self.g, (0.15, -0.4), ro.group_same_spot(goals([("b", (1.8, -0.4))])))
        self.assertGreater(D[0][1], 1.65 + 0.5)

    def test_unreachable_stop_goes_last(self):
        st = goals([("inside_shelf_far", (0.95, 0.5)), ("ok", (-1.4, 0.05))])
        g = ro.CostGrid([100 if (abs(i % W - int((0.95 - OX) / RES)) < 30 and abs(i // W - int((0.5 - OY) / RES)) < 30) else 0
                         for i in range(W * H)], W, H, RES, OX, OY)
        ordered, info = ro.plan_route(g, (-1.0, -1.0), st)
        self.assertEqual(ordered[-1]["item"], "inside_shelf_far")
        self.assertEqual(info["unreachable"], ["inside_shelf_far"])

    def test_as_entered_keeps_order(self):
        st = goals([("a", SPOTS[5]), ("b", SPOTS[0]), ("c", SPOTS[7])])
        ordered, info = ro.plan_route(self.g, (0.0, 0.0), st, "as_entered")
        self.assertEqual([s["item"] for s in ordered], ["a", "b", "c"])

    CHECKOUT = (-1.8, -1.45)   # in front of the checkout counter (box at -1.8, -2.05 in arc_store.sdf)

    def test_exact_with_fixed_end_is_optimal(self):
        rng = random.Random(4)
        for _ in range(8):
            pick = rng.sample(SPOTS, rng.randint(2, 6))
            stops = ro.group_same_spot(goals([(f"i{k}", p) for k, p in enumerate(pick)]))
            n = len(stops)
            D, _ = ro.cost_matrix(self.g, (0.0, 0.0), stops + [{"x": self.CHECKOUT[0], "y": self.CHECKOUT[1]}])
            end = n + 1
            best = min(ro.route_cost(D, p, end) for p in itertools.permutations(range(1, n + 1)))
            self.assertAlmostEqual(ro.route_cost(D, ro.order_exact(D, n, end), end), best, places=9)

    def test_plan_route_with_checkout_never_worse_than_open_order_then_checkout(self):
        rng = random.Random(9)
        for _ in range(10):
            pick = [rng.choice(SPOTS) for _ in range(rng.randint(3, 10))]
            st = goals([(f"i{k}", p) for k, p in enumerate(pick)])
            with_end, info = ro.plan_route(self.g, (0.0, 0.0), st, "exact", end_xy=self.CHECKOUT)
            open_order, _ = ro.plan_route(self.g, (0.0, 0.0), st, "exact")
            self.assertEqual(info["ends_at"], "end")
            # cost of the open-optimal order followed by the drive to checkout
            stops = ro.group_same_spot(st)
            D, _ = ro.cost_matrix(self.g, (0.0, 0.0), stops + [{"x": self.CHECKOUT[0], "y": self.CHECKOUT[1]}])
            idx = {s["item"]: i + 1 for i, s in enumerate(stops)}
            open_then_end = ro.route_cost(D, [idx[s["item"]] for s in open_order], len(stops) + 1)
            self.assertLessEqual(info["cost"], open_then_end + 1e-9)
            self.assertEqual(sorted(s["item"] for s in with_end), sorted(s["item"] for s in stops))

    def test_without_end_behaviour_unchanged(self):
        st = goals([("a", SPOTS[5]), ("b", SPOTS[0]), ("c", SPOTS[7])])
        _, info = ro.plan_route(self.g, (0.0, 0.0), st, "exact")
        self.assertEqual(info["ends_at"], "last stop")


if __name__ == "__main__":
    unittest.main()
