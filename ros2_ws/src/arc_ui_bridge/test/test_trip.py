"""Offline tests of the UI bridge logic (no ROS).   python3 -m pytest -q test"""
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from arc_ui_bridge import route_order as ro  # noqa: E402
from arc_ui_bridge.trip import Trip, resolve  # noqa: E402

PRODUCTS = json.load(open(os.path.join(ROOT, "web", "products.json")))
PMAP = yaml.safe_load(open(os.path.join(ROOT, "config", "product_map.yaml")))
ITEMS_FILE = os.environ.get("ARC_ITEM_LOCATIONS",
                            os.path.expanduser("~/slam_ws/src/arc_store_sim/config/item_locations.yaml"))
ITEMS = yaml.safe_load(open(ITEMS_FILE))["items"]


def test_every_ui_product_maps_to_a_store_item():
    ids = {p["id"] for p in PRODUCTS}
    assert ids == set(PMAP), (ids ^ set(PMAP))
    assert set(PMAP.values()) <= set(ITEMS), set(PMAP.values()) - set(ITEMS)


def test_resolve_merges_products_on_one_item():
    req = [{"item_id": "DY001", "quantity": 2, "name": "Milk A"}, {"item_id": "DY002", "quantity": 1, "name": "Milk B"},
           {"item_id": "FP001", "quantity": 1, "name": "Apple"}, {"item_id": "XX999", "quantity": 1}]
    goals, problems = resolve(req, PMAP, ITEMS)
    items = {g["item"]: g for g in goals}
    assert set(items) == {"milk", "fruits"}
    assert [p["qty"] for p in items["milk"]["products"]] == [2, 1]
    assert problems and "XX999" in problems[0]


def open_grid(w=120, h=120, res=0.05, ox=-3.0, oy=-3.0):
    return ro.CostGrid([0] * (w * h), w, h, res, ox, oy)


def test_route_is_ordered_and_same_spots_merge():
    req = [{"item_id": i, "quantity": 1} for i in ("GR001", "GR005", "BV001", "FP001", "HC001", "PC002")]
    goals, _ = resolve(req, PMAP, ITEMS)
    ordered, info = ro.plan_route(open_grid(), (0.0, 0.0), goals, "exact")
    # atta_flour and rice are on the same spot (gondola_2) -> one stop
    assert info["merged_items"] >= 1
    assert info["cost"] <= info["cost_as_entered"] + 1e-9
    labels = [s["item"] for s in ordered]
    assert len(labels) == len(set(labels))


def run_trip():
    req = [{"item_id": "FP001", "quantity": 1, "name": "Apple"}, {"item_id": "BV001", "quantity": 1, "name": "Tea"}]
    goals, _ = resolve(req, PMAP, ITEMS)
    ordered, info = ro.plan_route(open_grid(), (0.0, 0.0), goals, "exact")
    tr = Trip(arrive_m=1.0, min_leg_s=0.5)
    first = tr.start(ordered, goals, (0.0, 0.0, 0.0), info, t=0.0)
    return tr, first


def test_stale_and_far_arrivals_are_ignored():
    tr, first = run_trip()
    assert tr.state == "NAVIGATING"
    assert not tr.arrival((first["x"], first["y"]), t=0.1)          # latched message right after the goal
    assert not tr.arrival((first["x"] + 3, first["y"]), t=5.0)      # robot still far away
    assert tr.ignored_arrivals == 2
    assert tr.arrival((first["x"] + 0.3, first["y"]), t=6.0)
    assert tr.state == "AT_STOP" and tr.stops[0]["status"] == "reached"


def test_full_trip_returns_home():
    tr, first = run_trip()
    tr.arrival((first["x"], first["y"]), 2.0)
    second = tr.next(3.0)
    assert second is tr.stops[1]
    tr.arrival((second["x"], second["y"]), 6.0)
    home = tr.next(7.0)
    assert tr.state == "RETURNING" and home["label"] == "HOME"
    assert tr.arrival((0.2, 0.0), 12.0)
    assert tr.state == "DONE"


def test_pause_stop_resume_and_home():
    tr, first = run_trip()
    tr.pause(1.0)
    assert tr.state == "PAUSED"
    assert not tr.arrival((first["x"], first["y"]), 2.0)            # no arrivals while paused
    again = tr.resume(3.0)
    assert again is first and tr.state == "NAVIGATING"
    tr.pause(4.0, "STOPPED")
    assert tr.state == "STOPPED"
    tr.resume(5.0)
    home = tr.go_home(6.0)
    assert home["label"] == "HOME" and all(s["status"] == "skipped" for s in tr.stops)


def test_skip_moves_on():
    tr, first = run_trip()
    nxt = tr.skip(1.0)
    assert tr.stops[0]["status"] == "skipped" and nxt is tr.stops[1]


def test_cannot_start_twice():
    tr, _ = run_trip()
    try:
        tr.start([], [], (0, 0, 0), {}, 1.0)
    except ValueError:
        return
    raise AssertionError("second start should be refused")
