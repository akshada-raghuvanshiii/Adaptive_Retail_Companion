#!/usr/bin/env python3
"""Place every item in item_locations.yaml on a sensible aisle / gondola face, store-style.

Aisles (robot stops face the shelf; "a" = south half, "b" = north half of a 3 m gondola):
  Fresh       produce island, dairy fridge, tea & drinks wall shelf
  Aisle 1     x = 0.15, between the produce island and gondola 1  -> gondola 1 WEST face
  Aisle 2     x = 1.80, between gondola 1 and gondola 2           -> gondola 1 EAST face, gondola 2 WEST face
  Aisle 3     x = 3.48, between gondola 2 and the wall gondola 3  -> gondola 2 EAST face, gondola 3

Keeps every item name and every existing field; changes x, y, yaw, shelf, face, and adds
aisle + section. Item names not in PLAN are reported and left unchanged.

    python3 assign_items.py                 # dry run: print the plan, change nothing
    python3 assign_items.py --write         # back up item_locations.yaml, then rewrite it
"""

import argparse
import shutil
import sys
import time
from math import pi
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import layout as L  # noqa: E402

# destination id -> (aisle, section name, items)
PLAN = {
    "produce":          ("Fresh", "Fruit & vegetables", ["fruits", "vegetables"]),
    "dairy":            ("Fresh", "Dairy & frozen", ["milk", "curd_paneer", "eggs", "frozen"]),
    "tea_coffee":       ("Fresh", "Tea, coffee & drinks", ["tea_coffee", "juice", "water_beverages"]),
    "gondola_1_west_a": ("Aisle 1", "Bakery", ["bread", "bakery"]),
    "gondola_1_west_b": ("Aisle 1", "Breakfast", ["breakfast_cereal", "jam_spreads"]),
    "gondola_1_east_a": ("Aisle 2", "Biscuits", ["biscuits"]),
    "gondola_1_east_b": ("Aisle 2", "Snacks", ["snacks"]),
    "gondola_2_west_a": ("Aisle 2", "Rice & flour", ["rice", "atta_flour"]),
    "gondola_2_west_b": ("Aisle 2", "Dal, sugar & salt", ["dal_pulses", "sugar_salt"]),
    "gondola_2_east_a": ("Aisle 3", "Oil & spices", ["cooking_oil", "spices"]),
    "gondola_2_east_b": ("Aisle 3", "Pasta & sauces", ["pasta", "sauces"]),
    "gondola_3_a":      ("Aisle 3", "Household & pets", ["cleaning", "detergent", "pet_food"]),
    "gondola_3_b":      ("Aisle 3", "Personal & baby care", ["soap_shampoo", "toiletries", "baby_care"]),
}

DEST = {d.id: d for d in L.DESTINATIONS}


def face_of(yaw):
    a = (yaw + pi) % (2 * pi) - pi
    if abs(a) < pi / 4:
        return "west"
    if abs(abs(a) - pi) < pi / 4:
        return "east"
    return "south" if a > 0 else "north"


def item_to_stop():
    out = {}
    for dest_id, (aisle, section, items) in PLAN.items():
        for it in items:
            if it in out:
                raise ValueError(f"{it} is assigned twice")
            out[it] = (DEST[dest_id], aisle, section)
    return out


def assign(doc):
    items = doc.get("items")
    if not isinstance(items, dict):
        raise ValueError("expected 'items:' to be a mapping of item name -> fields")
    plan = item_to_stop()
    rows, missing = [], []
    for name, rec in items.items():
        if name not in plan:
            missing.append(name)
            continue
        d, aisle, section = plan[name]
        rec["x"], rec["y"], rec["yaw"] = round(d.x, 3), round(d.y, 3), round(d.yaw, 4)
        rec["shelf"] = d.fixture
        rec["face"] = face_of(d.yaw)
        rec["aisle"] = aisle
        rec["section"] = section
        rec["stop"] = d.id
        rows.append((name, aisle, section, d))
    if "frame_id" in doc:
        doc["frame_id"] = "map"
    unplanned = sorted(set(plan) - set(items))
    return doc, rows, missing, unplanned


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--items", default=str(Path.home() / "slam_ws/src/arc_store_sim/config/item_locations.yaml"))
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args(argv)
    path = Path(args.items).expanduser()
    doc = yaml.safe_load(path.read_text())
    doc, rows, missing, unplanned = assign(doc)
    for name, aisle, section, d in sorted(rows, key=lambda r: (r[1], r[3].id, r[0])):
        print(f"  {aisle:8s} {section:24s} {name:18s} -> {d.id:18s} ({d.x:5.2f}, {d.y:5.2f}, {round(d.yaw * 180 / pi):4d}°)")
    print(f"\n{len(rows)} items placed on {len({r[3].id for r in rows})} stops.")
    if missing:
        print(f"NOT in the plan (left unchanged): {missing}")
    if unplanned:
        print(f"In the plan but not in your file: {unplanned}")
    if not args.write:
        print("DRY RUN: nothing written. Re-run with --write.")
        return 0
    # Backup goes to ~/slam_ws/arc_backups (not config/, which setup.py installs wholesale).
    bkdir = Path.home() / "slam_ws" / "arc_backups"
    bkdir.mkdir(parents=True, exist_ok=True)
    bk = bkdir / f"item_locations.before_aisles_{time.strftime('%Y%m%d_%H%M%S')}.yaml"
    shutil.copy2(path, bk)
    with open(path, "w") as f:
        f.write("# ARC compact store: items placed by aisle (assign_items.py). Frame: map.\n")
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False, allow_unicode=True)
    print(f"Backed up the previous file to {bk}\nWrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
