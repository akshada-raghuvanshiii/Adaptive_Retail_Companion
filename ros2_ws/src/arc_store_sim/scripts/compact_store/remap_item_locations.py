"""Move every item in YOUR existing item_locations.yaml to a destination in the
compact store, keeping item IDs, categories, shelf mappings and all other fields.

It does not assume one fixed schema. An "item record" is any mapping that holds
numeric x and y, either directly (x:, y:), under position:/pose.position:
(x:, y:), or as a list (position: [x, y, z]). For each record it:
  * keeps every key and value except the coordinates (and orientation, frame);
  * picks a destination by keywords in the item's id/name/category/shelf fields
    (fruit -> produce, milk -> dairy, tea/coffee -> tea_coffee, checkout -> checkout);
  * spreads everything else over the gondola stops, grouped by the item's existing
    shelf/aisle value so items that shared a shelf still share a stop;
  * sets an existing orientation field (yaw/theta/heading or quaternion z,w) to face
    the fixture, and an existing frame field (frame/frame_id) to "map".

Dry run by default: prints the mapping and writes nothing. With --write it writes
--out (never the input file). It refuses if it finds no item records, or if any
record's coordinates cannot be read.

    python3 remap_item_locations.py ~/slam_ws/src/arc_store_sim/config/item_locations.yaml
    python3 remap_item_locations.py <in.yaml> --write --out item_locations.compact.yaml [--add-yaw]
"""

import argparse
import csv
import sys
from math import cos, pi, sin

import yaml

import layout as L

CATEGORY_KEYS = ("category", "section", "department", "type", "zone", "group")
SHELF_KEYS = ("shelf", "shelf_id", "aisle", "rack", "gondola", "location", "zone")
ID_KEYS = ("id", "item_id", "name", "item", "label", "product")
YAW_KEYS = ("yaw", "theta", "heading")
FRAME_KEYS = ("frame", "frame_id")

SPECIAL = [d for d in L.DESTINATIONS if d.category != "gondola"]


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def coord_slot(rec):
    """Return (kind, container) describing where x/y live, or None."""
    if isinstance(rec, dict):
        if _num(rec.get("x")) and _num(rec.get("y")):
            return ("xy", rec)
        pos = rec.get("position")
        if isinstance(pos, dict) and _num(pos.get("x")) and _num(pos.get("y")):
            return ("xy", pos)
        if isinstance(pos, list) and len(pos) >= 2 and _num(pos[0]) and _num(pos[1]):
            return ("list", pos)
        pose = rec.get("pose")
        if isinstance(pose, dict):
            p = pose.get("position")
            if isinstance(p, dict) and _num(p.get("x")) and _num(p.get("y")):
                return ("xy", p)
            if isinstance(p, list) and len(p) >= 2 and _num(p[0]) and _num(p[1]):
                return ("list", p)
            if _num(pose.get("x")) and _num(pose.get("y")):
                return ("xy", pose)
    return None


def find_records(node, path=()):
    """Yield (path, record) for every item record, outermost first; do not descend into a record."""
    if isinstance(node, dict):
        if coord_slot(node) is not None and not _is_coordinate_leaf(node, path):
            yield path, node
            return
        for k, v in node.items():
            yield from find_records(v, path + (k,))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from find_records(v, path + (i,))


def _is_coordinate_leaf(node, path):
    # {x:, y:} directly under "position"/"pose" belongs to its parent record
    return bool(path) and path[-1] in ("position", "pose") and set(node) <= {"x", "y", "z", "w"}


def text_fields(path, rec):
    ident = next((str(rec[k]) for k in ID_KEYS if k in rec), None)
    if ident is None:
        ident = next((str(p) for p in reversed(path) if isinstance(p, str)), "/".join(map(str, path)))
    cat = " ".join(str(rec[k]) for k in CATEGORY_KEYS if k in rec)
    shelf = next((str(rec[k]) for k in SHELF_KEYS if k in rec), "")
    return ident, cat, shelf


def choose(ident, cat, shelf, gondola_assign):
    hay = f"{ident} {cat} {shelf}".lower()
    for d in SPECIAL:
        for kw in d.keywords:
            if kw in hay:
                return d, f"keyword '{kw}'"
    key = shelf or cat or "unassigned"
    return gondola_assign(key), f"gondola by shelf '{key}'" if shelf else f"gondola by group '{key}'"


def make_gondola_assigner():
    stops = L.GONDOLA_DESTINATIONS
    seen = {}

    def assign(key):
        if key not in seen:
            seen[key] = stops[len(seen) % len(stops)]
        return seen[key]
    return assign


def set_pose(rec, d, add_yaw):
    kind, slot = coord_slot(rec)
    old = (float(slot["x"]), float(slot["y"])) if kind == "xy" else (float(slot[0]), float(slot[1]))
    if kind == "xy":
        slot["x"], slot["y"] = round(d.x, 3), round(d.y, 3)
    else:
        slot[0], slot[1] = round(d.x, 3), round(d.y, 3)
    yaw_set = False
    containers = [rec] + [rec.get("pose")] if isinstance(rec.get("pose"), dict) else [rec]
    for c in containers:
        for k in YAW_KEYS:
            if k in c and _num(c[k]):
                c[k] = round(d.yaw, 4)
                yaw_set = True
        o = c.get("orientation")
        if isinstance(o, dict) and "z" in o and "w" in o:
            o.update({"x": 0.0, "y": 0.0, "z": round(sin(d.yaw / 2), 6), "w": round(cos(d.yaw / 2), 6)})
            yaw_set = True
        elif _num(o):
            c["orientation"] = round(d.yaw, 4)
            yaw_set = True
    if not yaw_set and add_yaw:
        rec["yaw"] = round(d.yaw, 4)
        yaw_set = True
    for k in FRAME_KEYS:
        if k in rec:
            rec[k] = "map"
    return old, yaw_set


def remap(doc, add_yaw=False):
    records = list(find_records(doc))
    if not records:
        raise ValueError("No item records with numeric x/y found: structure not recognised. Nothing written.")
    assign = make_gondola_assigner()
    rows = []
    for path, rec in records:
        ident, cat, shelf = text_fields(path, rec)
        d, why = choose(ident, cat, shelf, assign)
        old, yaw_set = set_pose(rec, d, add_yaw)
        rows.append({"item": ident, "path": "/".join(map(str, path)), "category": cat, "shelf": shelf,
                     "old_x": old[0], "old_y": old[1], "destination": d.id, "new_x": round(d.x, 3),
                     "new_y": round(d.y, 3), "yaw_deg": round(d.yaw * 180 / pi), "orientation_set": yaw_set,
                     "reason": why})
    # top-level frame field, if the file has one
    if isinstance(doc, dict):
        for k in FRAME_KEYS:
            if k in doc:
                doc[k] = "map"
    return doc, rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("input")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--out", default="item_locations.compact.yaml")
    ap.add_argument("--report", default="item_remap_report.csv")
    ap.add_argument("--add-yaw", action="store_true",
                    help="add a yaw field to records that have no orientation field")
    args = ap.parse_args(argv)

    with open(args.input) as f:
        doc = yaml.safe_load(f)
    try:
        doc, rows = remap(doc, args.add_yaw)
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2

    w = max(len(r["item"]) for r in rows)
    print(f"{'item':<{w}}  {'old (x, y)':>16}  ->  {'destination':<18} {'new (x, y, yaw)':>22}  reason")
    for r in rows:
        print(f"{r['item']:<{w}}  ({r['old_x']:6.2f},{r['old_y']:6.2f})  ->  {r['destination']:<18} "
              f"({r['new_x']:5.2f},{r['new_y']:5.2f},{r['yaw_deg']:5d}°)  {r['reason']}"
              f"{'' if r['orientation_set'] else '  [no orientation field: use --add-yaw?]'}")
    used = sorted({r["destination"] for r in rows})
    unused = [d.id for d in L.DESTINATIONS if d.id not in used]
    print(f"\n{len(rows)} items -> {len(used)} destinations. Unused destinations: {unused or 'none'}")
    with open(args.report, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    print(f"Mapping report: {args.report}")
    if not args.write:
        print("DRY RUN: nothing written. Re-run with --write to create", args.out)
        return 0
    if args.out == args.input:
        print("REFUSED: --out must differ from the input; install.sh handles backup and replacement.")
        return 2
    with open(args.out, "w") as f:
        f.write("# Remapped to the ARC compact store by remap_item_locations.py. Frame: map.\n")
        f.write(f"# Source: {args.input} (comments in the source are not carried over)\n")
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False, allow_unicode=True)
    print(f"Wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
