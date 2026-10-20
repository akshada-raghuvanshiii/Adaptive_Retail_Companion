import sys
import unittest
from math import isclose
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import layout as L  # noqa: E402
from remap_item_locations import remap  # noqa: E402

DEST = {d.id: d for d in L.DESTINATIONS}

SCHEMA_A = """
items:
  - {id: 101, name: Apples, category: fruit, shelf: P1, x: 1.9, y: 7.2, yaw: 0.0}
  - {id: 102, name: Full cream milk, category: dairy, shelf: F1, x: -6.0, y: 3.0, yaw: 1.57}
  - {id: 103, name: Rice 5kg, category: grains, shelf: A2, x: 4.0, y: -2.0, yaw: 0.0}
  - {id: 104, name: Basmati rice, category: grains, shelf: A2, x: 4.2, y: -2.0, yaw: 0.0}
  - {id: 105, name: Detergent, category: household, shelf: B1, x: 9.0, y: 1.0, yaw: 0.0}
"""

SCHEMA_B = """
frame_id: world
items:
  banana: {category: fruit, position: [1.9, 7.2, 0.0]}
  green_tea: {category: beverages, position: [10.0, -3.0, 0.0]}
  billing: {category: checkout, position: [0.5, 0.5, 0.0]}
"""

SCHEMA_C = """
products:
  coffee_powder:
    shelf: C3
    pose:
      position: {x: 8.0, y: 2.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  biscuits:
    shelf: C3
    pose:
      position: {x: 8.2, y: 2.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"""


class RemapTests(unittest.TestCase):
    def test_list_schema_keywords_and_shelf_grouping(self):
        doc, rows = remap(yaml.safe_load(SCHEMA_A))
        by = {r["item"]: r for r in rows}
        self.assertEqual(by["101"]["destination"], "produce")
        self.assertEqual(by["102"]["destination"], "dairy")
        self.assertEqual(by["103"]["destination"], by["104"]["destination"])   # same shelf, same stop
        self.assertNotEqual(by["103"]["destination"], by["105"]["destination"])
        item = doc["items"][0]
        self.assertEqual((item["id"], item["name"], item["category"]), (101, "Apples", "fruit"))
        self.assertEqual((item["shelf"], item["old_shelf"]), ("produce_display", "P1"))
        self.assertAlmostEqual(item["x"], round(DEST["produce"].x, 3))
        self.assertAlmostEqual(item["yaw"], round(DEST["produce"].yaw, 4))

    def test_position_list_and_frame_field(self):
        doc, rows = remap(yaml.safe_load(SCHEMA_B))
        self.assertEqual(doc["frame_id"], "map")
        self.assertEqual({r["item"]: r["destination"] for r in rows},
                         {"banana": "produce", "green_tea": "tea_coffee", "billing": "checkout"})
        self.assertEqual(doc["items"]["banana"]["position"][2], 0.0)       # z kept
        self.assertFalse(rows[0]["orientation_set"])                        # nothing to set, not added

    def test_pose_with_quaternion(self):
        doc, rows = remap(yaml.safe_load(SCHEMA_C))
        cp = doc["products"]["coffee_powder"]["pose"]
        d = DEST["tea_coffee"]
        self.assertTrue(isclose(cp["position"]["x"], round(d.x, 3)))
        # yaw pi -> quaternion (0, 0, 1, 0)
        self.assertAlmostEqual(cp["orientation"]["z"], 1.0, places=5)
        self.assertAlmostEqual(cp["orientation"]["w"], 0.0, places=5)
        self.assertEqual(doc["products"]["biscuits"]["old_shelf"], "C3")
        self.assertEqual(doc["products"]["biscuits"]["shelf"], DEST[rows[1]["destination"]].fixture)
        self.assertEqual(len(rows), 2)

    def test_add_yaw_option(self):
        doc, rows = remap(yaml.safe_load(SCHEMA_B), add_yaw=True)
        self.assertIn("yaw", doc["items"]["banana"])
        self.assertTrue(all(r["orientation_set"] for r in rows))

    def test_real_arc_item_file(self):
        """The ARC laptop's original item_locations.yaml (29 items, frame world)."""
        src = Path(__file__).parent / "data" / "arc_item_locations_original.yaml"
        original = yaml.safe_load(src.read_text())
        doc, rows = remap(yaml.safe_load(src.read_text()))
        self.assertEqual(doc["frame_id"], "map")
        self.assertEqual(list(doc["items"]), list(original["items"]))          # same IDs, same order
        by = {r["item"]: r["destination"] for r in rows}
        for k in ("fruits", "vegetables"):
            self.assertEqual(by[k], "produce")
        for k in ("milk", "curd_paneer", "eggs", "frozen"):
            self.assertEqual(by[k], "dairy")
        for k in ("tea_coffee", "juice", "water_beverages"):
            self.assertEqual(by[k], "tea_coffee")
        for name, item in doc["items"].items():
            d = DEST[by[name]]
            self.assertEqual((item["x"], item["y"], item["yaw"]), (round(d.x, 3), round(d.y, 3), round(d.yaw, 4)))
            self.assertEqual(item["shelf"], d.fixture)
            self.assertEqual(item["old_shelf"], original["items"][name]["shelf"])
            self.assertEqual(item["old_face"], original["items"][name]["face"])
        # items that shared an old shelf still share a stop (non-keyword items)
        g = {}
        for r in rows:
            if r["reason"].startswith("gondola"):
                g.setdefault(r["shelf"], set()).add(r["destination"])
        self.assertTrue(all(len(v) == 1 for v in g.values()), g)

    def test_unrecognised_structure_refused(self):
        with self.assertRaises(ValueError):
            remap(yaml.safe_load("items: [apple, milk, tea]"))

    def test_every_new_position_is_a_verified_destination(self):
        for src in (SCHEMA_A, SCHEMA_B, SCHEMA_C):
            _, rows = remap(yaml.safe_load(src))
            for r in rows:
                d = DEST[r["destination"]]
                self.assertEqual((r["new_x"], r["new_y"]), (round(d.x, 3), round(d.y, 3)))


if __name__ == "__main__":
    unittest.main()
