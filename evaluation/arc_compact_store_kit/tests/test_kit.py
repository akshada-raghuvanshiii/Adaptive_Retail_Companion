import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))

import generate_compact_store as G  # noqa: E402
import layout as L  # noqa: E402

OLD_WORLD = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="arc_store">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose></light>
    <model name="ground_plane"><static>true</static><link name="link"/></model>
    <model name="old_shelf_A1"><static>true</static><link name="l"/></model>
    <model name="old_shelf_A2"><static>true</static><link name="l"/></model>
    <include><name>mannequin_1</name><uri>model://mannequin</uri><pose>2 2 0 0 0 0</pose></include>
    <include><uri>model://old_fridge</uri></include>
  </world>
</sdf>
"""


def collision_boxes(sdf_path):
    """Return {model_name: (x0, y0, x1, y1, height)} from the generated SDF."""
    out = {}
    world = ET.parse(sdf_path).getroot().find("world")
    for m in world.findall("model"):
        size = m.find("link/collision/geometry/box/size")
        if size is None:
            continue
        sx, sy, sz = map(float, size.text.split())
        px, py = map(float, m.findtext("pose").split()[:2])
        lz = float(m.findtext("link/pose").split()[2])
        out[m.get("name")] = (px - sx / 2, py - sy / 2, px + sx / 2, py + sy / 2, sz, lz)
    return out


class GeneratorTests(unittest.TestCase):
    def test_standalone_world_matches_layout(self):
        with tempfile.TemporaryDirectory() as d:
            G.main(["--out-dir", d])
            boxes = collision_boxes(Path(d) / "arc_store_compact.sdf")
            for f in L.ALL_OBSTACLES:
                self.assertIn(f.name, boxes)
                x0, y0, x1, y1, h, lz = boxes[f.name]
                for a, b in ((x0, f.x0), (y0, f.y0), (x1, f.x1), (y1, f.y1), (h, f.height)):
                    self.assertAlmostEqual(a, b, places=3)
                self.assertAlmostEqual(lz, f.height / 2, places=3)   # sits on the floor
            world = ET.parse(Path(d) / "arc_store_compact.sdf").getroot().find("world")
            plugins = {p.get("name") for p in world.findall("plugin")}
            self.assertIn("gz::sim::systems::Sensors", plugins)
            self.assertIsNotNone(world.find("model[@name='ground_plane']"))

    def test_visuals_inside_collision_footprint(self):
        """gpu_lidar sees visuals: no visual may stick out of its collision box in x/y."""
        with tempfile.TemporaryDirectory() as d:
            G.main(["--out-dir", d])
            world = ET.parse(Path(d) / "arc_store_compact.sdf").getroot().find("world")
            for m in world.findall("model"):
                col = m.find("link/collision/geometry/box/size")
                if col is None:
                    continue
                cx, cy, _ = map(float, col.text.split())
                for v in m.findall("link/visual"):
                    vx, vy, _ = map(float, v.find("geometry/box/size").text.split())
                    px, py = (map(float, v.findtext("pose").split()[:2]) if v.find("pose") is not None
                              else (0.0, 0.0))
                    self.assertLessEqual(abs(px) + vx / 2, cx / 2 + 1e-6, f"{m.get('name')}/{v.get('name')}")
                    self.assertLessEqual(abs(py) + vy / 2, cy / 2 + 1e-6, f"{m.get('name')}/{v.get('name')}")

    def test_template_keeps_settings_drops_old_store(self):
        with tempfile.TemporaryDirectory() as d:
            tpl = Path(d) / "old.sdf"
            tpl.write_text(OLD_WORLD)
            G.main(["--out-dir", d, "--template", str(tpl), "--keep-model", "mannequin_1"])
            world = ET.parse(Path(d) / "arc_store_compact.sdf").getroot().find("world")
            self.assertEqual(world.get("name"), "arc_store")
            plugins = {p.get("name") for p in world.findall("plugin")}
            self.assertIn("gz::sim::systems::Contact", plugins)          # copied from template
            names = {m.get("name") for m in world.findall("model")}
            self.assertNotIn("old_shelf_A1", names)
            self.assertIn("ground_plane", names)
            self.assertIn("gondola_1", names)
            includes = [i.findtext("name") or i.findtext("uri") for i in world.findall("include")]
            self.assertEqual(includes, ["mannequin_1"])                   # old fridge dropped

    def test_destinations_yaml_and_json_consistent(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            G.main(["--out-dir", d])
            data = json.loads((Path(d) / "store_layout_compact.json").read_text())
            ids = [x["id"] for x in data["destinations"]]
            self.assertEqual(ids, [x.id for x in L.DESTINATIONS])
            yml = (Path(d) / "destinations_map.yaml").read_text()
            for x in L.DESTINATIONS:
                self.assertIn(f"  {x.id}:", yml)
                self.assertIn(f"x: {x.x:.3f}", yml)
            self.assertIn("frame_id: map", yml)


if __name__ == "__main__":
    unittest.main()
