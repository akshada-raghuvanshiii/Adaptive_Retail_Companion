"""Generate the compact ARC store from layout.py.

Writes (into --out-dir):
  arc_store_compact.sdf        Gazebo Harmonic world
  store_layout_compact.json    fixtures + destinations + frame convention (machine-readable)
  destinations_map.yaml        canonical destination poses, frame "map"

World-level settings
  With --template <your existing arc_store.sdf>, every world-level element of your
  current world is copied (physics, plugins such as Sensors/IMU/Contact, scene, lights,
  GUI, ground plane), and its <model>/<include> entries are DROPPED except those named
  with --keep-model (e.g. a mannequin or a robot embedded in the world). This keeps the
  setup your robot's sensors already depend on.
  Without --template, standard Harmonic defaults are written (Physics, UserCommands,
  SceneBroadcaster, Sensors[ogre2], Imu, sun, ground plane).

    python3 generate_compact_store.py --out-dir output \
        [--template ~/slam_ws/src/arc_store_sim/worlds/arc_store.sdf --keep-model mannequin] \
        [--list-template]
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from math import pi
from pathlib import Path

import layout as L

WORLD_NAME_DEFAULT = "arc_store"


def _box_link(parent, name, size, pose_xyz, colour, collide=True, visual_only_parts=(), visual_inset=0.0):
    link = ET.SubElement(parent, "link", name=name)
    ET.SubElement(link, "pose").text = f"{pose_xyz[0]:.4f} {pose_xyz[1]:.4f} {pose_xyz[2]:.4f} 0 0 0"
    if collide:
        col = ET.SubElement(link, "collision", name="collision")
        ET.SubElement(ET.SubElement(ET.SubElement(col, "geometry"), "box"), "size").text = \
            f"{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}"
    vis = ET.SubElement(link, "visual", name="visual")
    ET.SubElement(ET.SubElement(ET.SubElement(vis, "geometry"), "box"), "size").text = \
        f"{size[0] - visual_inset:.4f} {size[1] - visual_inset:.4f} {size[2]:.4f}"
    _material(vis, colour)
    for i, (psize, ppose, pcol) in enumerate(visual_only_parts):
        v = ET.SubElement(link, "visual", name=f"detail_{i}")
        ET.SubElement(v, "pose").text = f"{ppose[0]:.4f} {ppose[1]:.4f} {ppose[2]:.4f} 0 0 0"
        ET.SubElement(ET.SubElement(ET.SubElement(v, "geometry"), "box"), "size").text = \
            f"{psize[0]:.4f} {psize[1]:.4f} {psize[2]:.4f}"
        _material(v, pcol)
    return link


def _material(vis, rgb):
    m = ET.SubElement(vis, "material")
    c = f"{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1"
    ET.SubElement(m, "ambient").text = c
    ET.SubElement(m, "diffuse").text = c
    ET.SubElement(m, "specular").text = "0.1 0.1 0.1 1"


PRODUCT_COLOURS = [(0.85, 0.25, 0.2), (0.95, 0.75, 0.2), (0.25, 0.55, 0.85), (0.3, 0.7, 0.4), (0.9, 0.9, 0.85)]


def fixture_model(world, f):
    """One static model per fixture: ONE solid collision box plus visual-only shelf boards
    and product rows for realism. Gazebo's gpu_lidar ray-casts VISUALS, so every visual
    stays inside the collision footprint: the mapped outline equals the collision outline."""
    model = ET.SubElement(world, "model", name=f.name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{f.centre[0]:.4f} {f.centre[1]:.4f} 0 0 0 0"
    sx, sy = f.size
    parts = []
    if f.kind in ("gondola", "wall_shelf"):
        # shelf boards and product rows sticking out slightly from the faces (visual only)
        for k, z in enumerate((0.35, 0.75, 1.15)):
            parts.append(((sx * 0.999, sy * 0.999, 0.02), (0, 0, z - f.height / 2), (0.75, 0.75, 0.75)))
            n = max(2, int(sy / 0.25))
            for i in range(n):
                y = -sy / 2 + (i + 0.5) * sy / n
                col = PRODUCT_COLOURS[(i + k) % len(PRODUCT_COLOURS)]
                parts.append(((sx * 0.999, sy / n * 0.8, 0.16), (0, y, z + 0.09 - f.height / 2), col))
    elif f.kind == "produce":
        for i, col in enumerate([(0.9, 0.2, 0.15), (0.95, 0.6, 0.1), (0.95, 0.85, 0.2), (0.3, 0.65, 0.2)]):
            x = -sx / 2 + (i % 2 + 0.5) * sx / 2
            y = -sy / 2 + (i // 2 + 0.5) * sy / 2
            parts.append(((sx / 2 * 0.85, sy / 2 * 0.85, 0.12), (x, y, f.height / 2 + 0.06), col))
    elif f.kind == "fridge":
        parts.append(((sx * 0.95, 0.008, f.height * 0.85), (0, -sy / 2 + 0.005, 0), (0.75, 0.9, 1.0)))
    inset = 0.02 if f.kind in ("gondola", "wall_shelf", "produce", "fridge") else 0.0
    _box_link(model, "link", (sx, sy, f.height), (0, 0, f.height / 2), f.colour, True, parts, inset)
    return model


def default_world(name):
    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name=name)
    phys = ET.SubElement(world, "physics", name="1ms", type="ignored")
    ET.SubElement(phys, "max_step_size").text = "0.001"
    ET.SubElement(phys, "real_time_factor").text = "1.0"
    for fname, pname in (("gz-sim-physics-system", "gz::sim::systems::Physics"),
                         ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
                         ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
                         ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
                         ("gz-sim-imu-system", "gz::sim::systems::Imu")):
        p = ET.SubElement(world, "plugin", filename=fname, name=pname)
        if pname.endswith("Sensors"):
            ET.SubElement(p, "render_engine").text = "ogre2"
    light = ET.SubElement(world, "light", type="directional", name="sun")
    ET.SubElement(light, "cast_shadows").text = "true"
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.9 0.9 0.9 1"
    ET.SubElement(light, "specular").text = "0.2 0.2 0.2 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"
    ground = ET.SubElement(world, "model", name="ground_plane")
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", name="link")
    for tag in ("collision", "visual"):
        el = ET.SubElement(link, tag, name=tag)
        plane = ET.SubElement(ET.SubElement(el, "geometry"), "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = "40 40"
        if tag == "visual":
            _material(el, (0.82, 0.80, 0.76))
    return sdf, world


def from_template(path, keep):
    tree = ET.parse(path)
    sdf = tree.getroot()
    world = sdf.find("world")
    if world is None:
        raise SystemExit(f"No <world> in {path}")
    dropped, kept = [], []
    for el in list(world):
        if el.tag in ("model", "include", "actor"):
            nm = el.get("name") or (el.findtext("name") or el.findtext("uri") or "")
            is_ground = "ground" in nm.lower() or "ground_plane" in (el.findtext("uri") or "").lower()
            is_sun = "sun" in (el.findtext("uri") or "").lower()
            if is_ground or is_sun or any(k == nm or k in nm for k in keep):
                kept.append(nm)
            else:
                world.remove(el)
                dropped.append(nm)
    has_ground = any("ground" in ((e.get("name") or e.findtext("name") or e.findtext("uri") or "").lower())
                     for e in world if e.tag in ("model", "include"))
    return sdf, world, kept, dropped, has_ground


def list_template(path):
    world = ET.parse(path).getroot().find("world")
    print(f"World name: {world.get('name')}")
    for el in world:
        nm = el.get("name") or el.findtext("name") or el.findtext("uri") or ""
        extra = el.get("filename", "") if el.tag == "plugin" else ""
        print(f"  <{el.tag}> {nm} {extra}")


def indent(el, level=0):
    pad = "\n" + "  " * level
    if len(el):
        if not (el.text or "").strip():
            el.text = pad + "  "
        for child in el:
            indent(child, level + 1)
        if not (child.tail or "").strip():
            child.tail = pad
    if level and not (el.tail or "").strip():
        el.tail = pad


def destinations_yaml(path):
    lines = [
        "# Generated by generate_compact_store.py from layout.py. Do not hand-edit; edit layout.py.",
        "# Frame: map. Valid in Gazebo world coordinates too ONLY under the spawn-at-origin",
        "# convention verified in RUNBOOK step 5.",
        "frame_id: map",
        "destinations:",
    ]
    for d in L.DESTINATIONS:
        lines += [f"  {d.id}:",
                  f"    label: \"{d.label}\"",
                  f"    category: {d.category}",
                  f"    fixture: {d.fixture}",
                  f"    x: {d.x:.3f}",
                  f"    y: {d.y:.3f}",
                  f"    yaw: {d.yaw:.4f}   # {d.yaw * 180 / pi:.0f} deg, facing the fixture"]
    Path(path).write_text("\n".join(lines) + "\n")


def layout_json(path, world_file):
    data = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "generator": "arc_compact_store_kit/generate_compact_store.py",
        "world_file": world_file,
        "frame_convention": {
            "frame_id": "map",
            "statement": "Gazebo world origin = robot spawn (0,0,yaw 0) = odom origin = RTAB-Map map origin "
                         "at start of mapping. Must be verified (RUNBOOK step 5).",
            "spawn": {"x": L.SPAWN[0], "y": L.SPAWN[1], "yaw": L.SPAWN[2]},
        },
        "store_bounds": {"x": list(L.STORE_X), "y": list(L.STORE_Y), "wall_thickness": L.WALL_T},
        "fixtures": [{"name": f.name, "kind": f.kind, "x_min": f.x0, "y_min": f.y0, "x_max": f.x1,
                      "y_max": f.y1, "height": f.height} for f in L.ALL_OBSTACLES],
        "destinations": [{"id": d.id, "category": d.category, "fixture": d.fixture, "label": d.label,
                          "x": round(d.x, 3), "y": round(d.y, 3), "yaw": round(d.yaw, 4)}
                         for d in L.DESTINATIONS],
    }
    Path(path).write_text(json.dumps(data, indent=2) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", default="output")
    ap.add_argument("--template", help="your existing world SDF: copy its world-level settings")
    ap.add_argument("--keep-model", action="append", default=[],
                    help="model/include name in the template to keep (repeatable)")
    ap.add_argument("--world-name", help="override the <world name>; default: template's, else arc_store")
    ap.add_argument("--list-template", action="store_true", help="list the template's elements and exit")
    args = ap.parse_args(argv)

    if args.list_template:
        if not args.template:
            ap.error("--list-template needs --template")
        list_template(args.template)
        return 0

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if args.template:
        sdf, world, kept, dropped, has_ground = from_template(args.template, args.keep_model)
        print(f"Template: {args.template}\n  kept: {kept or 'none'}\n  dropped (old store): {len(dropped)} elements")
        if not has_ground:
            _, w2 = default_world("tmp")
            world.append(w2.find("model[@name='ground_plane']"))
            print("  template had no ground plane: added one")
    else:
        sdf, world = default_world(WORLD_NAME_DEFAULT)
        print("No template: standard Harmonic world settings written. Compare plugins with your old world!")
    if args.world_name:
        world.set("name", args.world_name)

    for f in L.WALLS + L.FIXTURES:
        fixture_model(world, f)

    indent(sdf)
    world_path = out / "arc_store_compact.sdf"
    header = (f"<?xml version=\"1.0\"?>\n<!-- Generated by arc_compact_store_kit/generate_compact_store.py "
              f"from layout.py. Edit layout.py and regenerate; do not hand-edit. World name: "
              f"{world.get('name')} -->\n")
    world_path.write_text(header + ET.tostring(sdf, encoding="unicode") + "\n")
    layout_json(out / "store_layout_compact.json", world_path.name)
    destinations_yaml(out / "destinations_map.yaml")
    print(f"Wrote {world_path}, {out / 'store_layout_compact.json'}, {out / 'destinations_map.yaml'}")
    print(f"World name in SDF: {world.get('name')}  (your launch file / bridge topics may depend on it)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
