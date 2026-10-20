"""Make the two code changes the compact store needs, and nothing else.

1. dijkstra_planner.py: replace exactly these two lines inside shopping_goal_callback
       self.goal_x = msg.pose.position.x
       self.goal_y = msg.pose.position.y
   with a call to goal_frame_guard (accept map goals, transform via real TF, otherwise
   reject with a logged reason). Anything else in the callback is left untouched.

2. shopping_manager.py: change a hard-coded goal frame literal 'world' to 'map'.
   This is NOT a cosmetic rename: the item coordinates are regenerated in the map
   frame (destinations_map.yaml / remapped item_locations.yaml), and RUNBOOK step 5
   verifies that map and Gazebo world coincide under the spawn-at-origin convention.
   If the manager takes its frame from the YAML instead, nothing needs changing there
   (the remapped YAML already says map), and this step reports "no literal found".

Dry run by default (prints a unified diff). --apply writes the files; run install.sh
first so the originals are backed up. Each patch is applied only if its anchor text
is found exactly once; otherwise it is skipped and reported.
"""

import argparse
import difflib
import re
import sys
from pathlib import Path

GUARD_MARK = "ARC compact store: goal frame guard"

PLANNER_RE = re.compile(
    r"^(?P<ind>[ \t]+)self\.goal_x\s*=\s*msg\.pose\.position\.x[ \t]*\n"
    r"(?P=ind)self\.goal_y\s*=\s*msg\.pose\.position\.y[ \t]*$",
    re.M)

MANAGER_RE = re.compile(r"""(frame_id\s*=\s*)(['"])world\2""")


def callback_body(src: str, name="shopping_goal_callback"):
    m = re.search(rf"^(?P<ind>[ \t]*)def {name}\(.*?$", src, re.M)
    if not m:
        return ""
    ind = len(m.group("ind"))
    out = []
    for line in src[m.end():].splitlines()[1:]:
        if line.strip() and (len(line) - len(line.lstrip())) <= ind:
            break
        out.append(line)
    return "\n".join(out)


def has_equivalent_guard(src: str) -> bool:
    """True if the callback already checks header.frame_id and transforms to 'map' via TF."""
    body = callback_body(src)
    return ("header.frame_id" in body and "lookup_transform" in body
            and re.search(r"lookup_transform\(\s*['\"]map['\"]", body) is not None
            and "return" in body)


def patch_planner(src: str):
    if GUARD_MARK in src:
        return src, "already patched"
    if has_equivalent_guard(src):
        return src, ("equivalent goal-frame guard already present in shopping_goal_callback "
                     "(rejects empty frame, TF-transforms non-map frames): unchanged")
    hits = list(PLANNER_RE.finditer(src))
    if len(hits) != 1:
        return src, f"SKIPPED: expected the two goal_x/goal_y lines once, found {len(hits)}"
    ind = hits[0].group("ind")
    new = (f"{ind}# --- {GUARD_MARK} (arc_compact_store_kit) ---\n"
           f"{ind}from arc_planner.goal_frame_guard import resolve_goal_for_node\n"
           f"{ind}_res = resolve_goal_for_node(self, msg)\n"
           f"{ind}if not _res.ok:\n"
           f"{ind}    return\n"
           f"{ind}self.goal_x = _res.x\n"
           f"{ind}self.goal_y = _res.y\n"
           f"{ind}# --- end goal frame guard ---")
    return src[:hits[0].start()] + new + src[hits[0].end():], "patched shopping_goal_callback"


def patch_manager(src: str):
    hits = list(MANAGER_RE.finditer(src))
    if not hits:
        return src, "no hard-coded 'world' frame literal found (frame may come from the YAML): unchanged"
    out = MANAGER_RE.sub(lambda m: f"{m.group(1)}{m.group(2)}map{m.group(2)}", src)
    return out, f"changed {len(hits)} frame_id literal(s) 'world' -> 'map'"


def run(path: Path, fn, apply: bool):
    if not path.is_file():
        print(f"[MISSING] {path}")
        return False
    src = path.read_text()
    out, msg = fn(src)
    print(f"[{'CHANGE' if out != src else 'NOCHANGE'}] {path}: {msg}")
    if out != src:
        sys.stdout.writelines(difflib.unified_diff(src.splitlines(True), out.splitlines(True),
                                                   str(path), str(path) + " (patched)"))
        if apply:
            path.write_text(out)
            print(f"  written: {path}")
    return "SKIPPED" not in msg


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ws", default=str(Path.home() / "slam_ws"))
    ap.add_argument("--apply", action="store_true")
    args = ap.parse_args(argv)
    base = Path(args.ws).expanduser() / "src" / "arc_planner" / "arc_planner"
    ok1 = run(base / "dijkstra_planner.py", patch_planner, args.apply)
    ok2 = run(base / "shopping_manager.py", patch_manager, args.apply)
    if not args.apply:
        print("\nDRY RUN: nothing written. Re-run with --apply after install.sh has made backups.")
    return 0 if ok1 and ok2 else 1


if __name__ == "__main__":
    sys.exit(main())
