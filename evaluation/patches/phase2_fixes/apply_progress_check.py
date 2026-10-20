#!/usr/bin/env python3
"""Phase 3, patch 1: set ARC's progress check explicitly to 0.25 m within 20 s.

    python3 apply_progress_check.py            # dry run: shows the change, writes nothing
    python3 apply_progress_check.py --write    # backup -> ~/slam_ws/arc_backups/, then patch

ARC's teb_controller.yaml defines no progress checker, so Nav2 uses its default
(SimpleProgressChecker, 0.5 m within 10 s). Every one of the 62 A2 aborts was that check.
This inserts four lines under controller_server.ros__parameters, right after the
controller_plugins line, and changes nothing else in the file (comments and order kept):

    progress_checker_plugins: ["progress_checker"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.25
      movement_time_allowance: 20.0

These are the exact values the phase-2 runs (A2-pc, A2-final) used through the test kit's copy.
Nothing is written if: the file is missing, a progress checker is already set, the anchor line is
not found exactly once, or the patched file does not parse back to those values.
"""
import argparse
import difflib
import os
import re
import shutil
import sys
import time

DEFAULT = os.path.expanduser("~/slam_ws/src/arc_planner/config/teb_controller.yaml")
BACKUPS = os.path.expanduser("~/slam_ws/arc_backups")
RADIUS, ALLOW = 0.25, 20.0


def patch_text(text, radius=RADIUS, allow=ALLOW):
    """Return the patched text. Raises ValueError with the reason if it cannot be patched safely."""
    if re.search(r"^\s*progress_checker", text, re.M):
        raise ValueError("a progress_checker is already set in this file; nothing to do (check its values)")
    lines = text.splitlines(keepends=True)
    # the controller_plugins line inside controller_server: ros__parameters:
    in_cs = in_params = False
    hits = []
    for i, ln in enumerate(lines):
        if re.match(r"^controller_server:\s*$", ln):
            in_cs, in_params = True, False
            continue
        if in_cs and re.match(r"^\S", ln):          # next top-level key
            in_cs = in_params = False
        if in_cs and re.match(r"^\s+ros__parameters:\s*$", ln):
            in_params = True
            continue
        if in_params and re.match(r"^(\s+)controller_plugins\s*:", ln):
            hits.append(i)
    if len(hits) != 1:
        raise ValueError(f"expected one 'controller_plugins:' under controller_server.ros__parameters, found {len(hits)}")
    i = hits[0]
    ind = re.match(r"^(\s+)", lines[i]).group(1)
    nl = "\n" if lines[i].endswith("\n") else ""
    block = [
        nl,
        f"{ind}# Phase 3: explicit progress check (Nav2 default 0.5 m / 10 s caused all 62 A2 aborts).\n",
        f"{ind}progress_checker_plugins: [\"progress_checker\"]\n",
        f"{ind}progress_checker:\n",
        f"{ind}  plugin: \"nav2_controller::SimpleProgressChecker\"\n",
        f"{ind}  required_movement_radius: {radius}\n",
        f"{ind}  movement_time_allowance: {allow}\n",
    ]
    if not nl:
        block = ["\n"] + block[1:]
    return "".join(lines[: i + 1] + block + lines[i + 1:])


def check(patched, original):
    import yaml
    new = yaml.safe_load(patched)
    old = yaml.safe_load(original)
    p = new["controller_server"]["ros__parameters"]
    assert p["progress_checker_plugins"] == ["progress_checker"], "plugins list wrong"
    pc = p["progress_checker"]
    assert pc["plugin"] == "nav2_controller::SimpleProgressChecker", "plugin wrong"
    assert pc["required_movement_radius"] == RADIUS and pc["movement_time_allowance"] == ALLOW, "values wrong"
    # nothing else changed
    del p["progress_checker_plugins"], p["progress_checker"]
    assert new == old, "something other than the progress check changed"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", default=DEFAULT)
    ap.add_argument("--write", action="store_true", help="back up and patch (default: dry run)")
    a = ap.parse_args()
    if not os.path.isfile(a.file):
        sys.exit(f"not found: {a.file}")
    original = open(a.file).read()
    try:
        patched = patch_text(original)
        check(patched, original)
    except (ValueError, AssertionError, KeyError) as e:
        sys.exit(f"NOT patched: {e}")
    sys.stdout.writelines(difflib.unified_diff(original.splitlines(True), patched.splitlines(True),
                                               a.file, a.file + " (patched)"))
    print("\nparsed back OK: only progress_checker_plugins + progress_checker added")
    if not a.write:
        print("dry run: nothing written. Run again with --write to apply.")
        return
    os.makedirs(BACKUPS, exist_ok=True)
    bak = os.path.join(BACKUPS, f"teb_controller.before_progress_check_{time.strftime('%Y%m%d_%H%M%S')}.yaml")
    shutil.copy2(a.file, bak)
    with open(a.file, "w") as f:
        f.write(patched)
    print(f"backup: {bak}\npatched: {a.file}")
    print(f"undo:   cp {bak} {a.file} && cd ~/slam_ws && colcon build --packages-select arc_planner")
    print("next:   cd ~/slam_ws && colcon build --packages-select arc_planner && source install/setup.bash")


if __name__ == "__main__":
    main()
