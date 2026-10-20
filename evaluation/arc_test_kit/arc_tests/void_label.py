#!/usr/bin/env python3
"""Set aside every row of one label (a run that must not count), keeping it as evidence.

    python3 arc_tests/void_label.py results teb_icp_boxes "why"

Rows move from samples/legs/missions/events.csv into results/void/<label>_<time>/,
so analyze.py and the notebook summary no longer see them. Nothing is deleted.
"""
import csv
import os
import sys
import time

out, label = sys.argv[1], sys.argv[2]
why = " ".join(sys.argv[3:]) or "no reason given"
dest = os.path.join(out, "void", f"{label}_{time.strftime('%Y%m%d_%H%M%S')}")
moved = {}
for name in ("samples.csv", "legs.csv", "missions.csv", "events.csv"):
    p = os.path.join(out, name)
    if not os.path.exists(p):
        continue
    with open(p, newline="") as f:
        rd = csv.DictReader(f)
        cols, rows = rd.fieldnames, list(rd)
    if "label" not in (cols or []):
        continue
    keep = [r for r in rows if r.get("label") != label]
    gone = [r for r in rows if r.get("label") == label]
    if not gone:
        continue
    os.makedirs(dest, exist_ok=True)
    with open(os.path.join(dest, name), "w", newline="") as f:
        wr = csv.DictWriter(f, cols)
        wr.writeheader()
        wr.writerows(gone)
    tmp = p + ".tmp"
    with open(tmp, "w", newline="") as f:
        wr = csv.DictWriter(f, cols)
        wr.writeheader()
        wr.writerows(keep)
    os.replace(tmp, p)
    moved[name] = len(gone)
if not moved:
    sys.exit(f"no rows with label {label}")
with open(os.path.join(dest, "WHY.txt"), "w") as f:
    f.write(why + "\n")
msg = (f"- {time.strftime('%H:%M')} VOID: label `{label}` set aside ({', '.join(f'{k} {v}' for k, v in moved.items())}) "
       f"-> {dest}. Reason: {why}")
with open(os.path.join(out, "LAB_NOTEBOOK.md"), "a") as f:
    f.write(msg + "\n")
print(msg)
