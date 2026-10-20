#!/usr/bin/env python3
"""Append a short end-of-test summary for one label to LAB_NOTEBOOK.md."""
import csv
import math
import os
import sys
import time

out, label, test = sys.argv[1], sys.argv[2], sys.argv[3]


def rows(name):
    p = os.path.join(out, name)
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return [r for r in csv.DictReader(f) if r.get("label") == label]


def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


ms, ls, ss = rows("missions.csv"), rows("legs.csv"), rows("samples.csv")
ls = [l for l in ls if (num(l.get("leg")) or 0) >= 0]
ms = list({m["mission"]: m for m in ms}.values())          # re-run missions: keep the last attempt
ls = list({(l["mission"], l["leg"]): l for l in ls}.values())
false_arr = [l for l in ls if l.get("success") == "1" and (num(l.get("final_dist")) or 0) > 1.0]
for l in false_arr:                                          # reached, but ended > 1 m from the goal
    l["success"], l["reason"] = "0", "false_arrival"
    for m in ms:
        if m["mission"] == l["mission"]:
            m["completed"] = "0"
errs = sorted(e for e in (num(s.get("pos_err")) for s in ss) if e is not None)
med = errs[len(errs) // 2] if errs else None
fails = [f"{l['mission']} leg {l['leg']} ({l['reason']})" for l in ls if l.get("success") == "0"]
recov = sum(int(num(m.get("recoveries")) or 0) for m in ms)
coll = sum(int(num(m.get("collisions")) or 0) for m in ms)
by_msg = sum(1 for l in ls if l.get("reason") == "arrived")
lost = [e for e in (rows("events.csv")) if e.get("event") in ("localisation_lost", "sim_link_lost")]
lines = [f"- Finished {time.strftime('%H:%M')} ({test}, label `{label}`, cumulative for this label):",
         f"  - missions completed {sum(int(m['completed']) for m in ms)}/{len(ms)}; "
         f"legs reached {sum(int(l['success']) for l in ls)}/{len(ls)} "
         f"({by_msg} by /navigation_goal_reached)",
         f"  - recoveries (controller aborts + goal re-sends) {recov}; legs with clearance <= 0: {coll}",
         "  - leg failures: " + ((", ".join(fails))[:400] or "none"),
         f"  - median localisation error {med:.3f} m over {len(errs)} samples" if med is not None
         else "  - no true-pose samples (check the probe)"]
if false_arr:
    lines.append(f"  - false arrivals (reached message, robot > 1 m from goal) counted as failures: "
                 + ", ".join(f"{l['mission']} leg {l['leg']}" for l in false_arr))
if lost:
    lines.append(f"  - STOPPED (localisation or Gazebo link lost): {lost[-1]['mission']} leg {lost[-1]['leg']} - {lost[-1]['detail']}")
with open(os.path.join(out, "LAB_NOTEBOOK.md"), "a") as f:
    f.write("\n".join(lines) + "\n")
print("\n".join(lines))
