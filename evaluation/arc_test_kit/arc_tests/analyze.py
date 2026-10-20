#!/usr/bin/env python3
"""Turn the CSVs in a results folder into tables and figures.

    python3 arc_tests/analyze.py results [--compare teb_icp_pairs rpp_icp_pairs] [--compare teb_icp teb_wheel]

Writes results/report/results.md and PNG figures. Every number comes from the
CSVs; nothing is estimated. Labels with no rows are skipped, so the report can
be run at any point.
"""
import argparse
import csv
import math
import os
from collections import defaultdict

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:  # noqa: BLE001
    plt = None

try:
    from scipy.stats import wilcoxon
except Exception:  # noqa: BLE001
    wilcoxon = None

PASS_POS, PASS_YAW = 0.15, 5.0          # same limit as check_frames.py
FALSE_ARRIVAL_M = 1.0                   # a 'reached' leg that ended farther than this from its goal
LOSS_POS, LOSS_YAW = 0.5, 15.0          # a localisation-loss episode starts above either
A1_PERIOD = 2.0                          # plan: sample every 2 s


def read(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def stats(vals):
    v = np.array([x for x in vals if x is not None], dtype=float)
    if v.size == 0:
        return None
    return {"n": int(v.size), "mean": float(v.mean()), "median": float(np.median(v)),
            "p95": float(np.percentile(v, 95)), "max": float(v.max()), "min": float(v.min()),
            "sd": float(v.std(ddof=1)) if v.size > 1 else 0.0}


def f(x, d=2):
    return "-" if x is None else f"{x:.{d}f}"


def sget(st, key, d=2):
    return f(st[key], d) if st else "-"


def paired_test(a, b):
    """Wilcoxon signed-rank if scipy is present, else an exact two-sided sign test."""
    diffs = [x - y for x, y in zip(a, b) if x is not None and y is not None]
    nz = [d for d in diffs if abs(d) > 1e-9]
    if len(nz) < 5:
        return f"{len(diffs)} pairs, {len(nz)} non-zero differences (too few for a test)"
    if wilcoxon:
        p = wilcoxon(nz).pvalue
        return f"Wilcoxon signed-rank, n={len(nz)}, p={p:.3g}"
    k, n = sum(1 for d in nz if d > 0), len(nz)
    tail = sum(math.comb(n, i) for i in range(0, min(k, n - k) + 1)) / 2 ** n
    return f"sign test, n={n}, p={min(1.0, 2 * tail):.3g} (install scipy for Wilcoxon)"


def table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def thin(samples, period=A1_PERIOD):
    """One sample every `period` s of sim time per mission (the plan's A1 rate)."""
    out, last = [], {}
    for s in samples:
        t = fnum(s.get("sim_t"))
        if t is None:
            continue
        k = s["mission"]
        if k not in last or t - last[k] >= period - 1e-6 or t < last[k]:
            out.append(s)
            last[k] = t
    return out


def loss_episodes(samples):
    """Count entries into the loss state (pos > 0.5 m or yaw > 15 deg), per mission."""
    n, state = 0, {}
    for s in samples:
        p, y = fnum(s.get("pos_err")), fnum(s.get("yaw_err_deg"))
        if p is None or y is None:
            continue
        lost = p > LOSS_POS or y > LOSS_YAW
        if lost and not state.get(s["mission"]):
            n += 1
        state[s["mission"]] = lost
    return n


def a1_row(lab, ss):
    th = thin(ss)
    pe = [fnum(s["pos_err"]) for s in th]
    ye = [fnum(s["yaw_err_deg"]) for s in th]
    both = [(p, y) for p, y in zip(pe, ye) if p is not None and y is not None]
    st, sy = stats(pe), stats(ye)
    src = ",".join(sorted({s.get("truth_src") for s in ss if s.get("truth_src")})) or "-"
    if not st:
        return [lab, 0, len(ss), "no truth", "-", "-", "-", "-", "-", src]
    passed = 100 * sum(1 for p, y in both if p <= PASS_POS and y <= PASS_YAW) / len(both)
    return [lab, st["n"], len(ss), f(st["median"], 3), f(st["p95"], 3), f(st["max"], 3),
            sget(sy, "median"), f"{passed:.1f}%", loss_episodes(ss), src]


def leg_metric(legs, key):
    return [fnum(l.get(key)) for l in legs]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results")
    ap.add_argument("--compare", nargs=2, metavar=("LABEL_A", "LABEL_B"), action="append", default=[],
                    help="paired comparison of two configurations (A3 / B1); repeatable")
    a = ap.parse_args()
    R = a.results
    rep = os.path.join(R, "report")
    os.makedirs(rep, exist_ok=True)
    samples, legs, missions = read(f"{R}/samples.csv"), read(f"{R}/legs.csv"), read(f"{R}/missions.csv")
    legs = [l for l in legs if (fnum(l.get("leg")) or 0) >= 0]   # leg -1 = drive to the start
    # A mission re-run after a stop (--start-at) keeps only its last attempt in the leg/mission tables;
    # every sample still counts for A1.
    last = {}
    for m in missions:
        last[(m["label"], m["mission"])] = m
    missions = list(last.values())
    lastleg = {}
    for l in legs:
        lastleg[(l["label"], l["mission"], l["leg"])] = l
    legs = list(lastleg.values())
    # A leg logged as reached while the robot ended > 1 m from its goal is a false arrival (kits before
    # 2026-10-15.12 could take a stale re-plan of the previous goal and its "reached" message). Count it
    # as a failure and mark its mission not completed.
    false_arr = []
    for l in legs:
        fd = fnum(l.get("final_dist"))
        if int(l.get("success") or 0) and fd is not None and fd > FALSE_ARRIVAL_M:
            l["success"], l["reason"] = "0", "false_arrival"
            false_arr.append(l)
    for m in missions:
        k = sum(1 for l in false_arr if (l["label"], l["mission"]) == (m["label"], m["mission"]))
        if k:
            m["completed"] = "0"
            m["legs_ok"] = str(max(0, int(m.get("legs_ok") or 0) - k))
    events = read(f"{R}/events.csv")
    bench = read(f"{R}/ordering_benchmark.csv")
    md = ["# ARC simulation test results", "",
          f"Source: `{os.path.abspath(R)}`. Leg = one drive to one stop. Mission = one shopping list "
          "or one start-goal pair. Each label is one stack configuration.", ""]

    if false_arr:
        md += ["## Data check: false arrivals (counted as failures below)", "",
               table(["Config", "Mission", "Leg", "Planned path (m)", "Ended from goal (m)", "Leg time (s)"],
                     [[l["label"], l["mission"], l["leg"], l.get("planned_len", ""), l.get("final_dist", ""),
                       l.get("time_s", "")] for l in false_arr]), "",
               f"These legs were logged as reached, but the robot ended more than {FALSE_ARRIVAL_M} m from the goal "
               "(a stale re-plan of the previous goal was taken as this leg's path).", ""]
    else:
        md += [f"Data check: no false arrivals (every reached leg ended within {FALSE_ARRIVAL_M} m of its goal).", ""]

    # ---------------- A1 localisation
    if samples:
        md += ["## A1 Localisation accuracy (map pose vs Gazebo truth, every 2 s)", ""]
        by_label = defaultdict(list)
        for s in samples:
            by_label[s["label"]].append(s)
        rows = [a1_row(lab, ss) for lab, ss in sorted(by_label.items())]
        md += [table(["Config", "Samples (2 s)", "Samples (all)", "Median err (m)", "95th pct (m)", "Max (m)",
                      "Median yaw err (deg)", "Within 0.15 m / 5 deg", "Loss episodes", "Truth source"], rows), "",
               f"Loss episode = error rises above {LOSS_POS} m or {LOSS_YAW:.0f} deg (counted once until it recovers). "
               "Samples cover every drive, including returns to the start (leg -1).", ""]
        if plt:
            # Cumulative distribution on a shared log axis: overlaid histograms with per-label bins
            # (the old plot) made bar heights incomparable between configurations.
            fig, ax = plt.subplots(figsize=(7, 3.8))
            for lab, ss in sorted(by_label.items()):
                if lab == "smoke":
                    continue
                pe = sorted(max(fnum(s["pos_err"]), 1e-3) for s in thin(ss) if fnum(s["pos_err"]) is not None)
                if not pe:
                    continue
                n = len(pe)
                ax.step(pe, [(i + 1) / n * 100 for i in range(n)], where="post", lw=1.4,
                        label=f"{lab} (n={n}, median {pe[n // 2] * 100:.1f} cm)")
            ax.axvline(PASS_POS, ls="--", c="k", lw=1)
            ax.text(PASS_POS * 1.05, 3, f"{PASS_POS} m limit", fontsize=8)
            ax.set_xscale("log")
            ax.set_ylim(0, 101)
            ax.set_xlabel("Position error, map vs Gazebo truth (m, log scale)")
            ax.set_ylabel("Samples at or below this error (%)")
            ax.grid(alpha=0.3, which="both")
            ax.legend(fontsize=7, loc="upper left")
            fig.tight_layout(); fig.savefig(f"{rep}/A1_localisation_error.png", dpi=150); plt.close(fig)
            md += ["![A1](A1_localisation_error.png)", "",
                   "Read it as: at a given error on the x-axis, the curve gives the share of samples that were "
                   "at least that accurate. Curves further left and higher are better; a tail running to the right "
                   "is a localisation loss.", ""]

    # ---------------- A2 reliability per label / strategy
    if missions:
        md += ["## A2 Mission reliability", ""]
        groups = defaultdict(list)
        for m in missions:
            groups[(m["label"], m["strategy"])].append(m)
        rows = []
        for (lab, strat), ms in sorted(groups.items()):
            ls = [l for l in legs if l["label"] == lab and l["strategy"] == strat]
            ok = sum(int(l["success"]) for l in ls)
            first = sum(1 for l in ls if int(l["success"]) and not int(fnum(l.get("resends")) or 0))
            by_msg = sum(1 for l in ls if l["reason"] == "arrived")
            comp = sum(int(m["completed"]) for m in ms)
            tt = stats([fnum(m["time_total"]) for m in ms if int(m["completed"])])
            dr = stats([fnum(m["driven_total"]) for m in ms if int(m["completed"])])
            rec = sum(int(fnum(m.get("recoveries")) or 0) for m in ms)
            coll = sum(int(fnum(m.get("collisions")) or 0) for m in ms)
            reasons = defaultdict(int)
            for l in ls:
                if not int(l["success"]):
                    reasons[l["reason"]] += 1
            rows.append([lab, strat, f"{comp}/{len(ms)} ({100 * comp / len(ms):.0f}%)",
                         f"{ok}/{len(ls)}" if ls else "-", f"{first}/{len(ls)}" if ls else "-",
                         f"{by_msg}/{ok}" if ok else "-",
                         f"{sget(tt, 'mean', 1)} / {sget(tt, 'median', 1)}", sget(dr, "mean"),
                         f"{rec} ({rec / len(ms):.2f}/mission)", coll,
                         ", ".join(f"{k} {v}" for k, v in reasons.items()) or "-"])
        md += [table(["Config", "Strategy", "Missions completed", "Legs reached", "Legs first try",
                      "Arrivals by ARC signal", "Mission time mean / median (s)", "Driven (m, mean)",
                      "Recoveries", "Collisions", "Failures"], rows), "",
               "- Times and distances are over completed missions. Driven distance is from the Gazebo true pose.",
               "- Recoveries = FollowPath aborts + goal re-sends by the runner (ARC itself has no recovery "
               "behaviour; the Shopping Manager would wait forever).",
               "- Collisions = legs whose true pose came within the robot radius of a mapped obstacle cell or a "
               "test box (geometry at map resolution, not a contact sensor).", ""]

    # ---------------- A4 obstacles
    obs = [l for l in legs if (fnum(l.get("n_obstacles")) or 0) > 0]
    if obs:
        md += ["## A4 Reactive avoidance (boxes dropped on the path after planning)", ""]
        rows = []
        for lab in sorted({l["label"] for l in obs}):
            ol = [l for l in obs if l["label"] == lab]
            cl = stats(leg_metric(ol, "min_obstacle_clearance"))
            contacts = sum(1 for c in leg_metric(ol, "min_obstacle_clearance") if c is not None and c <= 0)
            succ = sum(int(l["success"]) for l in ol)
            rec = sum(int(fnum(l.get("ctrl_aborts")) or 0) + int(fnum(l.get("resends")) or 0) for l in ol)
            nb = sum(int(fnum(l["n_obstacles"])) for l in ol)
            rows.append([lab, len(ol), nb, f"{succ}/{len(ol)}", contacts, sget(cl, "median", 3),
                         sget(cl, "min", 3), rec])
        md += [table(["Config", "Runs with boxes", "Boxes", "Reached", "Contacts (clearance <= 0)",
                      "Min clearance median (m)", "Worst (m)", "Recoveries"], rows), "",
               "Clearance = true-pose distance from the robot edge (radius 0.105 m) to the box edge.", ""]

    # ---------------- comparisons between two configs (A3 / B1)
    labels_present = {m["label"] for m in missions}
    for A, B in a.compare:
        if A not in labels_present or B not in labels_present:
            continue
        mA = {(m["list_id"], m["strategy"]): m for m in missions if m["label"] == A}
        mB = {(m["list_id"], m["strategy"]): m for m in missions if m["label"] == B}
        keys = sorted(set(mA) & set(mB))
        pair_note = ""
        if not keys:
            # different strategies on the same lists (A2 typed order vs A2-final exact order): pair by list
            mA = {m["list_id"]: m for m in missions if m["label"] == A}
            mB = {m["list_id"]: m for m in missions if m["label"] == B}
            keys = sorted(set(mA) & set(mB))
            pair_note = " (paired by shopping list; the visiting order differs)"
        if not keys:
            continue
        md += [f"## Paired comparison: {A} vs {B} ({len(keys)} shared missions){pair_note}", ""]
        rows = []
        # Time and distance only where BOTH configurations completed the mission: a failed mission ends
        # early, so including it makes the less reliable configuration look faster and shorter.
        both = [k for k in keys if int(mA[k]["completed"]) and int(mB[k]["completed"])]
        for field, name in (("time_total", "Mission time (s)"), ("driven_total", "Driven distance (m)")):
            va = [fnum(mA[k][field]) for k in both]
            vb = [fnum(mB[k][field]) for k in both]
            rows.append([f"{name}, both completed (n={len(both)})", sget(stats(va), "mean"), sget(stats(vb), "mean"),
                         paired_test(va, vb) if len(both) >= 5 else "too few pairs"])
        ca = sum(int(mA[k]["completed"]) for k in keys); cb = sum(int(mB[k]["completed"]) for k in keys)
        da = sum(1 for k in keys if int(mA[k]["completed"]) and not int(mB[k]["completed"]))
        db = sum(1 for k in keys if int(mB[k]["completed"]) and not int(mA[k]["completed"]))
        nd = da + db
        p_sign = (min(1.0, 2 * sum(math.comb(nd, i) for i in range(0, min(da, db) + 1)) / 2 ** nd)
                  if nd else None)
        rows.append(["Missions completed", f"{ca}/{len(keys)}", f"{cb}/{len(keys)}",
                     f"sign test on {nd} discordant pairs ({da} vs {db}), p={p_sign:.3g}" if nd else "no discordant pairs"])
        la = sum(int(fnum(l["success"]) or 0) for l in legs if l["label"] == A and l["mission"] in {mA[k]["mission"] for k in keys})
        lb = sum(int(fnum(l["success"]) or 0) for l in legs if l["label"] == B and l["mission"] in {mB[k]["mission"] for k in keys})
        rows.append(["Stops reached", la, lb, "-"])
        ra = sum(int(fnum(mA[k].get("recoveries")) or 0) for k in keys)
        rb = sum(int(fnum(mB[k].get("recoveries")) or 0) for k in keys)
        rows.append(["Recoveries", ra, rb, "-"])

        def per_mission(label, mm, key, how=np.mean):
            out = []
            for k in keys:
                vals = [fnum(l.get(key)) for l in legs if l["label"] == label and l["mission"] == mm[k]["mission"]]
                vals = [v for v in vals if v is not None]
                out.append(float(how(vals)) if vals else None)
            return out
        wa, wb = per_mission(A, mA, "mean_abs_wdot"), per_mission(B, mB, "mean_abs_wdot")
        rows.append(["Smoothness: mean |angular accel| (rad/s^2)", sget(stats(wa), "mean", 3),
                     sget(stats(wb), "mean", 3), paired_test(wa, wb)])

        def flips_per_m(label, mm):
            out = []
            for k in keys:
                ls = [l for l in legs if l["label"] == label and l["mission"] == mm[k]["mission"]]
                d = sum(fnum(l["driven_len_true"]) or fnum(l["driven_len_map"]) or 0 for l in ls)
                out.append(sum(int(l["w_sign_changes"] or 0) for l in ls) / d if d > 0.1 else None)
            return out
        rows.append(["Steering reversals per metre (mean)", sget(stats(flips_per_m(A, mA)), "mean"),
                     sget(stats(flips_per_m(B, mB)), "mean"), "-"])
        sA = [s for s in samples if s["label"] == A and s["mission"] in {mA[k]["mission"] for k in keys}]
        sB = [s for s in samples if s["label"] == B and s["mission"] in {mB[k]["mission"] for k in keys}]
        eA, eB = stats([fnum(s["pos_err"]) for s in thin(sA)]), stats([fnum(s["pos_err"]) for s in thin(sB)])
        rows.append(["Localisation error median / max (m)", f"{sget(eA, 'median', 3)} / {sget(eA, 'max', 3)}",
                     f"{sget(eB, 'median', 3)} / {sget(eB, 'max', 3)}", "-"])
        rows.append(["Localisation-loss episodes", loss_episodes(sA), loss_episodes(sB), "-"])
        md += [table(["Metric", A, B, "Test"], rows), "",
               "Smoothness is computed from every /cmd_vel message (TwistStamped from the motion gate), "
               "time-differenced in sim time.", ""]

    # ---------------- C2 executed ordering
    for lab in sorted({m["label"] for m in missions}):
        ms = [m for m in missions if m["label"] == lab]
        strategies = sorted({m["strategy"] for m in ms} - {"pair"})
        if len(strategies) < 2:
            continue
        md += [f"## C2 Ordering strategies, planned vs driven ({lab})", ""]
        by = defaultdict(dict)
        for m in ms:
            by[m["list_id"]][m["strategy"]] = m
        rows, tests = [], []
        for s in strategies:
            sel = [d[s] for d in by.values() if s in d]
            J = stats([fnum(m["planned_cost_J"]) for m in sel])
            dr = stats([fnum(m["driven_total"]) for m in sel if int(m["completed"])])
            tt = stats([fnum(m["time_total"]) for m in sel if int(m["completed"])])
            ratio = stats([fnum(m["driven_total"]) / fnum(m["planned_cost_J"]) for m in sel
                           if int(m["completed"]) and fnum(m["planned_cost_J"]) and fnum(m["driven_total"])])
            comp = sum(int(m["completed"]) for m in sel)
            rows.append([s, f"{comp}/{len(sel)}", sget(J, "mean"), sget(dr, "mean"), sget(tt, "mean", 1),
                         sget(ratio, "median")])
        for base in ("as_entered", "exact", "nn_euclid"):
            if "nn_dijkstra" in strategies and base in strategies:
                pairs = [(d["nn_dijkstra"], d[base]) for d in by.values() if "nn_dijkstra" in d and base in d
                         and int(d["nn_dijkstra"]["completed"]) and int(d[base]["completed"])]
                for field, name in (("driven_total", "driven distance"), ("time_total", "time")):
                    t = paired_test([fnum(p[0][field]) for p in pairs], [fnum(p[1][field]) for p in pairs])
                    tests.append(f"- {name}, nn_dijkstra vs {base}: {t}")
        md += [table(["Strategy", "Completed", "Planned J (m, mean)", "Driven (m, mean)", "Time (s, mean)",
                      "Driven / planned (median)"], rows), ""]
        md += tests + ["", "Planned J is the kit's Dijkstra cost from home through the stops (no return). "
                       "Driven distance stops at the arrival point, so a ratio slightly below 1 is possible.", ""]

    # ---------------- C1 offline benchmark
    if bench:
        md += ["## C1 Ordering benchmark (offline, Dijkstra path cost)", ""]
        strat = ["as_entered", "nn_euclid", "nn_dijkstra"]
        rows = []
        ns = sorted({int(b["n_items"]) for b in bench})
        for n in ns:
            bs = [b for b in bench if int(b["n_items"]) == n and not int(b["unreachable"] or 0)]
            st_n = stats([fnum(b["n_stops"]) for b in bs])
            jo = stats([fnum(b["J_exact"]) for b in bs])
            row = [n, len(bs), sget(st_n, "mean", 1), sget(jo, "mean")]
            for s in strat:
                g = stats([fnum(b[f"gap_{s}_pct"]) for b in bs])
                opt = 100 * sum(1 for b in bs if (fnum(b[f"gap_{s}_pct"]) or 0) < 1e-6) / max(len(bs), 1)
                row.append(f"{f(g['mean'], 1)} / {f(g['p95'], 1)} / {opt:.0f}%" if g else "-")
            te = stats([fnum(b["t_exact_ms"]) for b in bs])
            row.append(sget(te, "max", 1))
            rows.append(row)
        md += [table(["Items", "Lists", "Driving stops (mean)", "Optimal J (m, mean)",
                      "As entered: gap mean / p95 / % optimal", "NN Euclidean", "NN Dijkstra", "Exact solve max (ms)"],
                     rows),
               "", "Gap = (route cost - optimum) / optimum, in %. Unreachable lists are excluded. "
               "Items on the same aisle spot count as one driving stop.", ""]
        if plt:
            fig, ax = plt.subplots(figsize=(7, 3.5))
            width = 0.25
            for k, s in enumerate(strat):
                means = []
                for n in ns:
                    st = stats([fnum(b[f"gap_{s}_pct"]) for b in bench if int(b["n_items"]) == n])
                    means.append(st["mean"] if st else 0.0)
                ax.bar(np.arange(len(ns)) + (k - 1) * width, means, width, label=s)
            ax.set_xticks(range(len(ns))); ax.set_xticklabels([str(n) for n in ns])
            ax.set_xlabel("Items on the list"); ax.set_ylabel("Mean gap to optimum (%)"); ax.legend()
            fig.tight_layout(); fig.savefig(f"{rep}/C1_ordering_gap.png", dpi=150); plt.close(fig)
            md += ["![C1](C1_ordering_gap.png)", ""]

    # ---------------- event counts
    if events:
        md += ["## Logged events", ""]
        cnt = defaultdict(lambda: defaultdict(int))
        for e in events:
            cnt[e["label"]][e["event"]] += 1
        names = sorted({e["event"] for e in events})
        md += [table(["Config"] + names, [[lab] + [cnt[lab].get(n, 0) for n in names] for lab in sorted(cnt)]), ""]

    modes = defaultdict(lambda: defaultdict(int))
    for s in samples:
        modes[s["label"]][s.get("bt_mode") or "unknown"] += 1
    mode_txt = "; ".join(f"{lab}: " + ", ".join(f"{m} {100 * c / sum(d.values()):.0f}%" for m, c in sorted(d.items()))
                         for lab, d in sorted(modes.items())) or "no samples"
    md += ["## Notes", "",
           "- Simulation only: TurtleBot3 Burger in the compact store; not the cart, not real motors.",
           f"- BT mode during logged samples (LEAD caps linear speed at 0.10 m/s, FOLLOW passes the controller "
           f"command): {mode_txt}.",
           "- The runner stands in for the Shopping Manager. ARC's shopping_manager.py visits items in the "
           "order entered; nn_dijkstra and exact orderings are the kit's implementations.",
           "- Report medians and percentiles, not single runs. Re-run with a different --seed before claiming a trend."]
    with open(f"{rep}/results.md", "w") as fh:
        fh.write("\n".join(md) + "\n")
    print(f"wrote {rep}/results.md")


if __name__ == "__main__":
    main()
