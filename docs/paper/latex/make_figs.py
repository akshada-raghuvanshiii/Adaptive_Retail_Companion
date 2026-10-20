#!/usr/bin/env python3
"""Figures for the ARC IEEE paper, drawn from the test logs (no invented values).

Sources: results.md of 16 Oct 09:20 (A1 table, C1 table), the C2 terminal log (c2_log.txt),
the B1 sample trace printed on 16 Oct 10:31 (b1_trace.txt).
Writes figs/*.pdf (LaTeX) and figs/*.png (doc).
"""
import os
import re
import statistics as st

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figs")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["DejaVu Serif"], "mathtext.fontset": "dejavuserif",
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8, "legend.fontsize": 7,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.linewidth": 0.6, "lines.linewidth": 1.1,
    "axes.spines.top": False, "axes.spines.right": False, "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})
COL = 3.45      # IEEE single column (in)
WIDE = 7.0      # IEEE double column (in)
INK, MID, LIGHT = "#1a1a1a", "#5b6b7f", "#a9b4c2"
C1, C2, C3 = "#1f4e79", "#c55a11", "#548235"   # distinguishable in grey too (with markers/hatches)


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + ".pdf"))
    fig.savefig(os.path.join(OUT, name + ".png"), dpi=300)
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 1 architecture
def fig_architecture():
    fig, ax = plt.subplots(figsize=(WIDE, 2.25))
    ax.set_xlim(-1, 101); ax.set_ylim(-1, 33); ax.axis("off")
    W, H, R1, R2 = 17, 8.5, 22, 4

    def box(x, y, title, sub, strong=False):
        ax.add_patch(FancyBboxPatch((x, y), W, H, boxstyle="round,pad=0.2,rounding_size=0.8",
                                    fc="#e8eef5" if strong else "white", ec=C1 if strong else MID,
                                    lw=1.1 if strong else 0.7))
        ax.text(x + W / 2, y + H * 0.64, title, ha="center", va="center", fontsize=5.8, weight="bold", color=INK)
        ax.text(x + W / 2, y + H * 0.28, sub, ha="center", va="center", fontsize=4.9, color=MID)

    def line(xs, ys, head=True, ls="-"):
        ax.plot(xs[:-1] + [xs[-1]] if not head else xs[:-1], ys[:-1] if head else ys, color=MID, lw=0.7, ls=ls)
        if head:
            ax.add_patch(FancyArrowPatch((xs[-2], ys[-2]), (xs[-1], ys[-1]), arrowstyle="-|>", mutation_scale=7,
                                         lw=0.7, color=MID, linestyle=ls, shrinkA=0, shrinkB=0))

    def lab(x, y, t, ha="center"):
        ax.text(x, y, t, ha=ha, va="center", fontsize=5.6, color=MID, style="italic")

    xs = [1, 21, 41, 61, 81]
    box(xs[0], R1, "Gazebo store", "2D lidar, wheel odometry")
    box(xs[1], R1, "ICP odometry", "lidar scan matching")
    box(xs[2], R1, "RTAB-Map", "locked map, localisation")
    box(xs[3], R1, "Dijkstra planner", "global path, 2 s replan", strong=True)
    box(xs[4], R1, "Path hand-over", "planned_path_to_teb")
    box(1, R2, "Shopping manager", "item order, y/n prompt", strong=True)
    box(24, R2, "Behaviour tree", "FOLLOW/WAIT/LEAD/E-STOP")
    box(52, R2, "Motion gate", "speed cap by BT mode")
    box(81, R2, "TEB-like controller", "Nav2 controller server", strong=True)
    ym = R1 + H / 2
    for i in range(4):
        line([xs[i] + W, xs[i + 1]], [ym, ym])
    # hand-over -> controller
    line([89.5, 89.5], [R1, R2 + H]); lab(91.2, 17.5, "FollowPath", ha="left")
    # controller -> gate, BT -> gate
    line([81, 69], [R2 + H / 2, R2 + H / 2]); line([41, 52], [R2 + H / 2, R2 + H / 2]); lab(46.5, R2 + H / 2 + 1.6, "mode")
    # shopping manager -> planner: /goal_pose
    line([12, 12, 66, 66], [R2 + H, 18.5, 18.5, R1]); lab(38, 19.9, "/goal_pose")
    # hand-over -> shopping manager: goal reached (dashed)
    line([98, 100, 100, 6, 6], [ym, ym, 16, 16, R2 + H], ls="--"); lab(74, 14.6, "/navigation_goal_reached")
    # gate -> robot: /cmd_vel
    line([60.5, 60.5, -0.5, -0.5, 1], [R2, 0.6, 0.6, ym, ym]); lab(30, 1.9, "/cmd_vel (TwistStamped) to the robot")
    lab(10, 31.6, "/scan, /odom", ha="left"); lab(30, 31.6, "/odom_icp", ha="left")
    lab(50, 31.6, "map pose (TF), map", ha="left"); lab(70, 31.6, "/planned_path", ha="left")
    for x in (19, 39, 59, 79):
        ax.plot([x, x], [ym + 0.2, 30.8], color="#d5dbe3", lw=0.5, ls=":")
    save(fig, "fig1_architecture")


# ---------------------------------------------------------------- Fig. 2 localisation
def fig_localisation():
    # results.md, 16 Oct 09:20 (A1 table): median, 95th percentile, max position error (m)
    # updated from results.md of 17 Oct 19:07 (phase 2 complete; A4 all 20 pairs)
    rows = [("A2 lists, TEB", 0.029, 0.059, 0.117, 2057),
            ("A2-pc", 0.025, 0.096, 1.149, 2296),
            ("A2-final", 0.035, 0.080, 0.140, 1766),
            ("A3 pairs, TEB", 0.020, 0.043, 0.076, 577),
            ("A3 pairs, RPP", 0.018, 0.039, 0.058, 421),
            ("C2 ordering, TEB", 0.029, 0.117, 0.213, 3420),
            ("A4 boxes, TEB", 0.038, 0.128, 2.170, 785)]
    fig, ax = plt.subplots(figsize=(COL * 0.62, 2.15))   # labels on the left widen the saved figure to ~1 column
    for i, (name, med, p95, mx, n) in enumerate(rows):
        y = len(rows) - 1 - i
        ax.plot([med, p95], [y, y], color=C1, lw=2.2, solid_capstyle="butt")
        ax.plot(med, y, "o", color=C1, ms=4.2, zorder=3)
        ax.plot(mx, y, "x", color=C2, ms=4.5, mew=1.1, zorder=3)
    ax.axvline(0.15, color=MID, lw=0.7, ls="--")
    ax.text(0.16, -0.45, "0.15 m", fontsize=6.2, color=MID)
    ax.set_xscale("log"); ax.set_xlim(0.01, 3.0); ax.set_ylim(-0.6, len(rows) - 0.4)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([f"{r[0]} (n={r[4]})" for r in reversed(rows)], fontsize=6.6)
    ax.tick_params(axis="y", length=0); ax.spines["left"].set_visible(False)
    ax.set_xlabel("Position error vs. truth (m, log)")
    ax.plot([], [], "o-", color=C1, lw=2.2, ms=4, label="median to 95th percentile")
    ax.plot([], [], "x", color=C2, ms=4.5, label="maximum")
    ax.legend(loc="lower right", frameon=False, ncol=2, bbox_to_anchor=(1.02, 1.0), handlelength=1.6,
              columnspacing=0.8)
    save(fig, "fig2_localisation")


# ---------------------------------------------------------------- Fig. 3 C1 offline ordering
def fig_c1():
    # results.md C1 table: mean gap to the exact optimum (%), 50 lists per size
    n = [3, 5, 8, 10, 12]
    typed = [22.9, 43.4, 72.6, 81.2, 93.8]
    nne = [3.5, 7.4, 7.1, 8.8, 6.9]
    nnd = [0.6, 4.5, 4.4, 7.3, 6.2]
    fig, ax = plt.subplots(figsize=(COL, 1.7))
    ax.plot(n, typed, "s-", color=C2, ms=3.5, label="typed order")
    ax.plot(n, nne, "^--", color=MID, ms=3.5, label="nearest-next, straight line")
    ax.plot(n, nnd, "o-", color=C1, ms=3.5, label="nearest-next, path cost")
    ax.set_xticks(n); ax.set_xlabel("Items on the shopping list"); ax.set_ylabel("Route longer than optimum (%)")
    ax.set_ylim(0, 100); ax.grid(axis="y", color="#e3e7ec", lw=0.5)
    ax.legend(frameon=False, loc="upper left")
    save(fig, "fig3_c1_ordering")


# ---------------------------------------------------------------- C2 log parsing
def parse_c2(path):
    ms, cur = [], None
    for line in open(path):
        m = re.match(r"\[(\d+)/90\] (s21_n(\d+)_(\d+)) (\w+): (\d+) stop\(s\), planned J=([\d.]+)", line)
        if m:
            cur = dict(lst=m[2], n=int(m[3]), strat=m[5], J=float(m[7]), legs=[])
            ms.append(cur)
            continue
        m = re.match(r"\s+(OK|XX) leg (-?\d+) -> (\S+): ([\d.]+) s, path ([\d.]+) m", line)
        if m and cur and int(m[2]) >= 0:
            cur["legs"].append(dict(ok=m[1] == "OK", t=float(m[4]), path=float(m[5])))
    return ms


def fig_c2():
    ms = parse_c2(os.path.join(HERE, "c2_log.txt"))
    assert len(ms) == 90, len(ms)
    sizes, strats = [5, 8, 12], [("as_entered", "typed order", C2, "//"),
                                 ("nn_dijkstra", "nearest-next", MID, ".."),
                                 ("exact", "exact (Held-Karp)", C1, "")]
    fig, ax = plt.subplots(figsize=(COL, 1.75))
    w = 0.26
    for k, (s, lab, c, hatch) in enumerate(strats):
        meds = [st.median(sum(x["t"] for x in m["legs"]) for m in ms if m["n"] == n and m["strat"] == s)
                for n in sizes]
        xs = [i + (k - 1) * w for i in range(len(sizes))]
        ax.bar(xs, meds, w * 0.92, color=c if not hatch else "white", edgecolor=c, hatch=hatch, lw=0.8, label=lab)
        for x, v in zip(xs, meds):
            ax.text(x, v + 6, f"{v:.0f}", ha="center", va="bottom", fontsize=5.8, color=INK)
    ax.set_xticks(range(len(sizes))); ax.set_xticklabels([f"{n} items" for n in sizes])
    ax.set_ylabel("Mission time, median (s)"); ax.set_ylim(0, 470)
    ax.grid(axis="y", color="#e3e7ec", lw=0.5); ax.set_axisbelow(True)
    ax.legend(frameon=False, loc="lower left", bbox_to_anchor=(0.0, 1.0), ncol=3, columnspacing=0.8,
              handlelength=1.4)
    save(fig, "fig4_c2_time")
    return ms


# ---------------------------------------------------------------- Fig. 5 B1 wheel-only trace
def fig_b1():
    t, me, we, cx, mis = [], [], [], [], []
    for line in open(os.path.join(HERE, "b1_trace.txt")):
        p = line.split()
        if len(p) == 8 and p[0] in ("002", "003") and p[3] != "-":
            try:
                t.append(float(p[2])); me.append(float(p[3])); we.append(float(p[4])); cx.append(float(p[5]))
                mis.append(p[0])
            except ValueError:
                pass
    freeze = next(tt for tt, c, cn in zip(t, cx, cx[1:] + [None]) if abs(c - 1.2478) < 1e-4)
    fig, ax = plt.subplots(figsize=(COL, 1.8))
    ax.plot(t, we, "-", color=MID, lw=0.9, label="wheel odometry error")
    ax.plot(t, me, "-", color=C1, lw=1.4, label="map pose error (localisation)")
    ax.axvspan(freeze, max(t), color="#f3e3d3", lw=0)
    ax.axhline(1.0, color=C2, lw=0.7, ls="--")
    ax.text(freeze + 2, 2.2, "RTAB-Map correction\nfrozen", fontsize=6.2, color=C2, va="top")
    ax.text(1305, 0.96, "1.0 m loss threshold", fontsize=6.0, color=C2, va="top")
    ax.set_xlabel("Simulation time (s)"); ax.set_ylabel("Error vs. truth (m)")
    ax.set_xlim(min(t), max(t)); ax.set_ylim(0, 2.4)
    ax.legend(frameon=False, loc="upper left", bbox_to_anchor=(0.0, 1.22), ncol=2, columnspacing=0.8)
    save(fig, "fig5_b1_wheel")
    return freeze


# ---------------------------------------------------------------- Phase 2 (A2, A2-pc, A2-final)
P2 = [("teb_icp", "A2", "#2a78d6", "o"), ("teb_icp_pc", "A2-pc", "#eb6834", "s"),
      ("teb_icp_pc_final", "A2-final", "#1baf7a", "^")]   # validated palette (dataviz check), + markers


def load_phase2():
    import csv
    d = {k: {} for k, *_ in P2}
    for r in csv.DictReader(open(os.path.join(HERE, "phase2_missions.csv"))):
        d[r["label"]][r["list_id"]] = r
    return d


def fig_phase2():
    """Three panels: first-attempt arrivals, recoveries per mission, time per completed list.
    Counts are the report's (results.md 17 Oct 19:07); times are per list from phase2_missions.csv."""
    d = load_phase2()
    first = {"teb_icp": (96, 144), "teb_icp_pc": (136, 141), "teb_icp_pc_final": (133, 144)}
    recov = {"teb_icp": 97, "teb_icp_pc": 4, "teb_icp_pc_final": 13}
    aborts = {"teb_icp": 62, "teb_icp_pc": 0, "teb_icp_pc_final": 2}
    done = {k: sum(int(r["completed"]) for r in d[k].values()) for k, *_ in P2}
    fig, axes = plt.subplots(1, 3, figsize=(WIDE, 1.85), gridspec_kw={"width_ratios": [1, 1, 1.5]})
    names = [n for _, n, _, _ in P2]
    x = range(len(P2))
    ax = axes[0]
    for i, (k, n, c, m) in enumerate(P2):
        ok, tot = first[k]
        ax.bar(i, 100 * ok / tot, width=0.62, color=c, edgecolor="white", linewidth=1.5)
        ax.text(i, 100 * ok / tot + 2, f"{100 * ok / tot:.0f}%\n({ok}/{tot})", ha="center", va="bottom",
                fontsize=6.2, color=INK)
        ax.text(i, 4, f"{done[k]}/30\nlists", ha="center", va="bottom", fontsize=5.8, color="white", weight="bold")
    ax.set_xticks(list(x)); ax.set_xticklabels(names); ax.set_ylim(0, 125); ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_ylabel("Stops reached at first attempt (%)"); ax.set_title("(a) First-attempt arrivals", loc="left")
    ax.grid(axis="y", color="#e3e7ec", lw=0.5); ax.set_axisbelow(True)
    ax = axes[1]
    for i, (k, n, c, m) in enumerate(P2):
        v = recov[k] / 30
        ax.bar(i, v, width=0.62, color=c, edgecolor="white", linewidth=1.5)
        ax.text(i, v + 0.08, f"{recov[k]}\n({aborts[k]} aborts)", ha="center", va="bottom", fontsize=6.2, color=INK)
    ax.set_xticks(list(x)); ax.set_xticklabels(names); ax.set_ylim(0, 4.3)
    ax.set_ylabel("Recoveries per mission"); ax.set_title("(b) Recoveries", loc="left")
    ax.grid(axis="y", color="#e3e7ec", lw=0.5); ax.set_axisbelow(True)
    ax = axes[2]
    import random
    rnd = random.Random(3)
    for i, (k, n, c, m) in enumerate(P2):
        ts = [float(r["time_total"]) for r in d[k].values() if r["completed"] == "1"]
        xs = [i + rnd.uniform(-0.17, 0.17) for _ in ts]
        ax.scatter(xs, ts, s=11, marker=m, facecolor=c, edgecolor="white", linewidth=0.5, alpha=0.9, zorder=2)
        med = st.median(ts)
        ax.plot([i - 0.3, i + 0.3], [med, med], color=INK, lw=1.4, zorder=3)
        ax.text(i + 0.33, med, f"{med:.0f} s\nmedian", va="center", fontsize=6.0, color=INK)
    ax.set_xticks(list(x)); ax.set_xticklabels(names); ax.set_xlim(-0.5, 2.75)
    ax.set_ylabel("Mission time (s)"); ax.set_title("(c) Time per completed list", loc="left")
    ax.grid(axis="y", color="#e3e7ec", lw=0.5); ax.set_axisbelow(True)
    fig.tight_layout(w_pad=1.2)
    save(fig, "fig6_phase2")


def fig_phase2_paired():
    """Per-list paired times: A2 (x) vs each variant (y), lists both completed; below the diagonal = faster."""
    d = load_phase2()
    fig, ax = plt.subplots(figsize=(COL, 2.75))
    lim = [60, 560]
    ax.plot(lim, lim, color=MID, lw=0.7, ls="--")
    ax.text(415, 395, "same time", fontsize=6.0, color=MID, rotation=45, ha="center", va="top")
    for k, n, c, m in P2[1:]:
        pts = [(float(d["teb_icp"][lid]["time_total"]), float(r["time_total"]))
               for lid, r in d[k].items()
               if r["completed"] == "1" and lid in d["teb_icp"] and d["teb_icp"][lid]["completed"] == "1"]
        faster = sum(y < x for x, y in pts)
        change = st.median([(y - x) / x * 100 for x, y in pts])
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=16, marker=m, facecolor=c, edgecolor="white",
                   linewidth=0.5, zorder=3,
                   label=f"{n}: faster on {faster}/{len(pts)} lists, median time "
                         f"{'+' if change >= 0 else chr(0x2212)}{abs(change):.0f}%")
    ax.set_xlim(lim); ax.set_ylim(lim); ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("A2 mission time (s)"); ax.set_ylabel("Variant mission time (s)")
    ax.grid(color="#e3e7ec", lw=0.5); ax.set_axisbelow(True)
    ax.legend(frameon=False, loc="lower left", bbox_to_anchor=(-0.02, 1.0), fontsize=6.1, handletextpad=0.3,
              borderaxespad=0.1, title="points below the line were faster than A2", title_fontsize=6.1)
    save(fig, "fig7_phase2_paired")


# ---------------------------------------------------------------- Fig. 8 HIL bench timeline
HIL_CSV = os.path.join(HERE, "hil_20261016_181250.csv")   # arc_hw_bridge log, 1 sample per ENC report (1 s)


def load_hil():
    import csv
    rows = list(csv.DictReader(open(HIL_CSV)))
    for r in rows:
        for k in r:
            if k != "bt_mode":
                r[k] = float(r[k])
    return rows


def fig_hil(t0=355.0, t1=840.0):
    from matplotlib.patches import Patch
    R = [r for r in load_hil() if t0 <= r["t"] <= t1]
    t = [r["t"] - t0 for r in R]
    fig, axs = plt.subplots(3, 1, figsize=(COL, 2.55), sharex=True,
                            gridspec_kw={"hspace": 0.22, "height_ratios": [0.32, 1, 1]})
    shade = {"LEAD": "#2a78d6", "FOLLOW": "#1baf7a"}
    runs = []                       # each sample covers the second before it
    for r in R:
        m = r["bt_mode"]
        if runs and runs[-1][0] == m:
            runs[-1][2] = r["t"] - t0
        else:
            runs.append([m, r["t"] - t0 - 1.0, r["t"] - t0])
    for i, ax in enumerate(axs):
        for m, a0, b0 in runs:
            if m in shade:
                ax.axvspan(a0, b0, color=shade[m], alpha=0.85 if i == 0 else 0.10, lw=0)
    axs[0].set_yticks([])
    axs[0].spines["left"].set_visible(False)
    axs[0].set_ylabel("BT", rotation=0, ha="right", va="center")
    axs[0].legend(handles=[Patch(color=shade["LEAD"], label="LEAD"), Patch(color=shade["FOLLOW"], label="FOLLOW"),
                           Patch(facecolor="white", edgecolor=LIGHT, label="WAIT")],
                  loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=3, frameon=False, handlelength=1.2)
    kw = dict(where="pre", lw=0.9)
    axs[1].step(t, [r["pwm_l"] for r in R], color="#eb6834", label="left", **kw)
    axs[1].step(t, [r["pwm_r"] for r in R], color=INK, label="right", **kw)
    axs[1].set_ylabel("PWM sent\n(limit \u00b160)")
    axs[1].set_ylim(-32, 66)
    axs[1].legend(loc="upper left", bbox_to_anchor=(0.20, 1.0), ncol=1, frameon=False, handlelength=1.4,
                  fontsize=6)
    axs[2].step(t, [r["target_r_mm_s"] for r in R], color="#2a78d6", lw=0.8, where="pre", label="target")
    axs[2].step(t, [r["meas_r_mm_s"] for r in R], color=INK, label="measured", **kw)
    axs[2].legend(loc="upper left", bbox_to_anchor=(0.20, 1.04), ncol=2, frameon=False, handlelength=1.4,
                  fontsize=6)
    axs[2].set_ylabel("right wheel\n(mm/s)")
    axs[2].set_ylim(-70, 225)
    axs[2].set_xlabel("time (s)")
    axs[2].set_xlim(0, t1 - t0)
    for ax in axs[1:]:
        ax.axhline(0, color=LIGHT, lw=0.5, zorder=0)
    for ax in axs:
        ax.yaxis.set_label_coords(-0.12, 0.5)
    save(fig, "fig8_hil_timeline")

if __name__ == "__main__":
    fig_architecture()
    fig_localisation()
    fig_c1()
    ms = fig_c2()
    fr = fig_b1()
    fig_phase2()
    fig_phase2_paired()
    fig_hil()
    print("figures written to", OUT, "| B1 freeze starts at sim t =", fr)
