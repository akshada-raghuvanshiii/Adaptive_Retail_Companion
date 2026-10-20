#!/usr/bin/env python3
"""Plot a hardware-in-the-loop log: commanded vs measured wheel speed, shaded by BT mode.

    python3 plot_hil.py hil_logs/hil_<time>.csv      -> hil_<time>.png and a short summary
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {"FOLLOW": "#cfe8d6", "LEAD": "#d6e4f5", "WAIT": "#fbecc8", "EMERGENCY": "#f6cfcf"}


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def main(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        sys.exit("empty log")
    t = [fnum(r["t"]) for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)
    # BT mode bands
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]["bt_mode"] != rows[start]["bt_mode"]:
            mode = rows[start]["bt_mode"] or "?"
            for ax in axes:
                ax.axvspan(t[start], t[i - 1], color=COLORS.get(mode.upper(), "#eeeeee"), lw=0)
            axes[0].text((t[start] + t[i - 1]) / 2, 1.0, mode, transform=axes[0].get_xaxis_transform(),
                         ha="center", va="bottom", fontsize=8)
            start = i
    for ax, side in zip(axes, ("l", "r")):
        ax.plot(t, [fnum(r[f"target_{side}_mm_s"]) for r in rows], label="commanded", lw=1.5)
        ax.plot(t, [fnum(r[f"meas_{side}_mm_s"]) for r in rows], label="measured (Hall)", lw=1.0)
        ax.set_ylabel(f"{'Left' if side == 'l' else 'Right'} wheel (mm/s)")
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[1].set_xlabel("Time (s)")
    fig.suptitle("ARC hardware-in-the-loop: /cmd_vel from the ARC stack vs real wheel speed", fontsize=10)
    fig.tight_layout()
    out = path.replace(".csv", ".png")
    fig.savefig(out, dpi=150)
    # summary: tracking error while commanded speed is steady and non-zero
    errs = []
    for side in ("l", "r"):
        for r in rows:
            c, m = fnum(r[f"target_{side}_mm_s"]), fnum(r[f"meas_{side}_mm_s"])
            if c is not None and m is not None and abs(c) > 20:
                errs.append(abs(abs(m) - abs(c)))
    modes = sorted({r["bt_mode"] for r in rows if r["bt_mode"]})
    print(f"saved {out}")
    print(f"{len(rows)} reports over {t[-1] - t[0]:.0f} s; BT modes seen: {', '.join(modes) or 'none'}")
    if errs:
        errs.sort()
        print(f"wheel speed error while moving: median {errs[len(errs) // 2]:.1f} mm/s, "
              f"95th pct {errs[int(0.95 * (len(errs) - 1))]:.1f} mm/s (n={len(errs)})")


if __name__ == "__main__":
    main(sys.argv[1])
