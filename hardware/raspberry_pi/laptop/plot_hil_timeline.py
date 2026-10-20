#!/usr/bin/env python3
"""Plot a hil_logger.py chain log and measure reaction times through the chain.

    python3 plot_hil_timeline.py ~/arc_hil_logs/chain_<time>.csv [--from 30 --to 200]

Figure (4 panels, shared time axis):
  1. webcam output: /person_moving, /person_confused (and shopper button presses)
  2. BT mode (/arc/behavior_mode) as coloured bands
  3. /cmd_vel: linear (m/s) and angular (rad/s)
  4. wheels: commanded to the Uno (dashed) vs measured by the Hall sensors (solid), m/s

Reaction times. "go" = moving OR confused (the BT is allowed to drive). For each go->stop and
stop->go change of the webcam output it finds, after that moment:
  bt_s     first BT mode message that differs from the mode at that moment
  cmd_s    stop: first /cmd_vel with |v| < 0.005 and |w| < 0.01;  go: first with |v| > 0.01
  wheel_s  stop: first measured sample with both wheels < --still m/s;  go: first above it
A go event may show no motion if the robot has no goal at that time (shown as "-").
Measured wheel speed comes from a few Hall pulses per turn, so wheel_s includes that lag.
Saves <log>_timeline.png and <log>_reactions.csv.
"""
import argparse
import csv
import statistics
import sys
from pathlib import Path

MODE_COLOURS = {"FOLLOW": "#3a9d3a", "LEAD": "#3a6fd8", "WAIT": "#f0a020", "IDLE": "#999999",
                "EMERGENCY": "#d83a3a", "STOP": "#d83a3a"}


def load(path):
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            r["t"] = float(r["t"])
            for k in ("a", "b", "c"):
                try:
                    r[k] = float(r[k])
                except (TypeError, ValueError):
                    r[k] = None
            rows.append(r)
    rows.sort(key=lambda r: r["t"])
    return rows


def series(rows, topic):
    return [r for r in rows if r["topic"] == topic]


def go_changes(rows):
    """[(t, 'stop'|'go')] where moving OR confused changes value (first value is not an event)."""
    moving = confused = None
    prev, out = None, []
    for r in rows:
        if r["topic"] == "/person_moving":
            moving = bool(r["a"])
        elif r["topic"] == "/person_confused":
            confused = bool(r["a"])
        else:
            continue
        if moving is None or confused is None:
            continue
        go = moving or confused
        if prev is not None and go != prev:
            out.append((r["t"], "go" if go else "stop"))
        prev = go
    return out


def mode_at(rows, t):
    m = None
    for r in rows:
        if r["t"] > t:
            break
        if r["topic"] == "/arc/behavior_mode":
            m = r["text"]
    return m


def first_after(rows, t, topic, pred, window):
    for r in rows:
        if r["t"] <= t or r["topic"] != topic:
            continue
        if r["t"] > t + window:
            return None
        if pred(r):
            return r["t"] - t
    return None


def reactions(rows, still=0.02, window=10.0):
    out = []
    for t, kind in go_changes(rows):
        m0 = mode_at(rows, t)
        bt = first_after(rows, t, "/arc/behavior_mode", lambda r: r["text"] != m0, window)
        if kind == "stop":
            cmd = first_after(rows, t, "/cmd_vel", lambda r: abs(r["a"] or 0) < 0.005 and abs(r["b"] or 0) < 0.01, window)
            wh = first_after(rows, t, "/arc_hw/wheels",
                             lambda r: abs(r["a"] or 0) < still and abs(r["b"] or 0) < still, window)
        else:
            cmd = first_after(rows, t, "/cmd_vel", lambda r: abs(r["a"] or 0) > 0.01, window)
            wh = first_after(rows, t, "/arc_hw/wheels",
                             lambda r: max(abs(r["a"] or 0), abs(r["b"] or 0)) > still, window)
        out.append({"t": round(t, 3), "event": kind, "mode_before": m0 or "",
                    "bt_s": bt, "cmd_s": cmd, "wheel_s": wh})
    return out


def fmt(v):
    return "-" if v is None else f"{v:.2f}"


def summary(reacts):
    lines = []
    for kind in ("stop", "go"):
        ev = [r for r in reacts if r["event"] == kind]
        if not ev:
            continue
        parts = []
        for k in ("bt_s", "cmd_s", "wheel_s"):
            v = [r[k] for r in ev if r[k] is not None]
            parts.append(f"{k} median {statistics.median(v):.2f} s (n={len(v)})" if v else f"{k} -")
        lines.append(f"{kind:>4} events: {len(ev)} | " + " | ".join(parts))
    return lines


def plot(rows, out_png, t_from=None, t_to=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = [r for r in rows if (t_from is None or r["t"] >= t_from) and (t_to is None or r["t"] <= t_to)]
    if not rows:
        raise SystemExit("no data in that time range")
    t_end = rows[-1]["t"]
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(11, 8.5),
                           gridspec_kw={"height_ratios": [1.0, 0.6, 1.2, 1.4]})
    # two lanes: moving low/high at 0/0.8, confused at 1.4/2.2; lane names on the y axis
    for topic, off, col in (("/person_moving", 0.0, "#3a9d3a"), ("/person_confused", 1.4, "#3a6fd8")):
        s = series(rows, topic)
        if s:
            ax[0].step([r["t"] for r in s], [off + 0.8 * (r["a"] or 0) for r in s], where="post", color=col, lw=1.4)
    for r in series(rows, "/arc/ui/command"):
        ax[0].axvline(r["t"], color="#777", ls=":", lw=0.8)
        ax[0].text(r["t"], 2.75, f" {r['text']}", fontsize=7, va="top", ha="left", color="#555")
    ax[0].set_yticks([0.4, 1.8], ["moving", "confused"])
    ax[0].set_ylim(-0.2, 2.8)
    ax[0].set_ylabel("webcam\nflags")

    modes = series(rows, "/arc/behavior_mode")
    seen = set()
    for i, r in enumerate(modes):
        t1 = modes[i + 1]["t"] if i + 1 < len(modes) else t_end
        name = r["text"]
        key = next((k for k in MODE_COLOURS if k in name.upper()), None)
        ax[1].axvspan(r["t"], t1, color=MODE_COLOURS.get(key, "#bbbbbb"), alpha=0.75,
                      label=name if name not in seen else None)
        seen.add(name)
    ax[1].set_yticks([])
    ax[1].set_ylabel("BT mode")
    if seen:
        ax[1].legend(loc="upper left", fontsize=8, ncol=len(seen), frameon=False)

    cv = series(rows, "/cmd_vel")
    if cv:
        ax[2].plot([r["t"] for r in cv], [r["a"] for r in cv], color="#222", lw=1.2, label="linear v (m/s)")
        ax2 = ax[2].twinx()
        ax2.plot([r["t"] for r in cv], [r["b"] for r in cv], color="#c05a00", lw=0.8, alpha=0.7, label="angular w (rad/s)")
        ax2.set_ylabel("w (rad/s)", color="#c05a00")
        ax[2].legend(loc="upper left", fontsize=8, frameon=False)
        ax2.legend(loc="upper right", fontsize=8, frameon=False)
    ax[2].set_ylabel("/cmd_vel\nv (m/s)")

    for topic, ls, lab in (("/arc_hw/cmd_wheels", "--", "commanded"), ("/arc_hw/wheels", "-", "measured")):
        s = series(rows, topic)
        if s:
            ax[3].plot([r["t"] for r in s], [r["a"] for r in s], ls=ls, color="#1f77b4", lw=1, label=f"left {lab}")
            ax[3].plot([r["t"] for r in s], [r["b"] for r in s], ls=ls, color="#d62728", lw=1, label=f"right {lab}")
    ax[3].set_ylabel("wheel speed\n(m/s)")
    ax[3].set_xlabel("time (s, laptop clock)")
    ax[3].legend(loc="upper left", fontsize=8, ncol=4, frameon=False)
    for t, kind in go_changes(rows):
        for a in ax:
            a.axvline(t, color="#d83a3a" if kind == "stop" else "#3a9d3a", lw=0.6, alpha=0.5)
    for a in ax:
        a.grid(alpha=0.25)
    fig.suptitle("ARC hardware-in-the-loop: webcam (Pi) -> BT -> /cmd_vel -> real wheels (Pi + Uno)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--from", dest="t_from", type=float)
    ap.add_argument("--to", dest="t_to", type=float)
    ap.add_argument("--still", type=float, default=0.02, help="wheel speed counted as stopped (m/s)")
    a = ap.parse_args(argv)
    rows = load(a.log)
    base = Path(a.log).with_suffix("")
    reacts = reactions(rows, a.still)
    with open(f"{base}_reactions.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["t", "event", "mode_before", "bt_s", "cmd_s", "wheel_s"])
        w.writeheader()
        w.writerows(reacts)
    print(f"{'t':>8} {'event':>5} {'mode before':<14} {'BT':>6} {'cmd_vel':>8} {'wheels':>7}  (s after the webcam flag change)")
    for r in reacts:
        print(f"{r['t']:8.2f} {r['event']:>5} {r['mode_before'][:14]:<14} {fmt(r['bt_s']):>6} {fmt(r['cmd_s']):>8} {fmt(r['wheel_s']):>7}")
    for line in summary(reacts):
        print(line)
    plot(rows, f"{base}_timeline.png", a.t_from, a.t_to)
    print(f"saved {base}_timeline.png and {base}_reactions.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
