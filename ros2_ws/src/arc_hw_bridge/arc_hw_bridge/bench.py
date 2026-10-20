#!/usr/bin/env python3
"""Bench tests for the ARC cart Uno sketch - no ROS needed. WHEELS OFF THE GROUND, E-stop in reach.

    ros2 run arc_hw_bridge motor_bench check                 # sketch answers? E-stop closed? sensors quiet?
    ros2 run arc_hw_bridge motor_bench monitor               # spin each wheel by hand: its count must rise
    ros2 run arc_hw_bridge motor_bench watchdog --pwm 40     # motors run 3 s, then commands stop: must stop
    ros2 run arc_hw_bridge motor_bench calibrate             # PWM sweep -> per-wheel values for arc_hw.yaml
    ros2 run arc_hw_bridge motor_bench step                  # speed steps with the calibration in the YAML

Port, wheel diameter, pulses per wheel turn and the calibration are read from
~/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml; --port / --wheel / --ppr override them.
pulses per WHEEL turn = encoder pulses per motor turn x gear ratio (count one wheel turn with monitor).
Every test writes a CSV in ~/arc_hw_logs/bench/. Measured speed is a 1 s average: the sketch reports
pulses once per second (TELEMETRY_MS = 1000).
"""
import argparse
import csv
import os
import sys
import time

from . import protocol as P
from .link import Link

RATE = 20.0          # M commands per second while driving (the sketch's watchdog needs >= 5 Hz)


class Bench:
    def __init__(self, a):
        self.a = a
        self.geo = P.Geometry(a.wheel, 0.4, a.ppr)
        self.ffw = a.ffw
        self.link = Link(a.port, a.baud, log=lambda t, warn=False: print(("WARN " if warn else "") + t))
        if not self.link.ensure():
            sys.exit(f"cannot open {a.port}")
        self.rows = []

    def run_for(self, seconds, pl, pr, on_enc=None):
        """Send M,pl,pr at RATE for `seconds` (None pl = send nothing); call on_enc(el, er) per ENC report."""
        end = time.time() + seconds
        while time.time() < end:
            if pl is not None:
                self.link.write(P.cmd_motor(pl, pr))
            for m in self.link.read():
                if m[0] == "enc" and on_enc:
                    on_enc(m[1], m[2])
            if not self.link.ok():
                sys.exit("serial link lost")
            time.sleep(1.0 / RATE)

    def speed(self, pulses, sign=1):
        return P.signed_speed(pulses, sign, self.geo.mm_per_pulse)

    def stop(self):
        self.link.write(P.CMD_STOP)
        time.sleep(0.2)
        self.link.read()

    def save(self, name, header):
        d = os.path.expanduser("~/arc_hw_logs/bench")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.rows)
        print(f"saved {path}")
        return path


def t_check(b):
    print("listening 3.5 s with motors at 0 ...")
    enc = []
    b.run_for(3.5, 0, 0, on_enc=lambda l, r: enc.append((l, r)))
    L = b.link
    ok = True
    if L.counts["ready"]:
        print("[ OK ] sketch answered PING with ARC_MOTOR_READY")
    elif L.counts["ok"] or L.counts["err"]:
        print("[ OK ] sketch answers commands (no ARC_MOTOR_READY seen, PING may have been lost in the reset)")
    else:
        ok = False
        print(f"[FAIL] no reply in this sketch's protocol. Unknown lines: {L.counts['unknown']} "
              f"(e.g. '{L.sample_unknown}'). Wrong sketch, or wrong baud.")
    if L.estop_open:
        ok = False
        print("[FAIL] ERR,ESTOP: the E-stop loop on D4 is open, so every motor command is refused.")
        print("       D4 must reach GND through the closed (NC) E-stop. Without an E-stop wired, D4 floats HIGH.")
    elif L.estop_open is False:
        print("[ OK ] E-stop loop closed: motor commands accepted (OK)")
    if not enc:
        ok = False
        print("[FAIL] no ENC report in 3.5 s (the sketch sends one per second)")
    else:
        for side, i in (("left", 0), ("right", 1)):
            worst = max(e[i] for e in enc)
            if worst > 5:
                ok = False
                print(f"[WARN] {side} encoder counts {worst} pulses/s with the motors stopped: electrical noise or a loose "
                      "encoder connector (VCC, GND, channel A). Check that lead, then run check again.")
            else:
                print(f"[ OK ] {side} encoder quiet at rest ({worst} pulses/s)")
    other = {k: v for k, v in L.errors.items() if k != "ESTOP"}
    if other:
        print(f"[WARN] other errors from the sketch: {other}")
    print("CHECK PASS" if ok else "CHECK: fix the lines above first")
    return ok


def t_monitor(b):
    print("Spin each wheel slowly by hand. One line per second. Ctrl+C to end.")
    tot = [0, 0]

    def show(l, r):
        tot[0] += l
        tot[1] += r
        b.rows.append([round(time.time(), 2), l, r])
        print(f"  this second: left {l:4d}  right {r:4d}   total: left {tot[0]:6d}  right {tot[1]:6d}")
    try:
        b.run_for(1e9, 0, 0, on_enc=show)
    except KeyboardInterrupt:
        pass
    b.save("monitor", ["t", "enc_l", "enc_r"])


def t_watchdog(b):
    pwm = b.a.pwm
    print(f"driving both wheels at PWM {pwm} for 3 s ...")
    enc = []
    b.run_for(3.0, pwm, pwm, on_enc=lambda l, r: enc.append(("drive", l, r)))
    print("now sending NOTHING (the sketch must stop after 300 ms) ...")
    t_silent = time.time()
    b.run_for(2.6, None, None, on_enc=lambda l, r: enc.append((f"silent+{time.time() - t_silent:.1f}s", l, r)))
    b.stop()
    for e in enc:
        b.rows.append(list(e))
        print(f"  {e[0]:>12}: left {e[1]:4d}  right {e[2]:4d} pulses/s")
    drive = [e for e in enc if e[0] == "drive"]
    silent = [e for e in enc if e[0] != "drive"]
    moved = drive and max(max(e[1], e[2]) for e in drive) > 0
    # the first ENC after going silent still covers up to ~0.7 s of driving; the second one must be ~0
    last = silent[-1] if len(silent) >= 2 else None
    b.save("watchdog", ["phase", "enc_l", "enc_r"])
    if not moved:
        print("INCONCLUSIVE: the wheels did not turn while driving (E-stop? PWM too low? run check / calibrate)")
        return False
    if last and max(last[1], last[2]) <= 2:
        print("PASS: the wheels stopped without a STOP command (sketch watchdog works). Resolution is 1 s, "
              "because the sketch reports once per second.")
        return True
    print("FAIL: pulses kept coming after the commands stopped. Do not run the bridge until this passes.")
    return False


def t_calibrate(b):
    top = min(b.a.max_pwm, P.MAX_PWM_SKETCH)
    steps = list(range(0, top + 1, b.a.step_pwm))
    hold = b.a.hold or 5.0
    print(f"PWM sweep {steps} on both wheels, {hold:.0f} s each (measured: mean of the ENC reports after the first)")
    print(f"Using {b.geo.pulses_per_rev} pulses per wheel turn and a {b.geo.wheel_diameter_m} m wheel "
          f"({b.geo.mm_per_pulse:.2f} mm per pulse). Wrong values here scale every mm/s.")
    pts = {0: [], 1: []}
    for pwm in steps:
        got = []
        b.run_for(hold, pwm, pwm, on_enc=lambda l, r: got.append((l, r)))
        use = got[1:] or got               # the first report includes spin-up or the previous level
        if not use:
            print(f"  PWM {pwm:3d}: no ENC report"); continue
        l = sum(e[0] for e in use) / len(use)
        r = sum(e[1] for e in use) / len(use)
        sl, sr = b.speed(l), b.speed(r)
        pts[0].append((pwm, sl)); pts[1].append((pwm, sr))
        b.rows.append([pwm, round(l, 2), round(r, 2), round(sl, 1), round(sr, 1)])
        print(f"  PWM {pwm:3d}: left {l:6.1f} pulses/s = {sl:6.1f} mm/s   right {r:6.1f} pulses/s = {sr:6.1f} mm/s")
        if b.link.estop_open:
            print("ERR,ESTOP: E-stop open, stopping the sweep"); break
    b.stop()
    b.save("calibrate", ["pwm", "enc_l", "enc_r", "meas_l_mm_s", "meas_r_mm_s"])
    fits = {}
    for i, side in ((0, "left"), (1, "right")):
        f = P.fit_wheel(pts[i])
        fits[side] = f
        if f is None:
            print(f"  {side}: did not move enough (or not steadily enough) to fit. Check that wheel turns freely.")
            continue
        top_s = P.max_speed_for_pwm(f["pwm_min"], f["ff"], top)
        print(f"  {side}: starts at PWM {f['start']:.0f}; PWM = {f['pwm_min']:.1f} + {f['ff']:.4f} x mm/s "
              f"({f['n']} points); about {top_s:.0f} mm/s at PWM {top}")
        if f["dropped"]:
            print(f"  {side}: WARNING speed FELL at PWM {f['dropped']} although PWM rose (left out of the fit). "
                  "Stall, binding or the driver limiting current: check that wheel turns freely, then calibrate again.")
    if not any(fits.values()):
        print("No fit: the wheels hardly moved up to PWM", top, "(sketch MAX_PWM). Check motor power and the drivers.")
        return False
    print("\nPut these in config/arc_hw.yaml:")
    for side, f in fits.items():
        if f:
            print(f"    pwm_start_{side}: {f['start']:.1f}\n    pwm_min_{side}: {f['pwm_min']:.2f}\n    ff_{side}: {f['ff']:.5f}")
    if not all(fits.values()):
        print("  (one wheel has no fit: fix it before driving, or the cart will turn instead of going straight)")
    return all(fits.values())


def t_step(b):
    a = b.a
    seq = [0, 50, 100, 75, 0]
    hold = a.hold or 4.0
    cap = min(a.max_pwm, P.MAX_PWM_SKETCH)
    print(f"speed steps {seq} mm/s, {hold:.0f} s each; left {b.ffw[0]}, right {b.ffw[1]} (start, pwm_min, ff)")
    t0 = time.time()
    for target in seq:
        (sl, ml, fl), (sr, mr, fr) = b.ffw
        pl = P.speed_to_pwm(target, ml, fl, cap, pwm_start=sl)
        pr = P.speed_to_pwm(target, mr, fr, cap, pwm_start=sr)

        def rec(l, r, target=target, pl=pl, pr=pr):
            row = [round(time.time() - t0, 2), target, pl, pr, l, r, round(b.speed(l), 1), round(b.speed(r), 1)]
            b.rows.append(row)
            print(f"  t {row[0]:5.1f} s  target {target:5.1f}  PWM {pl:3d}/{pr:3d}  "
                  f"left {row[6]:6.1f}  right {row[7]:6.1f} mm/s")
        b.run_for(hold, pl, pr, on_enc=rec)
    b.stop()
    b.save("step", ["t", "target_mm_s", "pwm_l", "pwm_r", "enc_l", "enc_r", "meas_l_mm_s", "meas_r_mm_s"])
    return True


def default_config():
    """The YAML you edit (src), else the installed copy."""
    src = os.path.expanduser("~/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml")
    if os.path.exists(src):
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("arc_hw_bridge"), "config", "arc_hw.yaml")
        return p if os.path.exists(p) else None
    except Exception:
        return None


def load_config(path):
    if not path or not os.path.exists(path):
        return {}
    try:
        import yaml
        with open(path) as f:
            return (yaml.safe_load(f) or {}).get("arc_hw_bridge", {}).get("ros__parameters", {}) or {}
    except Exception as e:
        print(f"WARN could not read {path}: {e}")
        return {}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=["check", "monitor", "watchdog", "calibrate", "step"])
    ap.add_argument("--config", default=None, help="YAML to read port/wheel/ppr/calibration from "
                    "(default ~/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml); command-line values win")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wheel", type=float, default=None, help="wheel diameter (m)")
    ap.add_argument("--ppr", type=int, default=None, help="encoder pulses per WHEEL turn (motor PPR x gear ratio)")
    ap.add_argument("--pwm", type=int, default=40, help="watchdog: PWM to drive at")
    ap.add_argument("--max-pwm", type=int, default=60)
    ap.add_argument("--step-pwm", type=int, default=5, help="calibrate: PWM increment")
    ap.add_argument("--hold", type=float, default=None, help="s per level (calibrate 5, step 4; at least 3)")
    a = ap.parse_args(argv)
    cfg_path = a.config or default_config()
    cfg = load_config(cfg_path)
    a.port = a.port or cfg.get("port", "/dev/ttyACM0")
    a.wheel = a.wheel or float(cfg.get("wheel_diameter", 0.10))
    a.ppr = a.ppr or int(cfg.get("pulses_per_rev", 8))
    a.ffw = [tuple(float(cfg.get(f"{k}_{s}", d)) for k, d in (("pwm_start", 25.0), ("pwm_min", 20.0), ("ff", 0.2)))
             for s in ("left", "right")]
    if cfg:
        print(f"settings from {cfg_path}: port {a.port}, wheel {a.wheel} m, {a.ppr} pulses per wheel turn")
    try:
        import serial  # noqa: F401
    except ImportError:
        sys.exit("pyserial missing: sudo apt install python3-serial")
    print("WHEELS OFF THE GROUND. E-stop within reach.")
    b = Bench(a)
    try:
        ok = {"check": t_check, "monitor": t_monitor, "watchdog": t_watchdog,
              "calibrate": t_calibrate, "step": t_step}[a.test](b)
    finally:
        b.stop()
        b.link.ser and b.link.ser.close()
    sys.exit(0 if ok in (True, None) else 1)


if __name__ == "__main__":
    main()
