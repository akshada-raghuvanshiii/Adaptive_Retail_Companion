#!/usr/bin/env python3
"""Bench tests for the ARC motor bridge - no ROS needed. WHEELS OFF THE GROUND.

  python3 bench.py --port /dev/ttyACM0 --wheel 0.10 --track 0.40 --ppr 8 monitor
        print what the Arduino reports (spin a wheel by hand: counts should rise)
  ... calibrate     PWM sweep 0..200 on both wheels; prints pwm_min and ff to use
  ... step          speed steps 0 -> 100 -> 150 -> 50 -> 0 mm/s; saves a CSV
  ... watchdog      sends one command, then goes silent: the motors must stop within ~0.5 s

Every run writes a CSV in ./bench_logs/.
"""
import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arc_hw import protocol as P  # noqa: E402


def open_port(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial missing: sudo apt install python3-serial")
    s = serial.Serial(port, baud, timeout=0.05)
    time.sleep(2.0)            # the Uno resets when the port opens
    s.reset_input_buffer()
    return s


class Link:
    def __init__(self, ser, log_path):
        self.ser = ser
        self.buf = b""
        self.last = None
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        self.f = open(log_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "phase", "target_l", "target_r", "count_l", "count_r", "speed_l", "speed_r",
                         "pwm_l", "pwm_r"])
        self.phase, self.tl, self.tr = "", 0, 0
        self.t0 = time.time()

    def send(self, s):
        self.ser.write(s.encode())

    def poll(self, dur):
        end = time.time() + dur
        while time.time() < end:
            self.buf += self.ser.read(256)
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                r = P.parse_report(line.decode(errors="ignore"))
                if r:
                    self.last = r
                    self.w.writerow([f"{time.time() - self.t0:.3f}", self.phase, self.tl, self.tr, r.count_l,
                                     r.count_r, r.speed_l, r.speed_r, r.pwm_l, r.pwm_r])

    def hold(self, cmd, dur, period=0.1):
        """Keep re-sending cmd (the watchdog needs it) for dur seconds."""
        end = time.time() + dur
        while time.time() < end:
            self.send(cmd)
            self.poll(period)

    def close(self):
        self.send(P.CMD_STOP)
        self.f.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=["monitor", "calibrate", "step", "watchdog"])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wheel", type=float, required=True, help="wheel diameter (m)")
    ap.add_argument("--track", type=float, required=True, help="track width (m)")
    ap.add_argument("--ppr", type=int, required=True, help="Hall pulses per wheel revolution")
    ap.add_argument("--pwm-min", type=float, default=60)
    ap.add_argument("--ff", type=float, default=0.6)
    ap.add_argument("--ki", type=float, default=0.0)
    a = ap.parse_args()
    g = P.Geometry(a.wheel, a.track, a.ppr)
    log = os.path.join("bench_logs", f"{a.test}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    L = Link(open_port(a.port, a.baud), log)
    try:
        L.send(P.cmd_mm_per_pulse(g.mm_per_pulse))
        L.send(P.cmd_gains(a.pwm_min, a.ff, a.ki))
        L.poll(0.3)
        print(f"{g.mm_per_pulse:.1f} mm per pulse. Log: {log}")
        if a.test == "monitor":
            print("Ctrl-C to stop. Spin each wheel by hand: its count should change.")
            while True:
                L.poll(0.5)
                if L.last:
                    r = L.last
                    print(f"counts L {r.count_l:6d} R {r.count_r:6d}   speed L {r.speed_l:6.1f} R {r.speed_r:6.1f} mm/s")
        elif a.test == "calibrate":
            pts_l, pts_r = [], []
            for pwm in range(0, 201, 20):
                L.phase, L.tl, L.tr = f"pwm{pwm}", pwm, pwm
                L.hold(P.cmd_pwm(pwm, pwm), 1.5)          # settle
                sl, sr = [], []
                end = time.time() + 2.0
                while time.time() < end:
                    L.send(P.cmd_pwm(pwm, pwm)); L.poll(0.1)
                    if L.last:
                        sl.append(abs(L.last.speed_l)); sr.append(abs(L.last.speed_r))
                ml = sum(sl) / len(sl) if sl else 0
                mr = sum(sr) / len(sr) if sr else 0
                pts_l.append((pwm, ml)); pts_r.append((pwm, mr))
                print(f"PWM {pwm:3d}: left {ml:6.1f} mm/s   right {mr:6.1f} mm/s")
            L.send(P.CMD_STOP)
            for name, pts in (("left", pts_l), ("right", pts_r)):
                fit = P.fit_feedforward(pts)
                print(f"{name}: " + (f"pwm_min {fit[0]:.0f}, ff {fit[1]:.3f} PWM per mm/s" if fit
                                     else "did not move - check wiring, magnets and the sensor threshold pot"))
            print("Use the average of left and right as --pwm-min / --ff (bench.py, bridge_node.py).")
        elif a.test == "step":
            for v in (0, 100, 150, 50, 0, -100, 0):
                L.phase, L.tl, L.tr = f"v{v}", v, v
                L.hold(P.cmd_speed(v, v), 4.0)
                r = L.last
                print(f"target {v:5d} mm/s -> measured L {r.speed_l:6.1f} R {r.speed_r:6.1f}" if r else f"target {v}: no reports")
        elif a.test == "watchdog":
            L.phase, L.tl, L.tr = "cmd", 120, 120
            L.hold(P.cmd_speed(120, 120), 2.0)
            print("going silent ...")
            L.phase = "silent"
            t0 = time.time()
            stopped = None
            while time.time() - t0 < 3.0:
                L.poll(0.05)
                if L.last and L.last.pwm_l == 0 and L.last.pwm_r == 0 and stopped is None:
                    stopped = time.time() - t0
            print(f"PASS: motors stopped {stopped:.2f} s after the last command" if stopped is not None and stopped < 1.0
                  else "FAIL: motors did not stop within 1 s - do not run on the floor")
    except KeyboardInterrupt:
        pass
    finally:
        L.close()


if __name__ == "__main__":
    main()
