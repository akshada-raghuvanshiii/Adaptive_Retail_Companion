"""Serial protocol and differential-drive kinematics shared by bench.py and bridge_node.py.

No ROS and no pyserial imports here, so it can be unit-tested anywhere.
"""
import math
from dataclasses import dataclass


@dataclass
class Geometry:
    wheel_diameter_m: float      # measured on the cart
    track_width_m: float         # centre of left tyre to centre of right tyre
    pulses_per_rev: int          # magnets per wheel (or magnets x gear ratio if on the motor shaft)

    @property
    def mm_per_pulse(self):
        return math.pi * self.wheel_diameter_m * 1000.0 / self.pulses_per_rev


def twist_to_wheels(v, w, track_width_m):
    """Body velocity (m/s, rad/s) -> left/right wheel speed (mm/s)."""
    half = track_width_m / 2.0
    return (v - w * half) * 1000.0, (v + w * half) * 1000.0


def wheels_to_twist(vl_mm_s, vr_mm_s, track_width_m):
    vl, vr = vl_mm_s / 1000.0, vr_mm_s / 1000.0
    return (vl + vr) / 2.0, (vr - vl) / track_width_m


def clamp_wheels(vl, vr, max_mm_s):
    """Scale both wheels together so neither exceeds max (keeps the turn radius)."""
    m = max(abs(vl), abs(vr))
    if m <= max_mm_s or m == 0:
        return vl, vr
    k = max_mm_s / m
    return vl * k, vr * k


def cmd_speed(vl, vr):
    return f"V {vl:.1f} {vr:.1f}\n"


def cmd_pwm(pl, pr):
    return f"P {int(pl)} {int(pr)}\n"


def cmd_gains(pwm_min, ff, ki):
    return f"K {pwm_min:.1f} {ff:.4f} {ki:.4f}\n"


def cmd_mm_per_pulse(mm):
    return f"M {mm:.3f}\n"


CMD_STOP = "S\n"


@dataclass
class Report:
    ms: int
    count_l: int
    count_r: int
    speed_l: float   # mm/s, signed by commanded direction
    speed_r: float
    pwm_l: int
    pwm_r: int


def parse_report(line):
    """'E ms cl cr sl sr pl pr' -> Report, or None for anything else (banner, noise)."""
    p = line.strip().split()
    if len(p) != 8 or p[0] != "E":
        return None
    try:
        return Report(int(p[1]), int(p[2]), int(p[3]), float(p[4]), float(p[5]), int(p[6]), int(p[7]))
    except ValueError:
        return None


def fit_feedforward(points):
    """Fit PWM = pwm_min + ff * speed from (pwm, measured mm/s) pairs where the wheel moved.

    Returns (pwm_min, ff) or None. Least squares on the moving points.
    """
    pts = [(p, s) for p, s in points if s > 1.0]
    if len(pts) < 2:
        return None
    n = len(pts)
    ms = sum(s for _, s in pts) / n
    mp = sum(p for p, _ in pts) / n
    sxx = sum((s - ms) ** 2 for _, s in pts)
    if sxx == 0:
        return None
    ff = sum((s - ms) * (p - mp) for p, s in pts) / sxx
    return mp - ff * ms, ff
