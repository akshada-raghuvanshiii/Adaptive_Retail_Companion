"""Serial protocol of the team's Uno sketch (firmware/arc_cart_motor/arc_cart_motor.ino) and the
differential-drive maths. No ROS and no pyserial here, so it can be unit-tested anywhere.

The sketch, 115200 baud, one line per message:
    to the Uno     M,<left>,<right>   PWM -MAX_PWM..MAX_PWM (sign = direction; the sketch clamps to MAX_PWM=60)
                   STOP               stop now
                   PING               the Uno answers ARC_MOTOR_READY
    from the Uno   OK                 after every accepted M command
                   OK,STOP            after STOP
                   ARC_MOTOR_READY    at power-up and after PING
                   ERR,ESTOP          M refused: the E-stop loop on D4 is open (D4 must read LOW)
                   ERR,COMMAND / ERR,TOO_LONG
                   ENC,<l>,<r>        pulses counted in the last TELEMETRY_MS (1000 ms), then reset to 0.
                                      No direction: only encoder channel A is counted, so the sign comes from the command.
    Watchdog: no M for 300 ms -> the sketch stops both motors. So M must be sent at 5 Hz or more.
"""
import math
from dataclasses import dataclass

MAX_PWM_SKETCH = 60            # const int MAX_PWM in the sketch
SKETCH_WATCHDOG_S = 0.3
TELEMETRY_PERIOD_S = 1.0       # TELEMETRY_MS / 1000


@dataclass
class Geometry:
    wheel_diameter_m: float      # measured on the cart
    track_width_m: float         # centre of left tyre to centre of right tyre
    pulses_per_rev: int          # encoder pulses per WHEEL turn (motor PPR x gear ratio)

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


def speed_to_pwm(v_mm_s, pwm_min, ff, max_pwm, deadband_mm_s=5.0, pwm_start=0.0):
    """Feed-forward: |PWM| = max(pwm_start, pwm_min + ff * |v|), signed like v, 0 inside the deadband,
    capped at max_pwm. pwm_min + ff*v is the straight line through the moving points; pwm_start is the
    lowest PWM at which the wheel actually breaks free (often above that line's value at low speed)."""
    if abs(v_mm_s) < deadband_mm_s:
        return 0
    p = max(pwm_start, pwm_min + ff * abs(v_mm_s))
    p = min(p, max_pwm)
    return int(round(math.copysign(p, v_mm_s)))


def max_speed_for_pwm(pwm_min, ff, max_pwm):
    """Highest wheel speed (mm/s) the feed-forward can ask for before PWM hits the cap."""
    return max(0.0, (max_pwm - pwm_min) / ff) if ff > 0 else 0.0


def cmd_motor(pl, pr):
    return f"M,{int(pl)},{int(pr)}\n"


CMD_STOP = "STOP\n"
CMD_PING = "PING\n"


def parse_line(line):
    """One line from the sketch -> (kind, ...) or None for anything unknown.

    ('enc', left_pulses, right_pulses) | ('ok',) | ('ok_stop',) | ('ready',) | ('err', CODE)
    """
    s = line.strip()
    if s == "OK":
        return ("ok",)
    if s == "OK,STOP":
        return ("ok_stop",)
    if s == "ARC_MOTOR_READY":
        return ("ready",)
    if s.startswith("ERR,"):
        return ("err", s[4:])
    if s.startswith("ENC,"):
        p = s.split(",")
        if len(p) == 3:
            try:
                return ("enc", int(p[1]), int(p[2]))
            except ValueError:
                return None
    return None


def signed_speed(pulses, sign, mm_per_pulse, period_s=TELEMETRY_PERIOD_S):
    """Pulses in one telemetry period -> mm/s, signed by the direction that was commanded."""
    s = 1.0 if sign >= 0 else -1.0
    return s * pulses * mm_per_pulse / period_s


def fit_wheel(points, drop_ratio=0.7):
    """Per-wheel calibration from (pwm, measured speed) pairs of a rising PWM sweep.

    - start: lowest PWM from which the wheel keeps moving at every higher PWM tried;
    - dropped: PWMs where the speed fell below drop_ratio x the best speed at a lower PWM
      (stall, binding, driver current limit); left out of the fit and reported;
    - pwm_min, ff: least squares PWM = pwm_min + ff * speed on the remaining moving points.
    Returns a dict, or None if fewer than 2 usable points or a non-positive slope.
    """
    pts = sorted(points)
    dropped, best, usable = [], 0.0, []
    for pwm, s in pts:
        if s > 1.0 and best > 0 and s < drop_ratio * best:
            dropped.append(pwm)
            continue
        if s > 1.0:
            usable.append((pwm, s))
        best = max(best, s)
    start = None
    for i, (pwm, s) in enumerate(pts):
        if s > 1.0 and all(s2 > 1.0 for p2, s2 in pts[i:] if p2 not in dropped):
            start = pwm
            break
    usable = [(p, s) for p, s in usable if start is not None and p >= start]
    f = fit_feedforward(usable)
    if f is None or f[1] <= 0:
        return None
    return {"start": float(start), "pwm_min": f[0], "ff": f[1], "dropped": dropped, "n": len(usable)}


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
