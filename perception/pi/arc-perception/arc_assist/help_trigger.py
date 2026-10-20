"""Help-offer trigger from observable movement in the MAP frame (metres, seconds).

Use the global planner's path length as distance_fn: straight-line distance
misreads walking up a neighbouring aisle as progress or regress.

Input: the locked shopper's map-frame position at each tick (camera box +
depth -> cart frame -> map via TF/RTAB-Map pose) and the current shopping-list
goal. Output: whether to OFFER help now, with the reasons.

This is not confusion detection. It flags the observable pattern "not making
progress towards the current list item", and the shopper decides by answering
the offer. Signals:

  no_progress   distance-to-goal fell by less than min_progress_m over window_s
                (and the shopper is not already at the goal)
  dwell_away    speed below stop_speed for dwell_s or longer, away from the goal
  reversals     >= reversals_needed heading reversals (>= reversal_deg between
                consecutive >= reversal_min_seg_m path segments) in reversal_window_s
  revisits      >= revisits_needed re-entries into previously left map cells
                in revisit_window_s (off by default, see use_revisits)

Offer rule: (no_progress AND at least one supporting signal) OR no_progress
sustained for long_no_progress_factor x window_s. Never at the goal, never
during cooldown after an answer, never twice without an answer.

The default thresholds are untuned design values, not validated settings.
"""

from collections import deque
from dataclasses import dataclass
from math import atan2, degrees, floor, hypot
from typing import Callable, Optional, Tuple

XY = Tuple[float, float]


@dataclass
class TriggerConfig:
    window_s: float = 30.0             # chosen from the synthetic sweep (20/30/45 s); see RESULTS_LOG
    min_progress_m: float = 1.0
    goal_radius_m: float = 1.5
    smooth_alpha: float = 0.3          # EMA on noisy positions
    speed_window_s: float = 2.0
    stop_speed: float = 0.15           # m/s
    dwell_s: float = 8.0
    reversal_deg: float = 120.0
    reversal_min_seg_m: float = 0.75
    reversals_needed: int = 2
    reversal_window_s: float = 30.0
    cell_m: float = 2.0
    cell_dwell_s: float = 1.0          # must stay this long in a new cell before it counts (noise hysteresis)
    use_revisits: bool = False         # off by default: noisy, and added nothing in the sweep
    revisits_needed: int = 2
    revisit_window_s: float = 60.0
    long_no_progress_factor: float = 2.0
    cooldown_s: float = 60.0


@dataclass
class TriggerOutput:
    offer: bool
    reasons: tuple
    distance_to_goal: Optional[float]
    signals: dict


def euclidean(p: XY, goal: XY) -> float:
    return hypot(p[0] - goal[0], p[1] - goal[1])


class HelpTrigger:
    def __init__(self, config: Optional[TriggerConfig] = None,
                 distance_fn: Callable[[XY, XY], float] = euclidean):
        """distance_fn: use the global planner's path length in ROS; Euclidean by default."""
        self.cfg = config or TriggerConfig()
        self.distance_fn = distance_fn
        self.goal: Optional[XY] = None
        self._reset_motion()
        self.pending = False
        self.cooldown_until = -1e9

    # ------------------------------------------------------------------ API
    def set_goal(self, goal: Optional[XY], t: Optional[float] = None):
        """Call when the shopping list advances. Clears history, keeps cooldown."""
        self.goal = goal
        self._reset_motion()
        self.pending = False

    def answered(self, t: float):
        """Shopper answered the offer (yes or no): start the cooldown."""
        self.pending = False
        self.cooldown_until = t + self.cfg.cooldown_s
        self._reset_motion()

    def update(self, t: float, x: float, y: float) -> TriggerOutput:
        cfg = self.cfg
        if self._smooth is None:
            self._smooth = (x, y)
        else:
            a = cfg.smooth_alpha
            self._smooth = ((1 - a) * self._smooth[0] + a * x, (1 - a) * self._smooth[1] + a * y)
        p = self._smooth
        self._pos.append((t, p))
        while self._pos and t - self._pos[0][0] > max(cfg.window_s * cfg.long_no_progress_factor,
                                                       cfg.revisit_window_s) + 1:
            self._pos.popleft()

        d = self.distance_fn(p, self.goal) if self.goal is not None else None
        at_goal = d is not None and d <= cfg.goal_radius_m
        speed = self._speed(t)
        self._update_dwell(t, speed, at_goal)
        self._update_reversals(t, p)
        self._update_revisits(t, p)

        npg = self._no_progress_for(t)
        signals = {
            "speed_mps": speed,
            "at_goal": at_goal,
            "no_progress_s": npg,
            "dwell_s": (t - self._dwell_start) if self._dwell_start is not None else 0.0,
            "reversals": self._recent(self._reversal_times, t, cfg.reversal_window_s),
            "revisits": self._recent(self._revisit_times, t, cfg.revisit_window_s),
        }

        reasons = []
        if (d is not None and not at_goal and not self.pending and t >= self.cooldown_until
                and npg >= cfg.window_s):
            if signals["dwell_s"] >= cfg.dwell_s:
                reasons.append(f"stopped {signals['dwell_s']:.0f} s away from the item")
            if signals["reversals"] >= cfg.reversals_needed:
                reasons.append(f"{signals['reversals']} direction reversals")
            if cfg.use_revisits and signals["revisits"] >= cfg.revisits_needed:
                reasons.append(f"{signals['revisits']} revisits of the same area")
            if npg >= cfg.window_s * cfg.long_no_progress_factor:
                reasons.append(f"no progress for {npg:.0f} s")
            if reasons:
                reasons.insert(0, f"distance to item not reduced by {cfg.min_progress_m:.1f} m "
                                  f"in {cfg.window_s:.0f} s")
        offer = bool(reasons)
        if offer:
            self.pending = True
        return TriggerOutput(offer, tuple(reasons), d, signals)

    # ------------------------------------------------------------ internals
    def _reset_motion(self):
        self._smooth = None
        self._pos = deque()
        self._dist = deque()
        self._dwell_start = None
        self._seg_start = None
        self._last_heading = None
        self._reversal_times = []
        self._cell = None
        self._cand_cell = None
        self._cand_since = None
        self._left_cells = set()
        self._revisit_times = []
        self._np_since = None

    @staticmethod
    def _recent(times, t, window):
        return sum(1 for x in times if t - x <= window)

    def _speed(self, t):
        w = self.cfg.speed_window_s
        older = [(tt, pp) for tt, pp in self._pos if t - tt >= w]
        if not older:
            return None
        t0, p0 = older[-1]
        _, p1 = self._pos[-1]
        return hypot(p1[0] - p0[0], p1[1] - p0[1]) / (t - t0)

    def _update_dwell(self, t, speed, at_goal):
        if speed is not None and speed < self.cfg.stop_speed and not at_goal:
            if self._dwell_start is None:
                self._dwell_start = t
        else:
            self._dwell_start = None

    def _update_reversals(self, t, p):
        if self._seg_start is None:
            self._seg_start = p
            return
        dx, dy = p[0] - self._seg_start[0], p[1] - self._seg_start[1]
        if hypot(dx, dy) < self.cfg.reversal_min_seg_m:
            return
        heading = degrees(atan2(dy, dx))
        if self._last_heading is not None:
            diff = abs((heading - self._last_heading + 180) % 360 - 180)
            if diff >= self.cfg.reversal_deg:
                self._reversal_times.append(t)
        self._last_heading = heading
        self._seg_start = p

    def _update_revisits(self, t, p):
        c = self.cfg.cell_m
        cell = (floor(p[0] / c), floor(p[1] / c))
        if cell == self._cell:
            self._cand_cell = None
            return
        if cell != self._cand_cell:
            self._cand_cell, self._cand_since = cell, t
            if self._cell is not None:
                return
        if self._cell is not None and t - self._cand_since < self.cfg.cell_dwell_s:
            return
        if cell in self._left_cells:
            self._revisit_times.append(t)
        if self._cell is not None:
            self._left_cells.add(self._cell)
        self._cell, self._cand_cell = cell, None

    def _no_progress_for(self, t):
        """Seconds for which progress over the trailing window has been below threshold."""
        if self.goal is None:
            return 0.0
        cfg = self.cfg
        _, p = self._pos[-1]
        d_now = self.distance_fn(p, self.goal)
        self._dist.append((t, d_now))
        while self._dist and t - self._dist[0][0] > cfg.window_s * cfg.long_no_progress_factor + 1:
            self._dist.popleft()
        past = [(tt, dd) for tt, dd in self._dist if t - tt >= cfg.window_s]
        if not past:
            self._np_since = None
            return 0.0
        _, d_then = past[-1]
        stalled = (d_then - d_now) < cfg.min_progress_m
        if not stalled:
            self._np_since = None
            return 0.0
        if self._np_since is None:
            self._np_since = t - cfg.window_s  # the stall began a window ago
        return t - self._np_since
