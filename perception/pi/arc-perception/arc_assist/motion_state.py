"""Is the enrolled shopper walking or standing still? Image-motion only (standard library).

Measures how fast the target box moves in the image, in box-heights per second, so it does not
depend on distance or image resolution: lateral motion of the box centre, plus the relative change
of box height (walking towards or away from the camera). Hysteresis and hold times stop normal
swaying and detector jitter from flickering the state. Starts as STILL (the safe default: the cart
waits). This measures movement only; it does not infer anything about the person's state of mind.

Caveat: valid for a camera that is not itself moving (the laptop webcam, a fixed camera). On a
moving cart the camera's own motion adds image motion; that needs ego-motion compensation first.
"""

from collections import deque
from math import hypot


class MotionState:
    def __init__(self, window_s=1.0, move_thr=0.15, still_thr=0.07, move_hold_s=0.4, still_hold_s=1.0,
                 min_span_s=0.25):
        self.window_s, self.move_thr, self.still_thr = window_s, move_thr, still_thr
        self.move_hold_s, self.still_hold_s, self.min_span_s = move_hold_s, still_hold_s, min_span_s
        self.reset()

    def reset(self):
        self.hist = deque()
        self.moving = False
        self.speed = 0.0
        self._above_since = None
        self._below_since = None

    def update(self, t, box):
        """box: (x1, y1, x2, y2) of the target this frame. Returns True while moving."""
        cx, cy, h = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2, max(1e-6, box[3] - box[1])
        self.hist.append((t, cx, cy, h))
        while self.hist and t - self.hist[0][0] > self.window_s:
            self.hist.popleft()
        t0, x0, y0, h0 = self.hist[0]
        span = t - t0
        if span < self.min_span_s:
            return self.moving
        hm = (h + h0) / 2
        lateral = hypot(cx - x0, cy - y0) / hm / span
        depth = abs(h - h0) / hm / span
        self.speed = max(lateral, depth)
        if self.speed >= self.move_thr:
            self._below_since = None
            self._above_since = self._above_since if self._above_since is not None else t
            if not self.moving and t - self._above_since >= self.move_hold_s:
                self.moving = True
        elif self.speed <= self.still_thr:
            self._above_since = None
            self._below_since = self._below_since if self._below_since is not None else t
            if self.moving and t - self._below_since >= self.still_hold_s:
                self.moving = False
        else:
            self._above_since = self._below_since = None
        return self.moving
