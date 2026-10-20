"""Single-target lock on top of a multi-object tracker (standard library only).

The tracker (ByteTrack / BoT-SORT) gives every person an ID, but IDs break
and occasionally swap. A follow-me cart needs one thing: keep following the
enrolled shopper, and never follow anyone else. This class turns the
tracker's per-frame output into exactly that.

States
------
IDLE         nobody enrolled yet
LOCKED       following the enrolled shopper (target track visible)
LOST         target not visible; the cart must stop (WAIT) while we look
NEEDS_ENROL  lost for longer than max_lost_s; the shopper must re-enrol

Re-acquisition (LOST -> LOCKED) needs ALL of:
  * the target did not leave through the image border (if it did, a newcomer
    at the same spot is indistinguishable: re-enrol instead),
  * the original track ID reappears, OR a candidate track that
      - was never seen at the same time as the target, other than as a
        duplicate box of it (otherwise it is a different person),
      - does not overlap another person's box,
      - is near the predicted position (gate grows with time lost),
      - has a similar box height,
      - has an appearance similarity >= min_similarity, clearly better
        than the second-best candidate (ambiguity margin),
      - passes these checks for confirm_frames consecutive frames.
If anything is ambiguous the lock stays LOST: stopping is the safe failure.

While LOCKED, a sustained drop in appearance similarity is treated as a
tracker ID swap: the lock drops to LOST and that track ID is blacklisted.

Coordinates are image pixels and times are seconds. Appearance features
are optional normalised histograms (see arc_assist.appearance); without
them, only original-ID recovery is allowed.
"""

from dataclasses import dataclass, field
from enum import Enum
from math import hypot, sqrt
from typing import Optional, Sequence, Tuple

Box = Tuple[float, float, float, float]  # x1, y1, x2, y2


class LockState(str, Enum):
    IDLE = "IDLE"
    LOCKED = "LOCKED"
    LOST = "LOST"
    NEEDS_ENROL = "NEEDS_ENROL"


@dataclass(frozen=True)
class Detection:
    track_id: int
    box: Box
    conf: float = 1.0
    feature: Optional[Tuple[float, ...]] = None


@dataclass
class LockConfig:
    # appearance signature
    signature_alpha: float = 0.05        # EMA weight for signature updates
    update_min_conf: float = 0.5         # only learn from confident boxes...
    update_max_iou: float = 0.3          # ...that are not overlapped by someone else
    min_similarity: float = 0.60         # Bhattacharyya coefficient, 1 = identical
    ambiguity_margin: float = 0.08       # best must beat second-best by this
    # geometry gates (in units of the target's box height)
    gate_base_h: float = 0.6
    gate_growth_h_per_s: float = 1.0
    gate_max_h: float = 3.0
    size_ratio_min: float = 0.5
    size_ratio_max: float = 2.0
    prediction_cap_s: float = 1.0        # extrapolate motion for at most this long
    # timing
    confirm_frames: int = 3
    max_lost_s: float = 3.0              # automatic re-acquisition window; then re-enrol (v1: 10 s)
    # leaving the camera view: no automatic re-acquisition (someone else may enter there)
    image_size: Optional[Tuple[int, int]] = None   # (width, height); None disables the check
    border_margin_px: float = 12.0
    # never re-acquire onto a box that overlaps another person's box
    candidate_max_iou: float = 0.3
    # ID-swap detection while locked
    switch_similarity: float = 0.45
    switch_frames: int = 5
    # "known other" tracks: seen at the same time as the target and not a duplicate box of it.
    # A duplicate (e.g. upper-body box inside a full-body box) lies almost entirely inside
    # the other box; two different people overlapping in depth do not. (v1 used IoU <= 0.1.)
    other_max_containment: float = 0.7
    other_min_frames: int = 3
    require_appearance: bool = True
    similarity_kind: str = "bhattacharyya"   # "bhattacharyya" (histograms) or "cosine" (ReID embeddings)


@dataclass
class LockOutput:
    state: LockState
    track_id: Optional[int]
    box: Optional[Box]
    reason: str


def _iou(a: Box, b: Box) -> float:
    ix1, iy1, ix2, iy2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0:
        return 0.0
    area = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / area if area > 0 else 0.0


def _containment(a: Box, b: Box) -> float:
    """Intersection over the smaller box's area (1 = one box inside the other)."""
    ix1, iy1, ix2, iy2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    smaller = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
    return inter / smaller if smaller > 0 else 0.0


def _centre(b: Box) -> Tuple[float, float]:
    return (b[0] + b[2]) / 2, (b[1] + b[3]) / 2


def _height(b: Box) -> float:
    return max(1e-6, b[3] - b[1])


def similarity(p: Optional[Sequence[float]], q: Optional[Sequence[float]],
               kind: str = "bhattacharyya") -> Optional[float]:
    """1 = identical. Bhattacharyya coefficient for normalised histograms,
    cosine similarity for embedding vectors."""
    if p is None or q is None or len(p) != len(q):
        return None
    if kind == "cosine":
        dot = sum(a * b for a, b in zip(p, q))
        na, nb = sqrt(sum(a * a for a in p)), sqrt(sum(b * b for b in q))
        return dot / (na * nb) if na > 0 and nb > 0 else None
    return sum(sqrt(max(a, 0.0) * max(b, 0.0)) for a, b in zip(p, q))


@dataclass
class TargetLock:
    config: LockConfig = field(default_factory=LockConfig)

    def __post_init__(self):
        self.reset()

    # ------------------------------------------------------------------ API
    def reset(self):
        self.state = LockState.IDLE
        self.target_id: Optional[int] = None
        self.signature: Optional[list] = None
        self.last_box: Optional[Box] = None
        self.last_t: Optional[float] = None
        self.velocity = (0.0, 0.0)         # px/s
        self.lost_since: Optional[float] = None
        self.blacklist: set = set()
        self.known_others: dict = {}       # track_id -> frames seen apart from target
        self._candidate_id: Optional[int] = None
        self._candidate_streak = 0
        self._low_sim_streak = 0
        self.relocks = 0
        self.exited = False

    def enrol(self, det: Detection, t: float):
        """Start following this detection's person."""
        self.reset()
        self.state = LockState.LOCKED
        self.target_id = det.track_id
        self.signature = list(det.feature) if det.feature is not None else None
        self.last_box = det.box
        self.last_t = t

    def update(self, detections: Sequence[Detection], t: float) -> LockOutput:
        if self.state == LockState.IDLE:
            return LockOutput(self.state, None, None, "not enrolled")
        if self.state == LockState.NEEDS_ENROL:
            return LockOutput(self.state, None, None,
                              f"lost > {self.config.max_lost_s:.0f} s: cart waits, shopper must re-enrol")
        by_id = {d.track_id: d for d in detections}

        if self.state == LockState.LOCKED:
            det = by_id.get(self.target_id)
            if det is not None:
                return self._follow(det, detections, t)
            self.state = LockState.LOST
            self.lost_since = t
            self._candidate_id, self._candidate_streak = None, 0
            self.exited = self._touches_border(self.last_box)
            # fall through to the LOST handling in the same frame

        # LOST
        if t - self.lost_since > self.config.max_lost_s:
            self.state = LockState.NEEDS_ENROL
            return LockOutput(self.state, None, None, f"lost > {self.config.max_lost_s:.0f} s")

        det = by_id.get(self.target_id)
        if det is not None:  # tracker recovered the original track
            self._relock(det, t, "original ID recovered")
            return self._follow(det, detections, t)

        return self._try_reacquire(detections, t)

    # ------------------------------------------------------------ internals
    def _follow(self, det: Detection, detections, t) -> LockOutput:
        cfg = self.config
        sim = similarity(det.feature, self.signature, cfg.similarity_kind)
        if sim is not None and sim < cfg.switch_similarity:
            self._low_sim_streak += 1
            if self._low_sim_streak >= cfg.switch_frames:
                self.blacklist.add(det.track_id)
                self.target_id = None  # never "recover" the swapped ID
                self.state = LockState.LOST
                self.lost_since = t
                self._low_sim_streak = 0
                return LockOutput(self.state, None, None,
                                  f"appearance changed on ID {det.track_id}: possible ID swap")
        else:
            self._low_sim_streak = 0

        # motion estimate
        if self.last_box is not None and self.last_t is not None and t > self.last_t:
            (cx0, cy0), (cx1, cy1) = _centre(self.last_box), _centre(det.box)
            dt = t - self.last_t
            vx, vy = (cx1 - cx0) / dt, (cy1 - cy0) / dt
            self.velocity = (0.7 * self.velocity[0] + 0.3 * vx, 0.7 * self.velocity[1] + 0.3 * vy)
        self.last_box, self.last_t = det.box, t

        # learn appearance only from clean views
        others = [d for d in detections if d.track_id != det.track_id]
        overlapped = any(_iou(det.box, d.box) > cfg.update_max_iou for d in others)
        if det.feature is not None and det.conf >= cfg.update_min_conf and not overlapped:
            if self.signature is None:
                self.signature = list(det.feature)
            else:
                a = cfg.signature_alpha
                self.signature = [(1 - a) * s + a * f for s, f in zip(self.signature, det.feature)]

        # remember who is definitely someone else
        for d in others:
            if _containment(det.box, d.box) < cfg.other_max_containment:
                self.known_others[d.track_id] = self.known_others.get(d.track_id, 0) + 1

        return LockOutput(LockState.LOCKED, det.track_id, det.box, "following")

    def _relock(self, det: Detection, t: float, reason: str):
        if det.track_id != self.target_id:
            self.relocks += 1
        self.target_id = det.track_id
        self.state = LockState.LOCKED
        self.lost_since = None
        self._candidate_id, self._candidate_streak = None, 0
        self.exited = False
        self._last_relock_reason = reason

    def _touches_border(self, b: Optional[Box]) -> bool:
        if b is None or self.config.image_size is None:
            return False
        w, h = self.config.image_size
        m = self.config.border_margin_px
        return b[0] <= m or b[1] <= m or b[2] >= w - m or b[3] >= h - m

    def _predicted_centre(self, t: float) -> Tuple[float, float]:
        cx, cy = _centre(self.last_box)
        dt = min(max(0.0, t - self.last_t), self.config.prediction_cap_s)
        return cx + self.velocity[0] * dt, cy + self.velocity[1] * dt

    def _try_reacquire(self, detections, t) -> LockOutput:
        cfg = self.config
        lost_for = t - self.lost_since
        if self.exited:
            self._candidate_id, self._candidate_streak = None, 0
            return LockOutput(LockState.LOST, None, None, "target left the camera view: re-enrol")
        h = _height(self.last_box)
        gate = min(cfg.gate_max_h, cfg.gate_base_h + cfg.gate_growth_h_per_s * lost_for) * h
        px, py = self._predicted_centre(t)

        scored = []
        for d in detections:
            if d.track_id in self.blacklist:
                continue
            if self.known_others.get(d.track_id, 0) >= cfg.other_min_frames:
                continue
            cx, cy = _centre(d.box)
            if hypot(cx - px, cy - py) > gate:
                continue
            ratio = _height(d.box) / h
            if not cfg.size_ratio_min <= ratio <= cfg.size_ratio_max:
                continue
            if any(_iou(d.box, q.box) > cfg.candidate_max_iou for q in detections if q is not d):
                continue  # overlapping another person: wait until they separate
            sim = similarity(d.feature, self.signature, cfg.similarity_kind)
            if sim is None:
                if cfg.require_appearance:
                    continue
                sim = 0.0
            elif sim < cfg.min_similarity:
                continue
            scored.append((sim, d))

        scored.sort(key=lambda s: s[0], reverse=True)
        if not scored:
            self._candidate_id, self._candidate_streak = None, 0
            return LockOutput(LockState.LOST, None, None, f"no candidate ({lost_for:.1f} s)")
        if len(scored) > 1 and scored[0][0] - scored[1][0] < cfg.ambiguity_margin:
            self._candidate_id, self._candidate_streak = None, 0
            return LockOutput(LockState.LOST, None, None, "ambiguous: two similar candidates")

        best = scored[0][1]
        if best.track_id == self._candidate_id:
            self._candidate_streak += 1
        else:
            self._candidate_id, self._candidate_streak = best.track_id, 1
        if self._candidate_streak >= cfg.confirm_frames:
            self._relock(best, t, "re-acquired")
            out = self._follow(best, detections, t)
            out.reason = f"re-acquired as ID {best.track_id}"
            return out
        return LockOutput(LockState.LOST, None, None,
                          f"confirming ID {best.track_id} ({self._candidate_streak}/{cfg.confirm_frames})")
