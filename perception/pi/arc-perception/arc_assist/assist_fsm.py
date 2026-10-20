"""Cart assist mode: FOLLOW / OFFER_HELP / LEAD / WAIT (standard library only).

This is a reference implementation and test oracle for the behaviour tree's
mode logic. In the ROS stack the behaviour tree stays the owner of the mode;
mirror these transitions there, or call this class from a BT condition node.

Rules that matter for safety:
  * Target not locked (LOST / NEEDS_ENROL)  -> WAIT, from every mode.
  * LEAD only after an explicit "yes" (or the shopper's own "lead me").
  * While leading, if the shopper falls behind  -> WAIT until they catch up.
  * An unanswered offer times out back to FOLLOW (treated as "no").
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .target_lock import LockState


class Mode(str, Enum):
    IDLE = "IDLE"            # nobody enrolled
    FOLLOW = "FOLLOW"
    OFFER_HELP = "OFFER_HELP"
    LEAD = "LEAD"
    WAIT = "WAIT"            # stop: target lost, or shopper fell behind while leading


@dataclass
class FSMConfig:
    offer_timeout_s: float = 15.0
    max_lead_gap_m: float = 3.0      # shopper-to-cart distance that pauses leading
    resume_lead_gap_m: float = 2.0   # hysteresis


@dataclass
class FSMOutput:
    mode: Mode
    reason: str
    offer_answered: bool = False     # integrator should call HelpTrigger.answered()


class AssistFSM:
    def __init__(self, config: Optional[FSMConfig] = None):
        self.cfg = config or FSMConfig()
        self.mode = Mode.IDLE
        self._resume = Mode.FOLLOW
        self._offer_t = None
        self._wait_reason = ""

    def step(self, t: float, lock_state: LockState, offer: bool = False,
             user: Optional[str] = None, at_goal: bool = False,
             shopper_gap_m: Optional[float] = None) -> FSMOutput:
        """user: None | "yes" | "no" | "lead_me" | "stop"."""
        cfg = self.cfg

        if user == "stop":
            self.mode = Mode.WAIT
            self._resume = Mode.FOLLOW
            self._wait_reason = "user stop"
            return FSMOutput(self.mode, "user pressed stop")

        locked = lock_state == LockState.LOCKED
        if not locked:
            if self.mode == Mode.IDLE:
                return FSMOutput(Mode.IDLE, "waiting for enrolment")
            if self.mode != Mode.WAIT:
                self._resume = Mode.LEAD if self.mode == Mode.LEAD else Mode.FOLLOW
            answered = self.mode == Mode.OFFER_HELP
            self.mode = Mode.WAIT
            self._wait_reason = "target lost"
            why = "re-enrolment needed" if lock_state == LockState.NEEDS_ENROL else "target lost: stop"
            return FSMOutput(self.mode, why, offer_answered=answered)

        if self.mode == Mode.IDLE:
            self.mode = Mode.FOLLOW
            return FSMOutput(self.mode, "enrolled")

        if self.mode == Mode.WAIT:
            if self._wait_reason == "shopper behind":
                if shopper_gap_m is not None and shopper_gap_m <= cfg.resume_lead_gap_m:
                    self.mode = Mode.LEAD
                    return FSMOutput(self.mode, "shopper caught up")
                return FSMOutput(self.mode, "waiting for shopper to catch up")
            if self._wait_reason == "user stop" and user not in ("yes", "lead_me"):
                return FSMOutput(self.mode, "stopped by user")
            self.mode = self._resume
            return FSMOutput(self.mode, "target re-acquired")

        if self.mode == Mode.FOLLOW:
            if user == "lead_me":
                self.mode = Mode.LEAD
                return FSMOutput(self.mode, "shopper asked to be led")
            if offer:
                self.mode = Mode.OFFER_HELP
                self._offer_t = t
                return FSMOutput(self.mode, "help offered")
            return FSMOutput(self.mode, "following")

        if self.mode == Mode.OFFER_HELP:
            if user in ("yes", "lead_me"):
                self.mode = Mode.LEAD
                return FSMOutput(self.mode, "shopper accepted", offer_answered=True)
            if user == "no":
                self.mode = Mode.FOLLOW
                return FSMOutput(self.mode, "shopper declined", offer_answered=True)
            if t - self._offer_t >= cfg.offer_timeout_s:
                self.mode = Mode.FOLLOW
                return FSMOutput(self.mode, "offer timed out", offer_answered=True)
            return FSMOutput(self.mode, "awaiting answer")

        if self.mode == Mode.LEAD:
            if at_goal:
                self.mode = Mode.FOLLOW
                return FSMOutput(self.mode, "arrived at item")
            if user == "no":
                self.mode = Mode.FOLLOW
                return FSMOutput(self.mode, "shopper cancelled leading")
            if shopper_gap_m is not None and shopper_gap_m > cfg.max_lead_gap_m:
                self.mode = Mode.WAIT
                self._wait_reason = "shopper behind"
                return FSMOutput(self.mode, f"shopper {shopper_gap_m:.1f} m behind: pause")
            return FSMOutput(self.mode, "leading")

        return FSMOutput(self.mode, "no change")
