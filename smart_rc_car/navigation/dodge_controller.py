"""
navigation/dodge_controller.py
==============================
State machine that manages the dodge lifecycle:

  NORMAL → DODGING → RETURNING → NORMAL

When an obstacle is detected, transitions to DODGING and applies
a smooth steering override. When the obstacle clears, transitions
to RETURNING and blends back toward the driver's original command.

The blend is smooth to avoid sudden steering reversals.
"""
from __future__ import annotations

import logging
import time
from enum import Enum, auto

from smart_rc_car.config.settings import CFG
from smart_rc_car.control.smoothing import RateLimiter
from smart_rc_car.navigation.obstacle_avoidance import AvoidanceAction, AvoidanceDecision

log = logging.getLogger(__name__)


class DodgeState(Enum):
    NORMAL    = auto()
    DODGING   = auto()
    RETURNING = auto()

    def __str__(self):
        return self.name


class DodgeController:
    """
    Manages the dodge state machine and produces smooth steering commands.

    Returns a blended (steering, throttle_scale) pair each update cycle.
    """

    def __init__(self):
        ncfg = CFG.navigation
        self._hold_s       = ncfg.dodge_hold_s
        self._blend_rate   = ncfg.return_blend_rate
        self._steer_limiter = RateLimiter(rate=0.08)   # max steer change per tick

        self._state        = DodgeState.NORMAL
        self._dodge_steer  = 0.0    # The AI's target steering during dodge
        self._dodge_start  = 0.0
        self._clear_start: float | None = None

    # ── Public update ─────────────────────────────────────────────────────────

    def update(
        self,
        decision: AvoidanceDecision,
        driver_steering: float,
        dt: float = 0.033,
    ) -> tuple[float, float]:
        """
        Update the dodge state machine.

        Parameters
        ----------
        decision        : AvoidanceDecision from obstacle_avoidance.decide()
        driver_steering : raw driver steering from RC (-1…+1)
        dt              : seconds since last call

        Returns
        -------
        (steering, throttle_scale)
            steering     : final steering command -1…+1
            throttle_scale: multiplier for driver throttle
        """
        action = decision.action

        # ── Transitions ────────────────────────────────────────────────────────
        if action in (AvoidanceAction.DODGE_LEFT, AvoidanceAction.DODGE_RIGHT):
            if self._state != DodgeState.DODGING:
                log.info(f"Dodge START → {action}: {decision.reason}")
                self._state       = DodgeState.DODGING
                self._dodge_steer = decision.steer_override
                self._dodge_start = time.monotonic()
                self._clear_start = None

        elif action == AvoidanceAction.STOP:
            self._state = DodgeState.NORMAL
            self._steer_limiter.reset(0.0)
            return 0.0, 0.0

        elif action == AvoidanceAction.SLOW_DOWN:
            # Partial block — follow driver's steering but slow down
            steer = self._steer_limiter.update(driver_steering)
            return steer, decision.throttle_scale

        elif action == AvoidanceAction.FOLLOW_DRIVER:
            if self._state == DodgeState.DODGING:
                # Obstacle just cleared — start return timer
                if self._clear_start is None:
                    self._clear_start = time.monotonic()
                    log.info("Obstacle cleared — beginning return.")
                    self._state = DodgeState.RETURNING
            elif self._state == DodgeState.RETURNING:
                # Already returning — continue blending
                pass

        # ── Output based on current state ─────────────────────────────────────
        if self._state == DodgeState.DODGING:
            target_steer = self._dodge_steer
            steer        = self._steer_limiter.update(target_steer)
            return steer, decision.throttle_scale

        elif self._state == DodgeState.RETURNING:
            elapsed = time.monotonic() - (self._clear_start or 0)
            # Blend from dodge steer toward driver steer over hold period
            blend = min(1.0, elapsed / max(0.1, self._hold_s))
            target_steer = (
                (1.0 - blend) * self._dodge_steer + blend * driver_steering
            )
            steer = self._steer_limiter.update(target_steer)

            if blend >= 1.0:
                log.info("Dodge complete — full driver control restored.")
                self._state = DodgeState.NORMAL
                self._steer_limiter.reset(driver_steering)

            return steer, 1.0  # Full throttle during return

        else:  # NORMAL
            steer = self._steer_limiter.update(driver_steering)
            return steer, 1.0

    @property
    def state(self) -> DodgeState:
        return self._state

    @property
    def is_overriding(self) -> bool:
        return self._state != DodgeState.NORMAL
