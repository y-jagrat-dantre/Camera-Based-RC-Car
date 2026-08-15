"""
navigation/obstacle_avoidance.py
=================================
Core obstacle avoidance decision table.

Given an ObstacleReport + driver's throttle command, decides
the appropriate action: FOLLOW_DRIVER, DODGE_LEFT, DODGE_RIGHT,
SLOW_DOWN, or STOP.

This is a pure function — no state, no threads. State is managed
by DodgeController.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from smart_rc_car.config.settings import CFG
from smart_rc_car.vision.driveable_area import ZoneStatus
from smart_rc_car.vision.obstacle_detector import ObstacleReport


class AvoidanceAction(Enum):
    FOLLOW_DRIVER = auto()
    DODGE_LEFT    = auto()
    DODGE_RIGHT   = auto()
    SLOW_DOWN     = auto()
    STOP          = auto()

    def __str__(self):
        return self.name.replace("_", " ")


@dataclass
class AvoidanceDecision:
    action:        AvoidanceAction
    reason:        str
    steer_override: float   # Suggested steering -1.0 … +1.0
    throttle_scale: float   # Multiplier for driver throttle (0.0 – 1.0)


def decide(
    report: ObstacleReport,
    driver_throttle: float,
    driver_steering: float,
) -> AvoidanceDecision:
    """
    Core decision function.

    Priority:
      1. All zones blocked → STOP
      2. Center blocked, side clear → DODGE to clear side
      3. Center partially blocked → SLOW_DOWN
      4. Distance warning → SLOW_DOWN
      5. Path clear → FOLLOW_DRIVER

    Parameters
    ----------
    report          : ObstacleReport from vision pipeline
    driver_throttle : normalized -1…+1 from RC receiver
    driver_steering : normalized -1…+1 from RC receiver

    Returns
    -------
    AvoidanceDecision
    """
    ncfg = CFG.navigation
    vcfg = CFG.vision

    # ── 1. No obstacle / not moving forward ─────────────────────────────────
    # If driver isn't commanding forward (or is reversing), don't interfere
    is_moving_forward = driver_throttle > 0.10
    if not is_moving_forward:
        return AvoidanceDecision(
            action         = AvoidanceAction.FOLLOW_DRIVER,
            reason         = "Driver not moving forward",
            steer_override = driver_steering,
            throttle_scale = 1.0,
        )

    # ── 2. All zones blocked → STOP ──────────────────────────────────────────
    if report.all_blocked:
        return AvoidanceDecision(
            action         = AvoidanceAction.STOP,
            reason         = "All paths blocked",
            steer_override = 0.0,
            throttle_scale = 0.0,
        )

    # ── 3. Path clear ─────────────────────────────────────────────────────────
    if not report.danger and not report.warning:
        return AvoidanceDecision(
            action         = AvoidanceAction.FOLLOW_DRIVER,
            reason         = "Path clear",
            steer_override = driver_steering,
            throttle_scale = 1.0,
        )

    zones = report.zones

    # ── 4. Center blocked → try to dodge ────────────────────────────────────
    if zones.center == ZoneStatus.BLOCKED or report.danger:
        safe_side = zones.safest_side()

        if safe_side == "left":
            return AvoidanceDecision(
                action         = AvoidanceAction.DODGE_LEFT,
                reason         = "Obstacle in center — left clear",
                steer_override = -ncfg.dodge_steer_amount,
                throttle_scale = ncfg.caution_speed_factor,
            )
        elif safe_side == "right":
            return AvoidanceDecision(
                action         = AvoidanceAction.DODGE_RIGHT,
                reason         = "Obstacle in center — right clear",
                steer_override = +ncfg.dodge_steer_amount,
                throttle_scale = ncfg.caution_speed_factor,
            )
        else:
            return AvoidanceDecision(
                action         = AvoidanceAction.STOP,
                reason         = "Center blocked, both sides blocked",
                steer_override = 0.0,
                throttle_scale = 0.0,
            )

    # ── 5. Partial blockage or distance warning → slow down ──────────────────
    if zones.center == ZoneStatus.PARTIAL or report.warning:
        return AvoidanceDecision(
            action         = AvoidanceAction.SLOW_DOWN,
            reason         = "Partial blockage or obstacle approaching",
            steer_override = driver_steering,
            throttle_scale = ncfg.caution_speed_factor,
        )

    # ── Default ───────────────────────────────────────────────────────────────
    return AvoidanceDecision(
        action         = AvoidanceAction.FOLLOW_DRIVER,
        reason         = "Path appears clear",
        steer_override = driver_steering,
        throttle_scale = 1.0,
    )
