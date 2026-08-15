"""
navigation/safety_controller.py
================================
Top-level safety and command pipeline.

Implements the full decision priority stack:

  Priority 1: Emergency Stop        → motors = 0, immediately
  Priority 2: Signal Lost           → emergency stop
  Priority 3: Mode = MANUAL         → pass driver commands directly
  Priority 4: Auto-Assist           → apply obstacle avoidance + ramp control
  Priority 5: Throttle smoothing    → apply ramp limiting to output

This is the single entry point for the main loop to get motor commands.

Usage:
    controller = SafetyController(motor_driver)
    # In loop:
    command = controller.update(channel_values, obstacle_report)
    # command.left / command.right are sent to the motor driver
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

from smart_rc_car.config.settings import CFG
from smart_rc_car.motors.motor_driver import MotorDriverBase
from smart_rc_car.motors.steering import WheelSpeeds, differential_mix
from smart_rc_car.motors.throttle import ThrottleController
from smart_rc_car.navigation.dodge_controller import DodgeController, DodgeState
from smart_rc_car.navigation.obstacle_avoidance import AvoidanceAction, decide
from smart_rc_car.navigation.ramp_controller import RampController
from smart_rc_car.remote.channels import ChannelValues, MODE_AUTO_ASSIST
from smart_rc_car.safety.emergency_stop import ESTOP, StopReason
from smart_rc_car.vision.obstacle_detector import ObstacleReport

log = logging.getLogger(__name__)


@dataclass
class ControlOutput:
    """What was actually sent to the motors this cycle."""
    left:         float   = 0.0
    right:        float   = 0.0
    throttle:     float   = 0.0
    steering:     float   = 0.0
    mode:         str     = "MANUAL"
    ai_action:    str     = "FOLLOWING DRIVER"
    ai_overriding: bool   = False


class SafetyController:
    """
    Priority-ordered command arbiter + motor output pipeline.
    """

    def __init__(self, motor_driver: MotorDriverBase):
        self._motors      = motor_driver
        self._dodge       = DodgeController()
        self._ramp        = RampController()
        self._throttle_sm = ThrottleController()

    def update(
        self,
        channels: ChannelValues,
        report: ObstacleReport | None = None,
        dt: float = 0.033,
    ) -> ControlOutput:
        """
        Run the priority stack and send commands to motors.

        Parameters
        ----------
        channels : ChannelValues  Latest RC receiver data
        report   : ObstacleReport or None (None in manual mode / no camera)
        dt       : Control loop period in seconds

        Returns
        -------
        ControlOutput  What was actually commanded
        """
        out = ControlOutput()

        # ── Priority 1 & 2: Emergency stop / signal lost ──────────────────────
        if ESTOP.active:
            self._motors.stop_all()
            self._throttle_sm.reset()
            out.ai_action = f"EMERGENCY STOP: {ESTOP.reason}"
            return out

        if not channels.valid:
            ESTOP.trigger(StopReason.SIGNAL_LOST)
            self._motors.stop_all()
            out.ai_action = "SIGNAL LOST — STOPPED"
            return out

        # ── Raw driver commands ────────────────────────────────────────────────
        driver_throttle = channels.throttle
        driver_steering = channels.steering
        mode            = channels.mode
        out.mode        = mode

        # ── Priority 3: Manual mode — pure passthrough ─────────────────────────
        if mode != MODE_AUTO_ASSIST:
            final_throttle = self._throttle_sm.update(driver_throttle)
            final_steering = driver_steering
            wheels = differential_mix(final_throttle, final_steering)
            self._motors.set_speeds(wheels.left, wheels.right)
            out.throttle   = final_throttle
            out.steering   = final_steering
            out.left       = wheels.left
            out.right      = wheels.right
            out.ai_action  = "MANUAL — FOLLOWING DRIVER"
            return out

        # ── Priority 4: Auto-Assist ────────────────────────────────────────────
        if report is None:
            # No vision data — fall through to driver command
            decision_action = AvoidanceAction.FOLLOW_DRIVER
            ai_steer   = driver_steering
            throt_scale = 1.0
            out.ai_action = "AUTO (NO VISION) — FOLLOWING DRIVER"
        else:
            # Get avoidance decision
            decision = decide(report, driver_throttle, driver_steering)

            # Dodge state machine produces smooth steering + throttle scale
            ai_steer, throt_scale = self._dodge.update(
                decision, driver_steering, dt
            )
            decision_action = decision.action
            out.ai_overriding = self._dodge.is_overriding
            out.ai_action     = str(decision_action)

        # ── Priority 5: Ramp safety ────────────────────────────────────────────
        ramp_info = report.ramp if report else None
        if ramp_info and ramp_info.detected:
            throttle_for_ramp = driver_throttle * throt_scale
            throttle_for_ramp, ai_steer = self._ramp.apply(
                throttle_for_ramp, ai_steer, ramp_info
            )
            final_throttle = self._throttle_sm.update(throttle_for_ramp)
            out.ai_action = "RAMP — ALIGNED CLIMBING"
        else:
            final_throttle = self._throttle_sm.update(
                driver_throttle * throt_scale
            )

        final_steering = ai_steer

        # Apply emergency stop check
        if decision_action == AvoidanceAction.STOP:
            final_throttle = 0.0
            final_steering = 0.0

        # ── Output to motors ───────────────────────────────────────────────────
        wheels = differential_mix(final_throttle, final_steering)
        self._motors.set_speeds(wheels.left, wheels.right)

        out.throttle = final_throttle
        out.steering = final_steering
        out.left     = wheels.left
        out.right    = wheels.right

        return out
