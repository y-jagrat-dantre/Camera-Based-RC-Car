"""
navigation/ramp_controller.py
==============================
Ramp-specific navigation assistance.

When a ramp is detected:
  - Suppress lateral steering corrections (keep car straight)
  - Ensure sufficient throttle is maintained (prevent stall)
  - Detect when the flat surface resumes and release control

The ramp controller does NOT drive the car — it only constrains
the driver's command to keep the car aligned on the ramp.
"""
from __future__ import annotations

import logging

from smart_rc_car.vision.ramp_detector import RampInfo

log = logging.getLogger(__name__)

# Minimum throttle maintained while on ramp (prevents stall)
RAMP_MIN_THROTTLE = 0.35

# Maximum lateral steering allowed on ramp (keeps car aligned)
RAMP_MAX_STEER = 0.15


class RampController:
    """
    Applies ramp-specific constraints to driver commands.
    """

    def __init__(self):
        self._on_ramp = False

    def apply(
        self,
        throttle: float,
        steering: float,
        ramp_info: RampInfo,
    ) -> tuple[float, float]:
        """
        Apply ramp constraints if a ramp is detected.

        Parameters
        ----------
        throttle   : driver's normalized throttle command
        steering   : driver's normalized steering command
        ramp_info  : latest RampInfo from ramp_detector

        Returns
        -------
        (throttle, steering) potentially modified
        """
        if ramp_info.detected:
            if not self._on_ramp:
                log.info(
                    f"Ramp detected ({ramp_info.angle_deg:.1f}°) — "
                    "ramp controller active."
                )
                self._on_ramp = True

            # Ensure we have enough throttle to climb
            if throttle > 0:   # Only when driver wants to go forward
                throttle = max(RAMP_MIN_THROTTLE, throttle)

            # Suppress aggressive lateral steering — car must stay aligned
            steering = max(-RAMP_MAX_STEER, min(RAMP_MAX_STEER, steering))

        else:
            if self._on_ramp:
                log.info("Ramp cleared — ramp controller released.")
                self._on_ramp = False

        return throttle, steering

    @property
    def on_ramp(self) -> bool:
        return self._on_ramp
