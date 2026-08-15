"""
motors/throttle.py
==================
Throttle management with acceleration/deceleration ramp limiting.

Prevents sudden speed jumps that could tip the car or cause
mechanical stress. The ramp rate is configured in config.yaml.
"""
from __future__ import annotations

from smart_rc_car.config.settings import CFG
from smart_rc_car.control.smoothing import RateLimiter


class ThrottleController:
    """
    Accepts a target throttle value and outputs a ramped throttle value.

    The ramp_rate in config limits how fast throttle changes per control cycle.
    """

    def __init__(self):
        self._limiter = RateLimiter(rate=CFG.motors.ramp_rate)

    def update(self, target: float) -> float:
        """
        Apply ramp limiting to the target throttle.

        Parameters
        ----------
        target : float   Desired throttle in [-1.0, +1.0]

        Returns
        -------
        float  Ramped throttle in [-1.0, +1.0]
        """
        return self._limiter.update(max(-1.0, min(1.0, target)))

    def reset(self):
        """Reset ramp state (e.g. after emergency stop)."""
        self._limiter.reset(0.0)

    @property
    def current(self) -> float:
        return self._limiter.value
