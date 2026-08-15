"""
control/pid.py
==============
Generic PID controller with anti-windup and output clamping.

Used by steering and throttle to produce smooth, stable corrections.

Example:
    pid = PIDController(kp=0.8, ki=0.05, kd=0.1, output_min=-1.0, output_max=1.0)
    correction = pid.update(setpoint=0.0, measured=current_heading, dt=0.033)
"""
from __future__ import annotations

import time


class PIDController:
    """
    Discrete PID controller with:
    - Anti-windup clamp (integral limited to output range)
    - Derivative on measurement (not error) — avoids derivative kick on setpoint change
    - Output clamping
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float = -1.0,
        output_max: float = 1.0,
        integral_limit: float | None = None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit or (output_max - output_min)

        self._integral: float = 0.0
        self._prev_measurement: float | None = None
        self._prev_time: float = time.monotonic()

    def update(
        self,
        setpoint: float,
        measured: float,
        dt: float | None = None,
    ) -> float:
        """
        Compute PID output.

        Parameters
        ----------
        setpoint  : desired value
        measured  : current measured value
        dt        : time delta in seconds (auto-computed if None)

        Returns
        -------
        float     : control output clamped to [output_min, output_max]
        """
        now = time.monotonic()
        if dt is None:
            dt = now - self._prev_time
        self._prev_time = now

        if dt <= 0:
            dt = 1e-6

        error = setpoint - measured

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(
            -self.integral_limit,
            min(self.integral_limit, self._integral),
        )
        i_term = self.ki * self._integral

        # Derivative on measurement (avoids kick on setpoint change)
        if self._prev_measurement is None:
            d_term = 0.0
        else:
            d_term = -self.kd * (measured - self._prev_measurement) / dt
        self._prev_measurement = measured

        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))

    def reset(self):
        """Reset integrator and derivative state."""
        self._integral = 0.0
        self._prev_measurement = None
        self._prev_time = time.monotonic()
