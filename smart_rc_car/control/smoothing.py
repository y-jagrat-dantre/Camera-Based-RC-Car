"""
control/smoothing.py
====================
Signal smoothing utilities:

  RateLimiter     — limits rate-of-change per update (prevents jerky motion)
  ExponentialFilter — low-pass filter for noisy values
  DeadbandFilter  — ignores small signals below a threshold
"""
from __future__ import annotations


class RateLimiter:
    """
    Clamps how fast an output value can change per call.

    Example:
        limiter = RateLimiter(rate=0.05)
        # If current is 0.0 and target is 1.0, output increases by 0.05 per call.
        smooth_val = limiter.update(target_value)
    """

    def __init__(self, rate: float):
        """
        Parameters
        ----------
        rate : float
            Maximum change per update step (absolute units).
            e.g. 0.05 means value can change by at most ±0.05 per call.
        """
        self._rate = rate
        self._value: float | None = None

    def update(self, target: float) -> float:
        if self._value is None:
            self._value = target
            return self._value

        delta = target - self._value
        delta = max(-self._rate, min(self._rate, delta))
        self._value += delta
        return self._value

    def reset(self, value: float = 0.0):
        self._value = value

    @property
    def value(self) -> float:
        return self._value if self._value is not None else 0.0


class ExponentialFilter:
    """
    Exponential moving average (low-pass filter).

    output = alpha * new_value + (1 - alpha) * previous_output

    alpha=1.0 → no filtering (passthrough)
    alpha=0.1 → heavy filtering (very smooth, slow to respond)
    """

    def __init__(self, alpha: float = 0.2):
        if not (0.0 < alpha <= 1.0):
            raise ValueError("alpha must be in (0, 1]")
        self._alpha = alpha
        self._value: float | None = None

    def update(self, new_value: float) -> float:
        if self._value is None:
            self._value = new_value
        else:
            self._value = self._alpha * new_value + (1.0 - self._alpha) * self._value
        return self._value

    def reset(self, value: float = 0.0):
        self._value = value

    @property
    def value(self) -> float:
        return self._value if self._value is not None else 0.0


class DeadbandFilter:
    """
    Passes through values that exceed the deadband threshold.
    Values within ±threshold of zero are clamped to zero.

    Example:
        f = DeadbandFilter(threshold=0.05)
        f.apply(0.03)  # → 0.0
        f.apply(0.10)  # → 0.10
    """

    def __init__(self, threshold: float):
        self._threshold = threshold

    def apply(self, value: float) -> float:
        return 0.0 if abs(value) < self._threshold else value
