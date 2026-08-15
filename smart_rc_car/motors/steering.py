"""
motors/steering.py
==================
Converts a normalized steering value + throttle into
left / right motor speed commands for a differential-drive chassis.

Differential (tank) mixing:
  - Pure forward:  L=+T, R=+T
  - Pure right:    L=+T, R=-T (pivot)
  - Mixed:         L = throttle + steering, R = throttle - steering

The mix is then normalized so neither channel exceeds 1.0.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class WheelSpeeds:
    left: float    # -1.0 … +1.0
    right: float   # -1.0 … +1.0


def differential_mix(throttle: float, steering: float) -> WheelSpeeds:
    """
    Convert throttle + steering (both in [-1, +1]) to wheel speeds.

    Parameters
    ----------
    throttle : float
        +1 = full forward, -1 = full reverse
    steering : float
        +1 = full right turn, -1 = full left turn

    Returns
    -------
    WheelSpeeds
        Normalized left and right speeds.
    """
    left  = throttle + steering
    right = throttle - steering

    # Normalize if any channel exceeds ±1.0
    max_val = max(abs(left), abs(right), 1.0)
    left  /= max_val
    right /= max_val

    return WheelSpeeds(left=left, right=right)


def blend_steering(
    driver_steering: float,
    ai_steering: float,
    ai_weight: float,
) -> float:
    """
    Blend driver steering with AI override steering.

    Parameters
    ----------
    driver_steering : float  Driver's raw steering command
    ai_steering     : float  AI's desired steering correction
    ai_weight       : float  0.0 = full driver, 1.0 = full AI

    Returns
    -------
    float   Blended steering value in [-1, +1]
    """
    ai_weight = max(0.0, min(1.0, ai_weight))
    return (1.0 - ai_weight) * driver_steering + ai_weight * ai_steering
