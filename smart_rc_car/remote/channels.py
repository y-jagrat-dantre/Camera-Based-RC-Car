"""
remote/channels.py
==================
Channel constants, PWM-to-normalized-value mapping, and deadband filtering.

The FS-R6B receiver outputs standard RC PWM:
  1000 µs = full negative / minimum
  1500 µs = center / neutral
  2000 µs = full positive / maximum
"""
from __future__ import annotations

from dataclasses import dataclass

from smart_rc_car.config.settings import CFG

# ── Channel names ────────────────────────────────────────────────────────────
CH_STEERING = "ch1_steering"
CH_THROTTLE  = "ch2_throttle"
CH_MODE      = "ch3_mode"

# ── Operating modes ──────────────────────────────────────────────────────────
MODE_MANUAL     = "MANUAL"
MODE_AUTO_ASSIST = "AUTO_ASSIST"


@dataclass
class ChannelValues:
    """Normalized channel values in range -1.0 … +1.0 (0.0 = center)."""
    steering: float = 0.0    # -1 = full left, +1 = full right
    throttle: float = 0.0    # -1 = full reverse, +1 = full forward
    mode_raw_us: int = 1000  # raw µs for CH3
    valid: bool = False       # False if signal is lost

    @property
    def mode(self) -> str:
        if not self.valid:
            return MODE_MANUAL  # Failsafe: always manual when signal is lost
        if self.mode_raw_us >= CFG.receiver.mode_switch_threshold_us:
            return MODE_AUTO_ASSIST
        return MODE_MANUAL


def pwm_to_normalized(pulse_us: int) -> float:
    """
    Map a raw PWM pulse width (µs) to a normalized float in [-1.0, +1.0].

    Applies deadband around the center position so tiny joystick drift
    does not produce small motor commands.
    """
    cfg = CFG.receiver
    center   = cfg.pwm_center_us
    half_rng = (cfg.pwm_max_us - cfg.pwm_min_us) / 2.0
    db       = cfg.deadband_us

    offset = pulse_us - center

    # Apply deadband
    if abs(offset) <= db:
        return 0.0

    # Shrink by deadband so value starts at 0 after dead-zone
    if offset > 0:
        offset -= db
    else:
        offset += db

    normalized = offset / (half_rng - db)
    return max(-1.0, min(1.0, normalized))


def is_valid_pwm(pulse_us: int) -> bool:
    """Return True if the pulse width is within expected RC PWM range."""
    return 800 <= pulse_us <= 2200
