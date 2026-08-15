"""
remote/flysky_receiver.py
=========================
Reads the FlySky FS-R6B receiver using individual PWM channels via lgpio.

The FS-R6B outputs standard servo PWM on each channel pin:
  • Pulse high = 1000–2000 µs
  • Period     ≈ 20 ms (50 Hz)

lgpio's callback mechanism reads kernel-timestamped edges with ns precision
without busy-polling, which is crucial for real-time performance on Pi 5.

Usage:
    receiver = FlySkyReceiver()
    receiver.start()
    values = receiver.read()
    receiver.stop()

Standalone test:
    python -m smart_rc_car.remote.flysky_receiver
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

try:
    import lgpio
    LGPIO_AVAILABLE = True
except ImportError:
    LGPIO_AVAILABLE = False

from smart_rc_car.config.settings import CFG
from smart_rc_car.remote.channels import (
    ChannelValues,
    is_valid_pwm,
    pwm_to_normalized,
)

log = logging.getLogger(__name__)

def _get_gpiochip() -> int:
    """Finds the correct GPIO chip. Pi 5 uses 4, Pi 4 uses 0."""
    if not LGPIO_AVAILABLE:
        return 0
    for chip in (4, 0):
        try:
            h = lgpio.gpiochip_open(chip)
            lgpio.gpiochip_close(h)
            return chip
        except Exception:
            pass
    return 0

# ── Simulated channel values for development on a non-Pi machine ─────────────
class _SimulatedReceiver:
    """Returns neutral values when lgpio is unavailable (dev machine)."""
    def read(self) -> ChannelValues:
        return ChannelValues(steering=0.0, throttle=0.0,
                             mode_raw_us=1000, valid=True)


class _PwmCallback:
    """Tracks rising/falling edges on one GPIO to measure pulse width."""

    def __init__(self, h: int, gpio: int):
        self._h      = h
        self._gpio   = gpio
        self._tick   = 0      # ns timestamp of rising edge
        self._pulse  = 1500   # last measured pulse width µs (default center)
        self._last_t = time.monotonic()
        self._lock   = threading.Lock()

        lgpio.gpio_claim_alert(h, gpio, lgpio.BOTH_EDGES)
        self._cb = lgpio.callback(h, gpio, lgpio.BOTH_EDGES, self._edge)

    def _edge(self, chip, gpio, level, tick):
        if level == 1:           # rising edge
            self._tick = tick
        elif level == 0:         # falling edge
            if self._tick:
                pulse_ns = tick - self._tick
                if pulse_ns < 0:
                    pulse_ns += (1 << 64)
                pulse = int(pulse_ns / 1000.0) # ns to µs
                if is_valid_pwm(pulse):
                    with self._lock:
                        self._pulse  = pulse
                        self._last_t = time.monotonic()

    @property
    def pulse_us(self) -> int:
        with self._lock:
            return self._pulse

    @property
    def age_s(self) -> float:
        with self._lock:
            return time.monotonic() - self._last_t

    def cancel(self):
        self._cb.cancel()


class FlySkyReceiver:
    """
    Reads CH1 (steering), CH2 (throttle), CH3 (mode) from the FS-R6B.

    Attributes
    ----------
    signal_lost : bool
        True when no valid pulse received within the timeout window.
    """

    def __init__(self):
        self._h = None
        self._callbacks: dict[str, _PwmCallback] = {}
        self._lock = threading.Lock()
        self._running = False

        if not LGPIO_AVAILABLE:
            log.warning("lgpio not available — using simulated receiver.")
            self._simulated = True
        else:
            self._simulated = False

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Connect to gpiochip and register GPIO callbacks."""
        if self._simulated:
            return
            
        try:
            chip = _get_gpiochip()
            self._h = lgpio.gpiochip_open(chip)
        except Exception as e:
            log.error(f"Could not open gpiochip: {e}")
            self._simulated = True
            return

        cfg = CFG.receiver.channels
        pin_map = {
            "ch1_steering": cfg.ch1_steering,
            "ch2_throttle": cfg.ch2_throttle,
            "ch3_mode":     cfg.ch3_mode,
        }

        for name, gpio in pin_map.items():
            if gpio is not None:
                self._callbacks[name] = _PwmCallback(self._h, gpio)
                log.info(f"Monitoring {name} on GPIO {gpio} (chip {chip})")

        self._running = True
        log.info("FlySky receiver started.")

    def stop(self):
        """Cancel callbacks and disconnect from lgpio."""
        self._running = False
        for cb in self._callbacks.values():
            cb.cancel()
        if self._h is not None:
            lgpio.gpiochip_close(self._h)
            self._h = None
        log.info("FlySky receiver stopped.")

    # ── Read ─────────────────────────────────────────────────────────────────

    def read(self) -> ChannelValues:
        """
        Return the latest channel values.
        Returns invalid ChannelValues if signal is lost.
        """
        if self._simulated:
            return ChannelValues(steering=0.0, throttle=0.0,
                                 mode_raw_us=1000, valid=True)

        timeout = CFG.receiver.signal_loss_timeout_s

        # Check signal validity based on age of last pulse
        ch1_cb = self._callbacks.get("ch1_steering")
        ch2_cb = self._callbacks.get("ch2_throttle")
        ch3_cb = self._callbacks.get("ch3_mode")

        if ch1_cb is None or ch2_cb is None:
            return ChannelValues(valid=False)

        signal_valid = (
            ch1_cb.age_s < timeout and
            ch2_cb.age_s < timeout
        )

        if not signal_valid:
            return ChannelValues(valid=False)

        steering_us = ch1_cb.pulse_us
        throttle_us = ch2_cb.pulse_us
        mode_us     = ch3_cb.pulse_us if ch3_cb else 1000

        return ChannelValues(
            steering    = pwm_to_normalized(steering_us),
            throttle    = pwm_to_normalized(throttle_us),
            mode_raw_us = mode_us,
            valid       = True,
        )

    @property
    def signal_lost(self) -> bool:
        return not self.read().valid


# ── Standalone test ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    logging.basicConfig(level=logging.DEBUG,
                        format="%(levelname)s  %(name)s: %(message)s")
    log.info("Starting FlySky receiver test. Press Ctrl+C to stop.")
    rx = FlySkyReceiver()
    rx.start()
    try:
        while True:
            v = rx.read()
            print(
                f"  Steering: {v.steering:+.3f}  "
                f"Throttle: {v.throttle:+.3f}  "
                f"Mode: {v.mode}  "
                f"Valid: {v.valid}",
                end="\r",
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        rx.stop()
