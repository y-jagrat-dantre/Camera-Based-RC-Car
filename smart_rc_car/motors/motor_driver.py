"""
motors/motor_driver.py
======================
Abstract motor driver base class and L298N concrete implementation.

The L298N controls two motors:
  Motor A (left):  ENA (PWM speed), IN1/IN2 (direction)
  Motor B (right): ENB (PWM speed), IN3/IN4 (direction)

Speed values are normalized floats:
  +1.0 = full forward
   0.0 = stop
  -1.0 = full reverse

The driver uses pigpio for hardware PWM on Pi 5 (required because
RPi.GPIO is NOT compatible with Pi 5's RP1 GPIO controller).

Standalone test:
    python -m smart_rc_car.motors.motor_driver
"""
from __future__ import annotations

import abc
import logging
import time
from typing import Optional

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False

from smart_rc_car.config.settings import CFG
from smart_rc_car.safety.emergency_stop import ESTOP, StopReason

log = logging.getLogger(__name__)


class MotorDriverBase(abc.ABC):
    """Abstract interface that all motor driver implementations must satisfy."""

    @abc.abstractmethod
    def set_speeds(self, left: float, right: float): ...

    @abc.abstractmethod
    def stop_all(self): ...

    @abc.abstractmethod
    def shutdown(self): ...


class L298NDriver(MotorDriverBase):
    """
    L298N dual H-bridge motor driver via pigpio.

    Pin assignments are read from config.yaml.

    Parameters
    ----------
    simulate : bool
        If True, only log commands without touching GPIO (for dev machines).
    """

    def __init__(self, simulate: bool = False):
        self._simulate = simulate or not PIGPIO_AVAILABLE
        self._pi: Optional["pigpio.pi"] = None

        cfg_m = CFG.motors
        self._pwm_freq   = cfg_m.pwm_frequency
        self._max_dc     = cfg_m.max_duty_cycle
        self._min_dc     = cfg_m.min_duty_cycle

        # Left motor pins
        self._ena = cfg_m.left.ena_pin
        self._in1 = cfg_m.left.in1_pin
        self._in2 = cfg_m.left.in2_pin

        # Right motor pins
        self._enb = cfg_m.right.enb_pin
        self._in3 = cfg_m.right.in3_pin
        self._in4 = cfg_m.right.in4_pin

        self._left_speed  = 0.0
        self._right_speed = 0.0

        if self._simulate:
            log.warning("L298N running in SIMULATION mode — no GPIO.")
        else:
            self._connect()

        # Register emergency stop callback
        ESTOP.register_stop_callback(self.stop_all)

    def _connect(self):
        self._pi = pigpio.pi()
        if not self._pi.connected:
            log.error("Cannot connect to pigpio daemon.")
            self._simulate = True
            return

        # Set direction pins as outputs
        for pin in (self._in1, self._in2, self._in3, self._in4):
            self._pi.set_mode(pin, pigpio.OUTPUT)
            self._pi.write(pin, 0)

        # Set PWM pins
        for pin in (self._ena, self._enb):
            self._pi.set_mode(pin, pigpio.OUTPUT)
            self._pi.set_PWM_frequency(pin, self._pwm_freq)
            self._pi.set_PWM_dutycycle(pin, 0)

        log.info("L298N motor driver initialized.")

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _duty(self, speed: float) -> int:
        """Convert normalized speed [-1, +1] to pigpio dutycycle [0, 255]."""
        abs_speed = abs(speed)
        if abs_speed < 0.01:
            return 0
        # Map from [min_dc, max_dc] to [0, 255]
        clamped = self._min_dc + abs_speed * (self._max_dc - self._min_dc)
        clamped = min(self._max_dc, clamped)
        return int(clamped * 255)

    def _apply_motor(self, in_a: int, in_b: int, en: int, speed: float):
        if self._simulate:
            return
        if speed > 0.01:
            self._pi.write(in_a, 1)
            self._pi.write(in_b, 0)
        elif speed < -0.01:
            self._pi.write(in_a, 0)
            self._pi.write(in_b, 1)
        else:
            self._pi.write(in_a, 0)
            self._pi.write(in_b, 0)
        self._pi.set_PWM_dutycycle(en, self._duty(speed))

    # ── Public API ────────────────────────────────────────────────────────────

    def set_speeds(self, left: float, right: float):
        """
        Set left and right motor speeds.

        Parameters
        ----------
        left  : float  -1.0 (full reverse) … +1.0 (full forward)
        right : float  -1.0 (full reverse) … +1.0 (full forward)
        """
        if ESTOP.active:
            self.stop_all()
            return

        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        self._left_speed  = left
        self._right_speed = right

        if self._simulate:
            log.debug(f"[SIM] Motors → L:{left:+.3f}  R:{right:+.3f}")
            return

        self._apply_motor(self._in1, self._in2, self._ena, left)
        self._apply_motor(self._in3, self._in4, self._enb, right)

    def stop_all(self):
        """Immediately stop both motors (coast, not brake)."""
        if self._simulate:
            log.debug("[SIM] Motors → STOP")
            return
        if self._pi and self._pi.connected:
            for pin in (self._in1, self._in2, self._in3, self._in4):
                self._pi.write(pin, 0)
            for pin in (self._ena, self._enb):
                self._pi.set_PWM_dutycycle(pin, 0)

    def shutdown(self):
        """Stop motors and release pigpio resources."""
        self.stop_all()
        if self._pi and self._pi.connected:
            self._pi.stop()
        log.info("L298N driver shut down.")

    @property
    def left_speed(self) -> float:
        return self._left_speed

    @property
    def right_speed(self) -> float:
        return self._right_speed


def create_motor_driver() -> MotorDriverBase:
    """Factory: returns the appropriate driver based on config."""
    driver_name = CFG.motors.driver.lower()
    if driver_name == "l298n":
        return L298NDriver()
    raise ValueError(f"Unknown motor driver: {driver_name}")


# ── Standalone test ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG,
                        format="%(levelname)s  %(name)s: %(message)s")
    driver = create_motor_driver()
    print("Motor test sequence. Ensure the car is elevated/safe!")
    try:
        print("Forward...")
        driver.set_speeds(0.5, 0.5)
        time.sleep(1.5)

        print("Spin left...")
        driver.set_speeds(-0.4, 0.4)
        time.sleep(1.0)

        print("Spin right...")
        driver.set_speeds(0.4, -0.4)
        time.sleep(1.0)

        print("Reverse...")
        driver.set_speeds(-0.5, -0.5)
        time.sleep(1.5)
    finally:
        driver.shutdown()
        print("Done.")
