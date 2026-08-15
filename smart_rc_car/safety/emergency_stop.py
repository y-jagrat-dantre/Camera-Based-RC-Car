"""
safety/emergency_stop.py
========================
Global emergency stop singleton.

Any module in the system can call EmergencyStop.trigger(reason)
to immediately halt all motors and set a system-wide flag.

This module has NO imports from other smart_rc_car modules to avoid
circular dependencies — it is the lowest-level safety primitive.
"""
from __future__ import annotations

import logging
import threading
from enum import Enum, auto
from typing import Callable, Optional

log = logging.getLogger(__name__)


class StopReason(Enum):
    SIGNAL_LOST         = auto()
    WATCHDOG_EXPIRED    = auto()
    CRITICAL_OBSTACLE   = auto()
    SOFTWARE_ERROR      = auto()
    MANUAL_TRIGGER      = auto()
    MOTOR_FAULT         = auto()


class _EmergencyStop:
    """
    Thread-safe emergency stop controller.

    Usage
    -----
    Import the module-level singleton:
        from smart_rc_car.safety.emergency_stop import ESTOP

    Trigger:
        ESTOP.trigger(StopReason.SIGNAL_LOST)

    Check:
        if ESTOP.active: ...

    Register motor stop callback:
        ESTOP.register_stop_callback(motor_driver.stop_all)

    Clear (re-arm):
        ESTOP.clear()
    """

    def __init__(self):
        self._active = False
        self._reason: Optional[StopReason] = None
        self._lock   = threading.Lock()
        self._callbacks: list[Callable[[], None]] = []

    def register_stop_callback(self, fn: Callable[[], None]):
        """Register a function to be called on emergency stop."""
        with self._lock:
            self._callbacks.append(fn)

    def trigger(self, reason: StopReason = StopReason.MANUAL_TRIGGER):
        with self._lock:
            if not self._active:
                self._active = True
                self._reason = reason
                log.critical(f"EMERGENCY STOP triggered: {reason.name}")
                for fn in self._callbacks:
                    try:
                        fn()
                    except Exception as e:
                        log.error(f"Stop callback failed: {e}")

    def clear(self):
        """Re-arm the system after a stop (only if safe to do so)."""
        with self._lock:
            self._active = False
            self._reason = None
            log.info("Emergency stop cleared — system re-armed.")

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    @property
    def reason(self) -> Optional[StopReason]:
        with self._lock:
            return self._reason


# Module-level singleton
ESTOP = _EmergencyStop()
