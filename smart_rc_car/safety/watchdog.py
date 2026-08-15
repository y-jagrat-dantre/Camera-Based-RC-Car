"""
safety/watchdog.py
==================
Software watchdog timer.

The main control loop must call watchdog.kick() at least once per
timeout interval. If it fails to do so (e.g. the loop hangs or
crashes), the watchdog fires ESTOP.

Runs in a dedicated daemon thread so it is immune to the main loop blocking.

Usage:
    watchdog = Watchdog(timeout_s=1.0)
    watchdog.start()
    # In main loop:
    watchdog.kick()
    # On shutdown:
    watchdog.stop()
"""
from __future__ import annotations

import logging
import threading
import time

from smart_rc_car.config.settings import CFG
from smart_rc_car.safety.emergency_stop import ESTOP, StopReason

log = logging.getLogger(__name__)


class Watchdog:
    """
    Software watchdog that triggers ESTOP if not kicked within timeout.

    Parameters
    ----------
    timeout_s : float
        Maximum allowed time between kicks. Defaults to config value.
    """

    def __init__(self, timeout_s: float | None = None):
        self._timeout = timeout_s or CFG.safety.watchdog_timeout_s
        self._last_kick = time.monotonic()
        self._running   = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def start(self):
        self._running   = True
        self._last_kick = time.monotonic()
        self._thread = threading.Thread(
            target=self._monitor_loop,
            name="watchdog",
            daemon=True,
        )
        self._thread.start()
        log.info(f"Watchdog started (timeout={self._timeout:.2f}s)")

    def stop(self):
        self._running = False
        log.info("Watchdog stopped.")

    def kick(self):
        """Reset the watchdog timer. Call this from the main control loop."""
        with self._lock:
            self._last_kick = time.monotonic()

    def _monitor_loop(self):
        check_interval = min(0.1, self._timeout / 4)
        while self._running:
            time.sleep(check_interval)
            with self._lock:
                age = time.monotonic() - self._last_kick
            if age > self._timeout:
                log.critical(
                    f"Watchdog expired! No kick for {age:.2f}s "
                    f"(limit {self._timeout:.2f}s)"
                )
                ESTOP.trigger(StopReason.WATCHDOG_EXPIRED)
                # Continue monitoring in case ESTOP is cleared and
                # the loop recovers.
                with self._lock:
                    self._last_kick = time.monotonic()
