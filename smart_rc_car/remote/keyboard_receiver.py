"""
remote/keyboard_receiver.py
===========================
Keyboard-controlled simulated RC receiver for PC development.

Simulates the FlySky CT6B joystick using keyboard keys:

  W / S        → Throttle forward / reverse
  A / D        → Steer left / right
  M            → Toggle Manual / Auto-Assist mode
  SPACE        → Brake (throttle = 0)
  ESC / Q      → Emergency stop

Inputs are smooth — holding a key ramps up the value;
releasing ramps it back to center.

Uses `pynput` for non-blocking keyboard capture that works
alongside the OpenCV window.

Usage (imported by simulate.py):
    rx = KeyboardReceiver()
    rx.start()
    values = rx.read()    # Same interface as FlySkyReceiver
    rx.stop()
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field

from smart_rc_car.remote.channels import ChannelValues, MODE_AUTO_ASSIST, MODE_MANUAL

log = logging.getLogger(__name__)

try:
    from pynput import keyboard as _kb
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False

# Ramp speed per update call (~30Hz assumed)
RAMP_UP   = 0.08   # How fast value increases while key held
RAMP_DOWN = 0.10   # How fast value returns to center when released


class KeyboardReceiver:
    """
    Simulates the FlySky receiver via keyboard for PC testing.

    Drop-in replacement for FlySkyReceiver — same .start()/.read()/.stop() API.
    """

    def __init__(self):
        self._steering  = 0.0
        self._throttle  = 0.0
        self._mode_us   = 1000   # 1000 = MANUAL, 2000 = AUTO-ASSIST
        self._brake     = False
        self._estop     = False
        self._lock      = threading.Lock()
        self._running   = False
        self._listener  = None
        self._pressed: set[str] = set()
        self._cv2_keys_last_seen = {}

        # Ramp update thread
        self._ramp_thread: threading.Thread | None = None

    def start(self):
        """Start keyboard listener and ramp thread."""
        self._running = True

        if PYNPUT_AVAILABLE:
            self._listener = _kb.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
                suppress=False,
            )
            self._listener.start()
        else:
            log.warning(
                "pynput not installed — keyboard control unavailable. "
                "Install with: pip install pynput"
            )

        self._ramp_thread = threading.Thread(
            target=self._ramp_loop, name="keyboard_ramp", daemon=True
        )
        self._ramp_thread.start()

        log.info(
            "Keyboard receiver started.\n"
            "  W/S = Throttle  |  A/D = Steer  |  M = Mode toggle\n"
            "  SPACE = Brake   |  ESC/Q = Emergency stop"
        )

    def stop(self):
        self._running = False
        if self._listener:
            self._listener.stop()

    def read(self) -> ChannelValues:
        """Return current simulated channel values (same as FlySkyReceiver.read)."""
        with self._lock:
            if self._estop:
                return ChannelValues(valid=False)
            return ChannelValues(
                steering    = self._steering,
                throttle    = 0.0 if self._brake else self._throttle,
                mode_raw_us = self._mode_us,
                valid       = True,
            )

    # ── Key callbacks ─────────────────────────────────────────────────────────


    def handle_cv2_key(self, key_code: int):
        import time
        if key_code == -1 or key_code == 255:
            return
        try:
            k = chr(key_code).lower()
        except ValueError:
            return

        with self._lock:
            if k in ("w", "s", "a", "d", " ", "m", "q"):
                # Handle toggles once per hold
                if k == "m" and k not in self._pressed:
                    self._mode_us = 2000 if self._mode_us == 1000 else 1000
                    mode_name = "AUTO-ASSIST" if self._mode_us == 2000 else "MANUAL"
                    log.info(f"Mode switched to: {mode_name}")
                elif k == "q":
                    log.warning("OpenCV ESC/Q — emergency stop!")
                    self._estop = True
                elif k == " ":
                    self._brake = True
                
                self._pressed.add(k)
                self._cv2_keys_last_seen[k] = time.monotonic()

    def _on_press(self, key):

        try:
            k = key.char.lower() if hasattr(key, "char") and key.char else None
        except AttributeError:
            k = None

        with self._lock:
            if k in ("w", "s", "a", "d"):
                self._pressed.add(k)
            elif k == " " or key == _kb.Key.space:
                self._brake = True
            elif k == "m":
                # Toggle mode
                self._mode_us = 2000 if self._mode_us == 1000 else 1000
                mode_name = "AUTO-ASSIST" if self._mode_us == 2000 else "MANUAL"
                log.info(f"Mode switched to: {mode_name}")
            elif k in ("q",) or key == _kb.Key.esc:
                log.warning("Keyboard ESC/Q — emergency stop!")
                self._estop = True

    def _on_release(self, key):
        try:
            k = key.char.lower() if hasattr(key, "char") and key.char else None
        except AttributeError:
            k = None

        with self._lock:
            self._pressed.discard(k)
            if key == _kb.Key.space:
                self._brake = False

    # ── Ramp loop ─────────────────────────────────────────────────────────────

    def _ramp_loop(self):
        """Continuously ramp steering/throttle toward target based on keys held."""
        import time
        dt = 1.0 / 30

        while self._running:
            time.sleep(dt)
            with self._lock:
                now = time.monotonic()
                expired = []
                for k, t in self._cv2_keys_last_seen.items():
                    if now - t > 0.15:
                        expired.append(k)
                for k in expired:
                    self._pressed.discard(k)
                    del self._cv2_keys_last_seen[k]
                    if k == " ":
                        self._brake = False

                pressed = set(self._pressed)


            # Throttle
            if "w" in pressed:
                self._ramp("_throttle", +RAMP_UP)
            elif "s" in pressed:
                self._ramp("_throttle", -RAMP_UP)
            else:
                self._return_to_zero("_throttle")

            # Steering
            if "d" in pressed:
                self._ramp("_steering", +RAMP_UP)
            elif "a" in pressed:
                self._ramp("_steering", -RAMP_UP)
            else:
                self._return_to_zero("_steering")

    def _ramp(self, attr: str, delta: float):
        val = getattr(self, attr) + delta
        setattr(self, attr, max(-1.0, min(1.0, val)))

    def _return_to_zero(self, attr: str):
        val = getattr(self, attr)
        if abs(val) < RAMP_DOWN:
            setattr(self, attr, 0.0)
        elif val > 0:
            setattr(self, attr, val - RAMP_DOWN)
        else:
            setattr(self, attr, val + RAMP_DOWN)

    @property
    def emergency_stopped(self) -> bool:
        return self._estop
