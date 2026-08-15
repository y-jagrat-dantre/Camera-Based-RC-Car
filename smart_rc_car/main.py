"""
main.py
=======
Smart RC Car — Main Control Loop

Initialization order:
  1. Logging
  2. Emergency stop
  3. Motor driver
  4. FlySky receiver
  5. Camera
  6. Vision / obstacle detector
  7. Safety controller
  8. Watchdog
  9. Web dashboard

Main loop runs at ~30 Hz:
  Read receiver → Capture frame → Vision → Safety decision → Motor output
  → Dashboard update → Watchdog kick → Sleep

Press Ctrl+C or trigger ESTOP to shut down gracefully.
"""
from __future__ import annotations

import logging
import logging.handlers
import signal
import sys
import time
from pathlib import Path

import cv2


# ── Logging setup (must happen before any module import) ─────────────────────
def _setup_logging():
    from smart_rc_car.config.settings import CFG
    log_path = Path(CFG.logging.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    level = getattr(logging, CFG.logging.level.upper(), logging.INFO)

    handler_file = logging.handlers.RotatingFileHandler(
        str(log_path),
        maxBytes=CFG.logging.max_bytes,
        backupCount=CFG.logging.backup_count,
    )
    handler_console = logging.StreamHandler(sys.stdout)

    fmt = logging.Formatter(
        "%(asctime)s  %(levelname)-8s  %(name)-30s  %(message)s",
        datefmt="%H:%M:%S",
    )
    handler_file.setFormatter(fmt)
    handler_console.setFormatter(fmt)

    root = logging.getLogger()
    root.setLevel(level)
    root.addHandler(handler_file)
    root.addHandler(handler_console)


_setup_logging()
log = logging.getLogger("main")

# ── Imports ───────────────────────────────────────────────────────────────────
from smart_rc_car.camera.camera import CameraCapture
from smart_rc_car.camera.preprocessing import ImagePreprocessor
from smart_rc_car.config.settings import CFG
from smart_rc_car.dashboard.app import DashboardServer
from smart_rc_car.motors.motor_driver import create_motor_driver
from smart_rc_car.navigation.safety_controller import SafetyController
from smart_rc_car.remote.channels import MODE_AUTO_ASSIST
from smart_rc_car.remote.flysky_receiver import FlySkyReceiver
from smart_rc_car.safety.emergency_stop import ESTOP, StopReason
from smart_rc_car.safety.watchdog import Watchdog
from smart_rc_car.vision.obstacle_detector import ObstacleDetector

# ── Target loop rate ─────────────────────────────────────────────────────────
TARGET_HZ   = 30
LOOP_PERIOD = 1.0 / TARGET_HZ


def main():
    log.info("=" * 60)
    log.info("  Smart RC Car — Starting up")
    log.info("=" * 60)

    # ── 1. Motor driver ───────────────────────────────────────────────────────
    log.info("Initializing motor driver...")
    motors = create_motor_driver()
    motors.stop_all()

    # ── 2. FlySky receiver ────────────────────────────────────────────────────
    log.info("Initializing FlySky receiver...")
    receiver = FlySkyReceiver()
    receiver.start()

    # ── 3. Camera ─────────────────────────────────────────────────────────────
    log.info("Initializing camera...")
    camera = CameraCapture()
    camera.start()
    preprocessor = ImagePreprocessor()
    time.sleep(0.5)  # Give camera time to open

    # ── 4. Vision ─────────────────────────────────────────────────────────────
    log.info("Initializing obstacle detector (loading YOLO model)...")
    vision = ObstacleDetector()
    vision.start()

    # ── 5. Safety controller ──────────────────────────────────────────────────
    controller = SafetyController(motors)

    # ── 6. Watchdog ───────────────────────────────────────────────────────────
    watchdog = Watchdog()
    watchdog.start()

    # ── 7. Dashboard ──────────────────────────────────────────────────────────
    dashboard = DashboardServer()
    dashboard.start()

    # ── Signal handlers ───────────────────────────────────────────────────────
    shutdown_requested = [False]

    def _handle_signal(sig, frame):
        log.info(f"Signal {sig} received — requesting shutdown.")
        shutdown_requested[0] = True

    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # ── Startup wait — wait for receiver to lock ──────────────────────────────
    log.info("Waiting for receiver signal (up to 5s)...")
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        watchdog.kick()
        if receiver.read().valid:
            log.info("Receiver signal acquired.")
            break
        time.sleep(0.1)
    else:
        log.warning("Receiver signal not found at startup — continuing anyway.")

    log.info("Control loop starting.")
    frame_count       = 0
    last_report       = None
    last_loop_time    = time.monotonic()
    
    web_mode_active = False
    last_m_pressed  = False

    # ── MAIN CONTROL LOOP ─────────────────────────────────────────────────────
    while not shutdown_requested[0]:
        loop_start = time.monotonic()
        dt = loop_start - last_loop_time
        last_loop_time = loop_start

        # ── Kick watchdog ─────────────────────────────────────────────────────
        watchdog.kick()

        # ── Read receiver ─────────────────────────────────────────────────────
        channels = receiver.read()

        # ── Web Override ──────────────────────────────────────────────────────
        web_steering = 0.0
        web_throttle = 0.0
        web_active = False

        if hasattr(dashboard, "pressed_keys"):
            m_pressed = 'm' in dashboard.pressed_keys
            if m_pressed and not last_m_pressed:
                web_mode_active = not web_mode_active
            last_m_pressed = m_pressed

            if dashboard.pressed_keys:
                web_active = True
                if 'w' in dashboard.pressed_keys: web_throttle += 0.8
                if 's' in dashboard.pressed_keys: web_throttle -= 0.8
                if 'd' in dashboard.pressed_keys: web_steering += 1.0
                if 'a' in dashboard.pressed_keys: web_steering -= 1.0
                if ' ' in dashboard.pressed_keys: web_throttle = 0.0
                if 'q' in dashboard.pressed_keys: ESTOP.trigger(StopReason.MANUAL_TRIGGER)

        # If physical remote is off, or if web keys are being actively pressed, override.
        if (not channels.valid) or web_active:
            channels.valid = True
            channels.throttle = web_throttle
            channels.steering = web_steering
            channels.mode_raw_us = 2000 if web_mode_active else 1000

        # ── Capture + process camera frame ───────────────────────────────────
        raw_frame = camera.read()
        annotated_frame = None
        report = last_report  # Use last valid report if no new frame

        if raw_frame is not None:
            # Preprocess
            frame = preprocessor.process(raw_frame)

            # Vision pipeline (submits to YOLO async, sync edge analysis)
            if channels.mode == MODE_AUTO_ASSIST:
                report = vision.process(frame)
                last_report = report
            else:
                report = None  # Don't run vision in manual mode

            # Annotate for dashboard
            if report is not None:
                annotated_frame = vision.annotate_frame(frame, report)
            else:
                annotated_frame = frame

            frame_count += 1

        # ── Safety controller — produces motor commands ───────────────────────
        output = controller.update(channels, report, dt)

        # ── Dashboard update ──────────────────────────────────────────────────
        zones = report.zones if report else None
        dash_state = {
            "mode":        output.mode,
            "remote":      "CONNECTED" if channels.valid else "DISCONNECTED",
            "camera":      "CONNECTED" if camera.connected else "DISCONNECTED",
            "ai":          "RUNNING" if channels.mode == MODE_AUTO_ASSIST else "STANDBY",
            "obstacle":    "DETECTED" if (report and report.danger) else "NONE",
            "zone_left":   str(zones.left)   if zones else "CLEAR",
            "zone_center": str(zones.center) if zones else "CLEAR",
            "zone_right":  str(zones.right)  if zones else "CLEAR",
            "ai_action":   output.ai_action,
            "distance_m":  report.closest_distance_m if report else None,
            "ramp":        report.ramp.detected if report else False,
            "throttle":    round(output.throttle, 3),
            "steering":    round(output.steering, 3),
        }
        dashboard.update(dash_state)
        if annotated_frame is not None:
            dashboard.update_frame(annotated_frame)

        # ── Periodic status log ───────────────────────────────────────────────
        if frame_count % (TARGET_HZ * 5) == 1:
            log.info(
                f"Mode={output.mode}  "
                f"T={output.throttle:+.2f}  "
                f"S={output.steering:+.2f}  "
                f"AI={output.ai_action}  "
                f"Cam={'OK' if camera.connected else 'ERR'}"
            )

        # ── Sleep to hit target rate ──────────────────────────────────────────
        elapsed = time.monotonic() - loop_start
        sleep_t = LOOP_PERIOD - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)
        elif sleep_t < -0.010:
            log.debug(f"Loop overrun: {-sleep_t * 1000:.1f}ms late")

    # ── Shutdown ──────────────────────────────────────────────────────────────
    log.info("Shutdown sequence starting...")
    ESTOP.trigger(StopReason.SOFTWARE_ERROR)
    watchdog.stop()
    motors.stop_all()
    motors.shutdown()
    camera.stop()
    vision.stop()
    dashboard.stop()
    receiver.stop()
    log.info("All systems stopped. Goodbye.")


if __name__ == "__main__":
    main()
