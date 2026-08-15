"""
simulate.py
===========
PC Simulation launcher for the Smart RC Car system.

Runs the full AI pipeline (camera, YOLO, vision, dashboard)
without any Raspberry Pi hardware:

  • Camera  → your PC webcam
  • Motors  → simulated (printed to console)
  • Receiver→ keyboard (W/A/S/D + M for mode, SPACE=brake, ESC=stop)
  • Dashboard → http://localhost:5000

Usage:
    python simulate.py
    python simulate.py --no-yolo      # Skip YOLO (faster, edge-only detection)
    python simulate.py --no-dashboard # Run without web dashboard
    python simulate.py --camera 1     # Use webcam index 1

Controls:
    W          Forward
    S          Reverse
    A          Steer left
    D          Steer right
    M          Toggle Manual / Auto-Assist mode
    SPACE      Brake
    ESC / Q    Emergency stop
"""
from __future__ import annotations

import argparse
import logging
import logging.handlers
import signal
import sys
import time
from pathlib import Path

# ── Logging setup ─────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)-28s  %(message)s",
    datefmt="%H:%M:%S",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger("simulate")


def parse_args():
    p = argparse.ArgumentParser(description="Smart RC Car — PC Simulator")
    p.add_argument("--no-yolo",      action="store_true", help="Disable YOLO detection")
    p.add_argument("--no-dashboard", action="store_true", help="Disable web dashboard")
    p.add_argument("--camera",       type=int, default=0,  help="Webcam device index")
    p.add_argument("--resolution",   type=str, default="640x480",
                   help="Camera resolution WxH (default: 640x480)")
    return p.parse_args()


def main():
    args = parse_args()

    # ── Override camera config for PC if needed ────────────────────────────────
    # Patch config before importing modules that read it at import time
    from smart_rc_car.config.settings import CFG
    CFG.camera.device_index = args.camera
    try:
        w, h = args.resolution.split("x")
        CFG.camera.width  = int(w)
        CFG.camera.height = int(h)
    except ValueError:
        pass

    # ── Import all modules (after config patch) ────────────────────────────────
    import cv2

    from smart_rc_car.camera.camera import CameraCapture
    from smart_rc_car.camera.preprocessing import ImagePreprocessor
    from smart_rc_car.dashboard.app import DashboardServer
    from smart_rc_car.motors.motor_driver import L298NDriver
    from smart_rc_car.navigation.safety_controller import SafetyController
    from smart_rc_car.remote.channels import MODE_AUTO_ASSIST
    from smart_rc_car.remote.keyboard_receiver import KeyboardReceiver
    from smart_rc_car.safety.emergency_stop import ESTOP, StopReason
    from smart_rc_car.safety.watchdog import Watchdog
    from smart_rc_car.vision.obstacle_detector import ObstacleDetector

    TARGET_HZ   = 30
    LOOP_PERIOD = 1.0 / TARGET_HZ

    log.info("=" * 60)
    log.info("  Smart RC Car — PC SIMULATION MODE")
    log.info("=" * 60)
    log.info("Controls: W/S=Throttle  A/D=Steer  M=Mode  SPACE=Brake  ESC=Stop")
    log.info("")

    # ── Motors — simulation mode (no GPIO) ────────────────────────────────────
    log.info("Initializing simulated motor driver...")
    motors = L298NDriver(simulate=True)

    # ── Keyboard receiver ─────────────────────────────────────────────────────
    log.info("Initializing keyboard receiver...")
    receiver = KeyboardReceiver()
    receiver.start()

    # ── Camera (PC webcam) ────────────────────────────────────────────────────
    log.info(f"Initializing camera (device {args.camera})...")
    camera = CameraCapture()
    camera.start()
    preprocessor = ImagePreprocessor()
    time.sleep(0.5)

    if not camera.connected:
        log.error(
            f"Cannot open webcam {args.camera}. "
            "Try: python simulate.py --camera 1"
        )
        sys.exit(1)

    # ── Vision (YOLO optional) ────────────────────────────────────────────────
    log.info("Initializing vision pipeline...")
    if args.no_yolo:
        log.info("  YOLO disabled — using edge detection only.")

    vision = ObstacleDetector(skip_yolo=args.no_yolo)
    vision.start()   # start() is a no-op for YOLO when skip_yolo=True

    # ── Safety controller ─────────────────────────────────────────────────────
    controller = SafetyController(motors)

    # ── Watchdog ──────────────────────────────────────────────────────────────
    watchdog = Watchdog()
    watchdog.start()

    # ── Dashboard ─────────────────────────────────────────────────────────────
    if not args.no_dashboard:
        dashboard = DashboardServer()
        dashboard.start()
        log.info("Dashboard: http://localhost:5000")
    else:
        dashboard = None

    # ── Signal handling ───────────────────────────────────────────────────────
    shutdown = [False]
    def _handle_signal(sig, frame):
        shutdown[0] = True
    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # ── HUD overlay on the OpenCV window ─────────────────────────────────────
    def draw_hud(frame, channels, output, report):
        """Draw keyboard control hint + status overlay on the preview window."""
        h, w = frame.shape[:2]
        overlay = frame.copy()

        # Semi-transparent top bar
        cv2.rectangle(overlay, (0, 0), (w, 90), (15, 17, 25), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

        mode = channels.mode
        mode_color = (0, 200, 100) if mode == MODE_AUTO_ASSIST else (150, 150, 150)
        cv2.putText(frame, f"MODE: {mode}", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, mode_color, 2)

        cv2.putText(frame, f"T:{channels.throttle:+.2f}  S:{channels.steering:+.2f}",
                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        ai_color = (0, 140, 255) if output.ai_overriding else (80, 200, 80)
        cv2.putText(frame, f"AI: {output.ai_action}", (10, 72),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, ai_color, 1)

        if ESTOP.active:
            cv2.rectangle(frame, (0, 0), (w, h), (0, 0, 220), 5)
            cv2.putText(frame, "EMERGENCY STOP", (w//2 - 140, h//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        # Key hints at bottom
        hint = "W/S:Throttle  A/D:Steer  M:Mode  SPACE:Brake  ESC:Stop"
        cv2.putText(frame, hint, (5, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80, 80, 80), 1)
        return frame

    # ── Main loop ─────────────────────────────────────────────────────────────
    log.info("Starting simulation loop. Preview window should open...")
    frame_count    = 0
    last_report    = None
    last_loop_time = time.monotonic()
    fps_t          = time.monotonic()
    fps_frames     = 0
    current_fps    = 0.0

    while not shutdown[0]:
        loop_start = time.monotonic()
        dt = loop_start - last_loop_time
        last_loop_time = loop_start

        watchdog.kick()

        # Check keyboard emergency stop
        if receiver.emergency_stopped:
            ESTOP.trigger(StopReason.MANUAL_TRIGGER)
            shutdown[0] = True
            break

        # Read simulated receiver
        channels = receiver.read()

        # Capture + process frame
        raw_frame = camera.read()
        annotated_frame = None
        report = last_report

        if raw_frame is not None:
            frame = preprocessor.process(raw_frame)

            if channels.mode == MODE_AUTO_ASSIST and not args.no_yolo:
                report = vision.process(frame)
                last_report = report
            elif channels.mode == MODE_AUTO_ASSIST:
                # Edge-only vision (YOLO disabled)
                report = vision.process(frame)
                last_report = report
            else:
                report = None

            if report is not None:
                annotated_frame = vision.annotate_frame(frame, report)
            else:
                annotated_frame = frame.copy()

            frame_count += 1
            fps_frames  += 1

        # Compute FPS every second
        if time.monotonic() - fps_t >= 1.0:
            current_fps = fps_frames / (time.monotonic() - fps_t)
            fps_frames  = 0
            fps_t       = time.monotonic()

        # Safety controller
        output = controller.update(channels, report, dt)

        # Show preview with HUD
        if annotated_frame is not None:
            display = draw_hud(annotated_frame, channels, output, report)

            # FPS counter
            cv2.putText(display, f"{current_fps:.0f} FPS",
                        (display.shape[1] - 80, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 60), 1)

            cv2.imshow("Smart RC Car — PC Simulator (press ESC to quit)", display)

            key = cv2.waitKey(1) & 0xFF
            receiver.handle_cv2_key(key)
            if key == 27 or key == ord("q"):  # ESC or Q in OpenCV window
                shutdown[0] = True

        # Dashboard update
        if dashboard:
            zones  = report.zones if report else None
            dashboard.update({
                "mode":        output.mode,
                "remote":      "CONNECTED (KEYBOARD)",
                "camera":      "CONNECTED" if camera.connected else "DISCONNECTED",
                "ai":          "RUNNING" if channels.mode == MODE_AUTO_ASSIST else "STANDBY",
                "obstacle":    "DETECTED" if (report and report.danger) else "NONE",
                "zone_left":   str(zones.left)   if zones else "CLEAR",
                "zone_center": str(zones.center) if zones else "CLEAR",
                "zone_right":  str(zones.right)  if zones else "CLEAR",
                "ai_action":   output.ai_action,
                "distance_m":  report.closest_distance_m if report else None,
                "ramp":        report.ramp.detected if report else False,
                "throttle":    round(channels.throttle, 3),
                "steering":    round(channels.steering, 3),
            })
            if annotated_frame is not None:
                dashboard.update_frame(annotated_frame)

        # Print compact status to terminal every 30 frames
        if frame_count % 30 == 1:
            zones = report.zones if report else None
            zl = str(zones.left)[0]   if zones else "?"
            zc = str(zones.center)[0] if zones else "?"
            zr = str(zones.right)[0]  if zones else "?"
            print(
                f"\r  Mode={output.mode[:4]}  "
                f"T={channels.throttle:+.2f}  S={channels.steering:+.2f}  "
                f"Zones=[{zl}|{zc}|{zr}]  "
                f"AI={output.ai_action[:20]:<20}  "
                f"L={output.left:+.2f} R={output.right:+.2f}",
                end="",
            )

        # Sleep to target rate
        elapsed = time.monotonic() - loop_start
        sleep_t = LOOP_PERIOD - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    # ── Shutdown ──────────────────────────────────────────────────────────────
    print()
    log.info("Shutting down simulation...")
    watchdog.stop()
    motors.stop_all()
    camera.stop()
    vision.stop()
    receiver.stop()
    if dashboard:
        dashboard.stop()
    cv2.destroyAllWindows()
    log.info("Simulation ended.")


if __name__ == "__main__":
    main()
