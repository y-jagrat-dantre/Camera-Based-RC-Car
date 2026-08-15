"""
dashboard/app.py
================
Flask + Flask-SocketIO web dashboard.

Provides:
  - Live MJPEG camera feed at /video_feed
  - WebSocket telemetry push every 100ms
  - REST endpoint GET /status
  - REST endpoint POST /mode  {mode: "MANUAL" | "AUTO_ASSIST"}
  - REST endpoint POST /estop  (trigger emergency stop)
  - REST endpoint POST /rearm  (re-arm after stop)

The dashboard receives state via the global `DASHBOARD_STATE` dict
which the main loop updates each cycle.

Usage:
    from smart_rc_car.dashboard.app import DashboardServer
    server = DashboardServer()
    server.update(state_dict)
    server.start()
"""
from __future__ import annotations

import io
import logging
import threading
import time
from typing import Any

import cv2
import numpy as np

from smart_rc_car.config.settings import CFG
from smart_rc_car.safety.emergency_stop import ESTOP, StopReason

log = logging.getLogger(__name__)

try:
    from flask import Flask, Response, jsonify, render_template, request
    from flask_socketio import SocketIO
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False
    log.warning("Flask/Flask-SocketIO not installed. Dashboard disabled.")


class DashboardServer:
    """
    Web dashboard server.

    Thread-safe state store + MJPEG stream + WebSocket telemetry.
    """

    def __init__(self):
        self._state: dict[str, Any] = {
            "mode":        "MANUAL",
            "remote":      "DISCONNECTED",
            "camera":      "DISCONNECTED",
            "ai":          "STOPPED",
            "obstacle":    "NONE",
            "zone_left":   "CLEAR",
            "zone_center": "CLEAR",
            "zone_right":  "CLEAR",
            "ai_action":   "WAITING",
            "distance_m":  None,
            "ramp":        False,
            "throttle":    0.0,
            "steering":    0.0,
            "estop":       False,
            "estop_reason": None,
            "uptime_s":    0,
        }
        self._state_lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()
        self._app: "Flask | None" = None
        self._sio: "SocketIO | None" = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._start_time = time.monotonic()
        self.pressed_keys = set()

    def update(self, state: dict[str, Any]):
        """Update dashboard state (called from main loop, thread-safe)."""
        with self._state_lock:
            self._state.update(state)
            self._state["uptime_s"] = int(time.monotonic() - self._start_time)
            self._state["estop"]    = ESTOP.active
            self._state["estop_reason"] = (
                ESTOP.reason.name if ESTOP.reason else None
            )

    def update_frame(self, frame: np.ndarray):
        """Update the camera frame shown in the MJPEG stream."""
        with self._frame_lock:
            self._frame = frame

    def start(self):
        if not FLASK_AVAILABLE or not CFG.dashboard.enabled:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run_server,
            name="dashboard",
            daemon=True,
        )
        self._thread.start()
        log.info(
            f"Dashboard starting at http://{CFG.dashboard.host}:"
            f"{CFG.dashboard.port}"
        )

    def stop(self):
        self._running = False

    def _run_server(self):
        app = Flask(__name__, template_folder="templates")
        app.config["SECRET_KEY"] = "rc_car_secret"
        sio = SocketIO(app, async_mode="threading", cors_allowed_origins="*")
        self._app = app
        self._sio = sio

        # ── Routes ─────────────────────────────────────────────────────────────

        @app.route("/")
        def index():
            return render_template("index.html")

        @app.route("/status")
        def status():
            with self._state_lock:
                return jsonify(dict(self._state))

        @app.route("/video_feed")
        def video_feed():
            return Response(
                self._generate_mjpeg(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.route("/mode", methods=["POST"])
        def set_mode():
            data = request.get_json()
            # Note: actual mode is controlled by the CT6B switch.
            # This endpoint is for logging / future use.
            new_mode = data.get("mode", "MANUAL")
            log.info(f"Dashboard requested mode: {new_mode}")
            return jsonify({"status": "ok", "mode": new_mode})

        @app.route("/estop", methods=["POST"])
        def trigger_estop():
            ESTOP.trigger(StopReason.MANUAL_TRIGGER)
            return jsonify({"status": "emergency_stop_triggered"})

        @app.route("/rearm", methods=["POST"])
        def rearm():
            ESTOP.clear()
            return jsonify({"status": "rearmed"})

        # ── SocketIO telemetry push ─────────────────────────────────────────────

        
        @sio.on('keydown')
        def handle_keydown(data):
            k = data.get('key')
            if k: self.pressed_keys.add(k)

        @sio.on('keyup')
        def handle_keyup(data):
            k = data.get('key')
            if k in self.pressed_keys: self.pressed_keys.remove(k)

        @sio.on('disconnect')
        def handle_disconnect():
            self.pressed_keys.clear()

        def telemetry_loop():
            while self._running:
                time.sleep(0.10)
                with self._state_lock:
                    snapshot = dict(self._state)
                sio.emit("telemetry", snapshot)

        tel_thread = threading.Thread(
            target=telemetry_loop, name="telemetry", daemon=True
        )
        tel_thread.start()

        # Run Flask
        sio.run(
            app,
            host=CFG.dashboard.host,
            port=CFG.dashboard.port,
            use_reloader=False,
            log_output=False,
        )

    def _generate_mjpeg(self):
        """Generator that yields MJPEG frames."""
        quality = CFG.dashboard.mjpeg_quality
        while True:
            with self._frame_lock:
                frame = self._frame

            if frame is None:
                # Send a blank frame
                blank = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(blank, "No Camera", (60, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
                frame = blank

            ret, jpeg = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            )
            if not ret:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                jpeg.tobytes() +
                b"\r\n"
            )
            time.sleep(1 / 20)  # ~20 FPS for dashboard stream
