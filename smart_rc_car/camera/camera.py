"""
camera/camera.py
================
Thread-safe camera capture using OpenCV (USB webcam).

Runs the camera in a background thread so the main loop never blocks
waiting for a frame. Always provides the latest available frame.

Usage:
    cam = CameraCapture()
    cam.start()
    frame = cam.read()      # numpy array (H, W, 3) BGR
    cam.stop()

Standalone test:
    python -m smart_rc_car.camera.camera
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import cv2
import numpy as np

from smart_rc_car.config.settings import CFG

log = logging.getLogger(__name__)


class CameraCapture:
    """
    Non-blocking camera reader.

    Spawns a thread that continuously reads from the camera device.
    `read()` always returns the most recently captured frame instantly.
    """

    def __init__(self):
        self._cfg = CFG.camera
        self._cap: Optional[cv2.VideoCapture] = None
        self._frame: Optional[np.ndarray] = None
        self._lock  = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0
        self._connected   = False

    def start(self):
        """Open the camera and start the capture thread."""
        self._cap = cv2.VideoCapture(self._cfg.device_index)

        if not self._cap.isOpened():
            log.error(f"Cannot open camera device {self._cfg.device_index}")
            return

        # Configure resolution and FPS
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._cfg.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._cfg.height)
        self._cap.set(cv2.CAP_PROP_FPS,          self._cfg.fps)
        # Reduce internal buffer to minimize latency
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        log.info(f"Camera opened: {actual_w}×{actual_h} "
                 f"@ {int(self._cap.get(cv2.CAP_PROP_FPS))} FPS")

        self._connected = True
        self._running   = True
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="camera_capture",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        log.info("Camera stopped.")

    def _capture_loop(self):
        while self._running:
            if self._cap is None or not self._cap.isOpened():
                time.sleep(0.1)
                continue

            ret, frame = self._cap.read()
            if not ret:
                log.warning("Camera read failed — retrying.")
                time.sleep(0.05)
                continue

            # Optional flips
            if self._cfg.flip_horizontal and self._cfg.flip_vertical:
                frame = cv2.flip(frame, -1)
            elif self._cfg.flip_horizontal:
                frame = cv2.flip(frame, 1)
            elif self._cfg.flip_vertical:
                frame = cv2.flip(frame, 0)

            with self._lock:
                self._frame       = frame
                self._frame_count += 1

    def read(self) -> Optional[np.ndarray]:
        """
        Return the latest frame (BGR numpy array), or None if not ready.
        Never blocks.
        """
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    @property
    def connected(self) -> bool:
        return self._connected and self._running

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count

    @property
    def resolution(self) -> tuple[int, int]:
        return (self._cfg.width, self._cfg.height)


# ── Standalone test ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    cam = CameraCapture()
    cam.start()
    print("Camera preview — press 'q' to quit")
    while True:
        frame = cam.read()
        if frame is not None:
            cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cam.stop()
    cv2.destroyAllWindows()
