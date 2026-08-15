"""
vision/yolo_detector.py
========================
YOLOv8 object detector wrapper.

Uses the Ultralytics YOLOv8n (nano) model — optimized for speed on
Raspberry Pi 5. The model is loaded once at startup and reused for
all subsequent frames.

Inference runs in a dedicated thread separate from camera capture,
so a slow inference call does not block the control loop.

Standalone test:
    python -m smart_rc_car.vision.yolo_detector
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from smart_rc_car.config.settings import CFG

log = logging.getLogger(__name__)


@dataclass
class Detection:
    """A single YOLO detection result."""
    class_id:   int
    class_name: str
    confidence: float
    x1: int    # bounding box — top-left x
    y1: int    # bounding box — top-left y
    x2: int    # bounding box — bottom-right x
    y2: int    # bounding box — bottom-right y

    @property
    def center_x(self) -> int:
        return (self.x1 + self.x2) // 2

    @property
    def center_y(self) -> int:
        return (self.y1 + self.y2) // 2

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1

    @property
    def area(self) -> int:
        return self.width * self.height


class YoloDetector:
    """
    Wraps YOLOv8 for real-time object detection.

    Runs inference in a background thread. The latest results are
    always available immediately via `get_detections()`.
    """

    def __init__(self):
        self._cfg   = CFG.vision
        self._model = None
        self._latest: list[Detection] = []
        self._latest_lock = threading.Lock()
        self._running     = False
        self._thread: Optional[threading.Thread] = None
        self._pending_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._new_frame  = threading.Event()
        self._initialized = False

    def start(self):
        """Load the YOLO model and start the inference thread."""
        try:
            from ultralytics import YOLO
            self._model = YOLO(self._cfg.yolo_model)
            # Warm up
            dummy = np.zeros((self._cfg.yolo_input_size,
                              self._cfg.yolo_input_size, 3), dtype=np.uint8)
            self._model.predict(dummy, verbose=False)
            log.info(f"YOLO model loaded: {self._cfg.yolo_model}")
            self._initialized = True
        except Exception as e:
            log.error(f"Failed to load YOLO model: {e}. Detection disabled.")
            self._initialized = False
            return

        self._running = True
        self._thread = threading.Thread(
            target=self._inference_loop,
            name="yolo_inference",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._running = False
        self._new_frame.set()  # Unblock waiting thread
        log.info("YOLO detector stopped.")

    def submit_frame(self, frame: np.ndarray):
        """Submit a new frame for inference (non-blocking)."""
        if not self._initialized:
            return
        with self._frame_lock:
            self._pending_frame = frame
        self._new_frame.set()

    def get_detections(self) -> list[Detection]:
        """Return the latest detection results (thread-safe)."""
        with self._latest_lock:
            return list(self._latest)

    def _inference_loop(self):
        while self._running:
            self._new_frame.wait()
            self._new_frame.clear()

            with self._frame_lock:
                frame = self._pending_frame
                self._pending_frame = None

            if frame is None:
                continue

            try:
                results = self._model.predict(
                    frame,
                    imgsz=self._cfg.yolo_input_size,
                    conf=self._cfg.yolo_confidence,
                    iou=self._cfg.yolo_iou,
                    classes=self._cfg.yolo_target_classes,
                    verbose=False,
                )

                detections: list[Detection] = []
                if results and results[0].boxes is not None:
                    boxes = results[0].boxes
                    h_scale = frame.shape[0] / self._cfg.yolo_input_size
                    w_scale = frame.shape[1] / self._cfg.yolo_input_size

                    for box in boxes:
                        cls_id = int(box.cls[0])
                        cls_name = self._model.names[cls_id]
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = box.xyxy[0].tolist()

                        # Scale back to original frame size
                        detections.append(Detection(
                            class_id=cls_id,
                            class_name=cls_name,
                            confidence=conf,
                            x1=int(x1 * w_scale),
                            y1=int(y1 * h_scale),
                            x2=int(x2 * w_scale),
                            y2=int(y2 * h_scale),
                        ))

                with self._latest_lock:
                    self._latest = detections

            except Exception as e:
                log.error(f"YOLO inference error: {e}")

    @property
    def initialized(self) -> bool:
        return self._initialized

    def draw_detections(
        self,
        frame: np.ndarray,
        detections: list[Detection],
    ) -> np.ndarray:
        """Draw bounding boxes and labels on a frame copy."""
        out = frame.copy()
        for d in detections:
            color = (0, 255, 100)
            cv2.rectangle(out, (d.x1, d.y1), (d.x2, d.y2), color, 2)
            label = f"{d.class_name} {d.confidence:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (d.x1, d.y1 - th - 4), (d.x1 + tw, d.y1), color, -1)
            cv2.putText(out, label, (d.x1, d.y1 - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        return out


# ── Standalone test ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    from smart_rc_car.camera.camera import CameraCapture

    cam = CameraCapture()
    cam.start()
    det = YoloDetector()
    det.start()

    print("YOLO live detection — press 'q' to quit")
    while True:
        frame = cam.read()
        if frame is not None:
            det.submit_frame(frame)
            detections = det.get_detections()
            annotated = det.draw_detections(frame, detections)
            cv2.imshow("YOLO", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    det.stop()
    cam.stop()
    cv2.destroyAllWindows()
