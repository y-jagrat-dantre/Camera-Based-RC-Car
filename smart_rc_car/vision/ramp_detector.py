"""
vision/ramp_detector.py
=======================
Detects upward-sloping ramps in the camera feed using line geometry.

Method:
  1. Crop the center-bottom portion of the frame (where ramp appears).
  2. Convert to grayscale, apply edge detection.
  3. Use Hough line transform to find long lines.
  4. Classify lines as "ramp lines" if they have a significant slope
     and converge toward a vanishing point above center.
  5. Vote — if enough ramp lines are found consistently, a ramp is detected.

Returns RampInfo with detection flag, estimated angle, and confidence.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import cv2
import numpy as np

from smart_rc_car.config.settings import CFG

log = logging.getLogger(__name__)


@dataclass
class RampInfo:
    detected:    bool  = False
    angle_deg:   float = 0.0    # estimated slope angle
    confidence:  float = 0.0    # 0.0 – 1.0
    side:        str   = "center"  # "left", "right", or "center"


class RampDetector:
    """
    Detects ramps using Hough-line analysis on the lower-center frame region.
    """

    def __init__(self):
        self._cfg        = CFG.vision
        self._angle_min  = self._cfg.ramp_angle_threshold_deg
        # Temporal smoothing: ramp must be detected in multiple frames
        self._history    = [False] * 5
        self._angle_hist : list[float] = []

    def detect(self, frame: np.ndarray) -> RampInfo:
        """
        Run ramp detection on a frame.

        Parameters
        ----------
        frame : np.ndarray  BGR full-resolution frame

        Returns
        -------
        RampInfo
        """
        h, w = frame.shape[:2]

        # Focus on lower-center region where ramp surface would appear
        y1 = int(h * 0.45)   # Start from 45% down
        x1 = int(w * 0.20)   # Ignore far left/right
        x2 = int(w * 0.80)
        roi = frame[y1:h, x1:x2]

        gray    = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges   = cv2.Canny(blurred, 30, 100)

        # Hough probabilistic line detection
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=40,
            minLineLength=int(roi.shape[1] * 0.25),
            maxLineGap=20,
        )

        ramp_angles: list[float] = []

        if lines is not None:
            lines = lines.reshape(-1, 4)
            for x1l, y1l, x2l, y2l in lines:
                dx = x2l - x1l
                dy = y2l - y1l
                if abs(dx) < 5:
                    continue  # Near-vertical — not a ramp surface line

                angle_rad = math.atan2(abs(dy), abs(dx))
                angle_deg = math.degrees(angle_rad)

                # Ramp lines have moderate slope (not flat, not steep)
                if self._angle_min <= angle_deg <= 45:
                    ramp_angles.append(angle_deg)

        # Update history
        detected_now = len(ramp_angles) >= 3
        self._history.pop(0)
        self._history.append(detected_now)

        confirmed = sum(self._history) >= 3  # Majority vote over 5 frames
        confidence = sum(self._history) / len(self._history)

        if ramp_angles:
            avg_angle = float(np.mean(ramp_angles))
        else:
            avg_angle = 0.0

        return RampInfo(
            detected   = confirmed,
            angle_deg  = avg_angle,
            confidence = confidence if confirmed else 0.0,
        )
