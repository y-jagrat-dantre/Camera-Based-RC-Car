"""
vision/driveable_area.py
========================
Divides the camera frame into LEFT / CENTER / RIGHT zones and
determines whether each zone is CLEAR, PARTIALLY_BLOCKED, or BLOCKED.

Two complementary methods are combined:

  1. YOLO detections — checks if any bounding box occupies
     a significant portion of each zone.

  2. Edge / texture analysis — uses Canny edge detection and
     morphological operations to find dense edge regions (walls,
     obstacles) in each zone even if YOLO doesn't detect them.

The combination provides better coverage: YOLO handles known objects;
edge analysis catches unknown obstacles (random boxes, walls, debris).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import cv2
import numpy as np

from smart_rc_car.config.settings import CFG
from smart_rc_car.vision.depth_estimator import ClosestObstacleEstimator
from smart_rc_car.vision.yolo_detector import Detection

log = logging.getLogger(__name__)


class ZoneStatus(Enum):
    CLEAR    = auto()
    PARTIAL  = auto()
    BLOCKED  = auto()

    def __str__(self):
        return self.name


@dataclass
class ZoneAnalysis:
    left:   ZoneStatus = ZoneStatus.CLEAR
    center: ZoneStatus = ZoneStatus.CLEAR
    right:  ZoneStatus = ZoneStatus.CLEAR

    def any_blocked(self) -> bool:
        return any(s == ZoneStatus.BLOCKED for s in
                   (self.left, self.center, self.right))

    def center_blocked(self) -> bool:
        return self.center == ZoneStatus.BLOCKED

    def safest_side(self) -> Optional[str]:
        """Return 'left' or 'right' — whichever is clearer. None if both blocked."""
        scores = {
            "left":  self._score(self.left),
            "right": self._score(self.right),
        }
        best_side = min(scores, key=scores.get)
        if scores[best_side] >= 2:  # Both blocked
            return None
        return best_side

    @staticmethod
    def _score(status: ZoneStatus) -> int:
        return {ZoneStatus.CLEAR: 0, ZoneStatus.PARTIAL: 1,
                ZoneStatus.BLOCKED: 2}[status]


class DriveableAreaAnalyzer:
    """
    Analyzes a camera frame to determine zone clearance.

    Parameters
    ----------
    frame_width  : int  full frame width in pixels
    frame_height : int  full frame height in pixels
    """

    def __init__(self, frame_width: int = 640, frame_height: int = 480):
        self._vcfg = CFG.vision
        self._fw   = frame_width
        self._fh   = frame_height
        self._depth_est = ClosestObstacleEstimator()

        # Pre-compute zone x-boundaries
        lf  = self._vcfg.zone_left_fraction
        cf  = self._vcfg.zone_center_fraction
        self._left_x_end   = int(self._fw * lf)
        self._right_x_start = int(self._fw * (lf + cf))
        self._roi_y_start   = int(self._fh * self._vcfg.roi_top_fraction)

    # ── Public API ────────────────────────────────────────────────────────────

    def analyze(
        self,
        frame: np.ndarray,
        detections: list[Detection],
    ) -> ZoneAnalysis:
        """
        Analyze zones using both YOLO bboxes and edge detection.

        Parameters
        ----------
        frame      : BGR numpy array (full resolution)
        detections : YOLO detection results

        Returns
        -------
        ZoneAnalysis
        """
        # Method 1: YOLO bounding box analysis
        yolo_zones = self._analyze_yolo(detections)

        # Method 2: Edge density analysis
        edge_zones = self._analyze_edges(frame)

        # Combine: take the worst (highest) status from both methods
        combined = ZoneAnalysis(
            left   = self._worst(yolo_zones.left,   edge_zones.left),
            center = self._worst(yolo_zones.center, edge_zones.center),
            right  = self._worst(yolo_zones.right,  edge_zones.right),
        )
        return combined

    def get_zone_boundaries(self) -> tuple[int, int, int, int]:
        """Return (left_x_end, right_x_start, roi_y_start, frame_height)."""
        return (self._left_x_end, self._right_x_start,
                self._roi_y_start, self._fh)

    # ── Private methods ───────────────────────────────────────────────────────

    @staticmethod
    def _worst(a: ZoneStatus, b: ZoneStatus) -> ZoneStatus:
        scores = {ZoneStatus.CLEAR: 0, ZoneStatus.PARTIAL: 1,
                  ZoneStatus.BLOCKED: 2}
        worst = max(a, b, key=lambda s: scores[s])
        return worst

    def _analyze_yolo(self, detections: list[Detection]) -> ZoneAnalysis:
        """Check how much each YOLO bounding box overlaps each zone."""
        zone_scores = {"left": 0.0, "center": 0.0, "right": 0.0}
        frame_area = max(1, self._fw * (self._fh - self._roi_y_start))
        block_thresh = self._vcfg.block_threshold

        zones = {
            "left":   (0,                  self._left_x_end),
            "center": (self._left_x_end,   self._right_x_start),
            "right":  (self._right_x_start, self._fw),
        }

        for det in detections:
            # Only consider detections in the lower ROI (road level)
            if det.y2 < self._roi_y_start:
                continue

            for zone_name, (zx1, zx2) in zones.items():
                zone_w = zx2 - zx1
                if zone_w <= 0:
                    continue
                # Intersection of bbox with zone
                ix1 = max(det.x1, zx1)
                ix2 = min(det.x2, zx2)
                overlap_w = max(0, ix2 - ix1)
                fraction  = overlap_w / zone_w
                zone_scores[zone_name] = max(zone_scores[zone_name], fraction)

        def score_to_status(score: float) -> ZoneStatus:
            if score >= block_thresh:
                return ZoneStatus.BLOCKED
            elif score >= block_thresh * 0.5:
                return ZoneStatus.PARTIAL
            return ZoneStatus.CLEAR

        return ZoneAnalysis(
            left   = score_to_status(zone_scores["left"]),
            center = score_to_status(zone_scores["center"]),
            right  = score_to_status(zone_scores["right"]),
        )

    def _analyze_edges(self, frame: np.ndarray) -> ZoneAnalysis:
        """
        Use Canny edge detection to find obstacle regions.
        Walls and close obstacles show up as dense edge areas.
        """
        # Work on lower ROI only
        roi = frame[self._roi_y_start:, :]

        gray    = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges   = cv2.Canny(blurred, 50, 150)

        # Dilate edges to fill gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        edges  = cv2.dilate(edges, kernel, iterations=2)

        roi_h, roi_w = edges.shape

        # Only analyze bottom 60% (ground-level obstacles)
        lower_start = int(roi_h * 0.4)
        lower_edges = edges[lower_start:, :]

        total_pixels = max(1, lower_edges.shape[0] * lower_edges.shape[1])

        def zone_density(x1: int, x2: int) -> float:
            zone = lower_edges[:, x1:x2]
            return float(np.sum(zone > 0)) / max(1, zone.size)

        # Remap zone boundaries to ROI width
        # (roi_w should equal frame_w unless we cropped horizontally)
        rw = roi_w
        lx = int(rw * self._vcfg.zone_left_fraction)
        rx = int(rw * (self._vcfg.zone_left_fraction +
                       self._vcfg.zone_center_fraction))

        d_left   = zone_density(0,  lx)
        d_center = zone_density(lx, rx)
        d_right  = zone_density(rx, rw)

        # Thresholds tuned empirically — adjust via config or testing
        def density_to_status(d: float) -> ZoneStatus:
            if d > 0.35:   return ZoneStatus.BLOCKED
            elif d > 0.15: return ZoneStatus.PARTIAL
            return ZoneStatus.CLEAR

        return ZoneAnalysis(
            left   = density_to_status(d_left),
            center = density_to_status(d_center),
            right  = density_to_status(d_right),
        )

    def draw_zones(
        self,
        frame: np.ndarray,
        analysis: ZoneAnalysis,
    ) -> np.ndarray:
        """Draw zone overlays on the frame for visualization."""
        out = frame.copy()
        h, w = out.shape[:2]

        color_map = {
            ZoneStatus.CLEAR:   (0, 200, 0),    # Green
            ZoneStatus.PARTIAL: (0, 165, 255),  # Orange
            ZoneStatus.BLOCKED: (0, 0, 220),    # Red
        }

        zones = [
            ("LEFT",   0,                   self._left_x_end, analysis.left),
            ("CENTER", self._left_x_end,    self._right_x_start, analysis.center),
            ("RIGHT",  self._right_x_start, w, analysis.right),
        ]

        overlay = out.copy()
        for label, x1, x2, status in zones:
            color = color_map[status]
            cv2.rectangle(overlay, (x1, self._roi_y_start), (x2, h), color, -1)
            # Label
            tx = x1 + (x2 - x1) // 2 - 30
            cv2.putText(overlay, f"{label}", (tx, self._roi_y_start + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(overlay, str(status), (tx, self._roi_y_start + 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.addWeighted(overlay, 0.30, out, 0.70, 0, out)

        # Zone divider lines
        cv2.line(out, (self._left_x_end, 0), (self._left_x_end, h),
                 (200, 200, 200), 1)
        cv2.line(out, (self._right_x_start, 0), (self._right_x_start, h),
                 (200, 200, 200), 1)
        # ROI line
        cv2.line(out, (0, self._roi_y_start), (w, self._roi_y_start),
                 (100, 100, 255), 1)

        return out
