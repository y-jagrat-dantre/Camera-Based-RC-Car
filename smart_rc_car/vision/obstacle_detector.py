"""
vision/obstacle_detector.py
============================
Top-level vision orchestrator.

Pulls together:
  - YOLO detection results
  - Driveable area zone analysis
  - Depth estimation
  - Ramp detection

And produces a unified ObstacleReport consumed by the navigation layer.

This module runs in the main thread and calls already-computed results
from the threaded YOLO detector.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

import cv2
import numpy as np

from smart_rc_car.config.settings import CFG
from smart_rc_car.vision.depth_estimator import ClosestObstacleEstimator
from smart_rc_car.vision.driveable_area import (
    DriveableAreaAnalyzer,
    ZoneAnalysis,
    ZoneStatus,
)
from smart_rc_car.vision.ramp_detector import RampDetector, RampInfo
from smart_rc_car.vision.yolo_detector import Detection, YoloDetector

log = logging.getLogger(__name__)


@dataclass
class ObstacleReport:
    """Unified obstacle and environment state from the vision pipeline."""
    zones:             ZoneAnalysis
    closest_distance_m: float               = 9.99
    ramp:              RampInfo             = field(default_factory=RampInfo)
    detections:        list[Detection]      = field(default_factory=list)
    confidence:        float                = 1.0

    # Convenience accessors
    @property
    def danger(self) -> bool:
        return (self.zones.center_blocked() or
                self.closest_distance_m <= CFG.vision.danger_distance_m)

    @property
    def warning(self) -> bool:
        return self.closest_distance_m <= CFG.vision.warning_distance_m

    @property
    def all_blocked(self) -> bool:
        return (self.zones.left == ZoneStatus.BLOCKED and
                self.zones.center == ZoneStatus.BLOCKED and
                self.zones.right == ZoneStatus.BLOCKED)

    @property
    def safest_side(self):
        return self.zones.safest_side()


class ObstacleDetector:
    """
    Orchestrates the full vision pipeline per frame.

    Usage:
        detector = ObstacleDetector()
        detector.start()
        report = detector.process(frame)
        detector.stop()
    """

    def __init__(self, skip_yolo: bool = False):
        cfg = CFG.camera
        self._yolo      = YoloDetector()
        self._area_ana  = DriveableAreaAnalyzer(cfg.width, cfg.height)
        self._depth_est = ClosestObstacleEstimator()
        self._ramp_det  = RampDetector()
        self._skip_yolo = skip_yolo

        # Confirmation buffer (avoid single-frame false positives)
        self._confirm_count = 0
        self._confirm_threshold = CFG.navigation.obstacle_confirm_frames

    def start(self):
        """Start background YOLO inference thread (skipped if skip_yolo=True)."""
        if not self._skip_yolo:
            self._yolo.start()

    def stop(self):
        self._yolo.stop()

    def process(self, frame: np.ndarray) -> ObstacleReport:
        """
        Run vision pipeline on a frame.

        Submit frame to YOLO (async), use last available YOLO results,
        run synchronous edge analysis and ramp detection.

        Parameters
        ----------
        frame : np.ndarray  BGR camera frame

        Returns
        -------
        ObstacleReport
        """
        # Submit to YOLO (non-blocking — inference happens in background)
        self._yolo.submit_frame(frame)

        # Get latest YOLO results (may be from previous frame — that's fine)
        detections = self._yolo.get_detections()

        # Synchronous zone analysis
        zones = self._area_ana.analyze(frame, detections)

        # Depth estimation
        closest_dist = self._depth_est.closest_distance(
            detections, frame.shape[0]
        )

        # Ramp detection (sync, cheap)
        ramp = self._ramp_det.detect(frame)

        return ObstacleReport(
            zones              = zones,
            closest_distance_m = closest_dist,
            ramp               = ramp,
            detections         = detections,
            confidence         = 1.0,
        )

    def annotate_frame(
        self,
        frame: np.ndarray,
        report: ObstacleReport,
    ) -> np.ndarray:
        """Return frame with all vision overlays drawn for dashboard display."""
        # Draw zone overlays
        annotated = self._area_ana.draw_zones(frame, report.zones)

        # Draw YOLO bounding boxes
        annotated = self._yolo.draw_detections(annotated, report.detections)

        # Danger indicator
        if report.danger:
            h, w = annotated.shape[:2]
            cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), (0, 0, 255), 4)
            cv2.putText(annotated, "OBSTACLE DETECTED",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 0, 255), 2)

        # Ramp indicator
        if report.ramp.detected:
            cv2.putText(annotated, f"RAMP {report.ramp.angle_deg:.0f}°",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 200, 0), 2)

        # Distance indicator
        dist_text = (f"Dist: {report.closest_distance_m:.2f}m"
                     if report.closest_distance_m < 9.0 else "Dist: --")
        cv2.putText(annotated, dist_text,
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (200, 200, 200), 2)

        return annotated

    @property
    def area_analyzer(self) -> DriveableAreaAnalyzer:
        return self._area_ana

    @property
    def yolo_detector(self) -> YoloDetector:
        return self._yolo
