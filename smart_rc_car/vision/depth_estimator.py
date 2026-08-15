"""
vision/depth_estimator.py
=========================
Estimates how far away a detected obstacle is using camera-only methods.

Default method: Bounding-box height ratio (Approach B from spec).
  - As an object gets closer, its bounding box occupies more vertical
    height of the frame.
  - Estimated distance ∝ 1 / bbox_height_fraction

This is approximate and assumes a roughly fixed camera height and angle.
For better accuracy, add a depth camera or LiDAR later.

The architecture is pluggable via the `DepthEstimatorBase` interface.
"""
from __future__ import annotations

import abc
import logging
from typing import Optional

import numpy as np

from smart_rc_car.vision.yolo_detector import Detection

log = logging.getLogger(__name__)


class DepthEstimatorBase(abc.ABC):
    @abc.abstractmethod
    def estimate(self, detection: Detection, frame_height: int) -> float:
        """Return estimated distance in meters."""
        ...


class BBoxDepthEstimator(DepthEstimatorBase):
    """
    Estimates distance from bounding-box apparent height.

    Calibration:
        ref_height_fraction : fraction of frame height the object occupies
                              at the reference distance
        ref_distance_m      : the real-world distance at calibration

    Example: if a 30cm-tall box occupies 25% of the frame at 1 metre away,
    set ref_height_fraction=0.25, ref_distance_m=1.0

    These defaults are rough — tune via calibration.md.
    """

    def __init__(
        self,
        ref_height_fraction: float = 0.25,
        ref_distance_m: float = 1.0,
    ):
        self._ref_hf = ref_height_fraction
        self._ref_d  = ref_distance_m

    def estimate(self, detection: Detection, frame_height: int) -> float:
        if frame_height <= 0 or detection.height <= 0:
            return 9.99  # Unknown — return large value (far away)

        hf = detection.height / frame_height
        if hf < 0.01:
            return 9.99

        # Inverse proportion: distance ∝ ref_hf / hf * ref_distance
        distance = (self._ref_hf / hf) * self._ref_d
        return max(0.05, min(10.0, distance))


class ClosestObstacleEstimator:
    """
    Given a list of detections, returns the estimated distance to
    the closest obstacle in the scene.
    """

    def __init__(self):
        self._estimator = BBoxDepthEstimator()

    def closest_distance(
        self,
        detections: list[Detection],
        frame_height: int,
    ) -> float:
        """
        Return distance in meters to the closest detection.
        Returns a large number (9.99) if no detections.
        """
        if not detections:
            return 9.99
        distances = [
            self._estimator.estimate(d, frame_height)
            for d in detections
        ]
        return min(distances)

    def distance_for(
        self,
        detection: Detection,
        frame_height: int,
    ) -> float:
        return self._estimator.estimate(detection, frame_height)
