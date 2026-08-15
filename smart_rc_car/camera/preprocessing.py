"""
camera/preprocessing.py
=======================
Image preprocessing pipeline applied to each frame before vision processing.

Steps:
  1. Resize to inference resolution (if different from capture resolution)
  2. Apply camera undistortion (if calibration data available)
  3. Apply ROI crop (ignore sky / distant background)

Returns a preprocessed BGR numpy array.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from smart_rc_car.config.settings import CFG


class ImagePreprocessor:
    """Prepares camera frames for computer vision."""

    def __init__(self):
        self._cfg    = CFG.camera
        self._vcfg   = CFG.vision
        self._cam_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._map1: Optional[np.ndarray] = None
        self._map2: Optional[np.ndarray] = None

        if self._cfg.apply_undistort:
            self._load_calibration()

    def _load_calibration(self):
        cal_path = Path(self._cfg.calibration_file)
        if not cal_path.exists():
            import logging
            logging.getLogger(__name__).warning(
                f"Calibration file not found: {cal_path}. "
                "Undistortion disabled."
            )
            return

        data = np.load(str(cal_path))
        self._cam_matrix  = data["camera_matrix"]
        self._dist_coeffs = data["dist_coefficients"]

        h, w = self._cfg.height, self._cfg.width
        new_cam, _ = cv2.getOptimalNewCameraMatrix(
            self._cam_matrix, self._dist_coeffs, (w, h), 1, (w, h)
        )
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self._cam_matrix, self._dist_coeffs, None,
            new_cam, (w, h), cv2.CV_16SC2
        )

    def process(self, frame: np.ndarray) -> np.ndarray:
        """
        Apply full preprocessing pipeline.

        Parameters
        ----------
        frame : np.ndarray  BGR image from camera

        Returns
        -------
        np.ndarray  Preprocessed BGR image
        """
        # Undistort
        if self._map1 is not None:
            frame = cv2.remap(frame, self._map1, self._map2,
                              cv2.INTER_LINEAR)

        # Resize to YOLO input size if needed (YOLO module handles its own resize,
        # but we keep capture-res for OpenCV analysis)
        # (No resize here — keep full resolution for zone detection)

        return frame

    def apply_roi(self, frame: np.ndarray) -> np.ndarray:
        """
        Crop the frame vertically to the region of interest (remove sky).

        Returns the cropped frame and the y-offset for coordinate mapping.
        """
        h = frame.shape[0]
        top = int(h * self._vcfg.roi_top_fraction)
        return frame[top:, :].copy()

    def resize_for_inference(self, frame: np.ndarray, size: int) -> np.ndarray:
        """Resize frame to square size for model inference."""
        return cv2.resize(frame, (size, size))
