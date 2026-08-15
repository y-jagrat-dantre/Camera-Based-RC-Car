"""
camera/calibration.py
=====================
Camera calibration using a printed chessboard pattern.

Run this script ONCE to generate camera_cal.npz which enables
lens distortion correction.

Usage:
    python -m smart_rc_car.camera.calibration

Instructions:
    1. Print a chessboard pattern (e.g. 9×6 inner corners).
    2. Hold it flat in front of the camera and move it slowly.
    3. Press SPACE to capture each position (need at least 20 images).
    4. Press 'q' when done — calibration runs automatically.
    5. Results saved to config/camera_cal.npz
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path

import cv2
import numpy as np

log = logging.getLogger(__name__)

# Chessboard dimensions (inner corners)
BOARD_W = 9
BOARD_H = 6
SQUARE_SIZE_MM = 25.0   # physical size of each square


def run_calibration(device_index: int = 0,
                    output_path: str = "smart_rc_car/config/camera_cal.npz"):
    """Capture calibration images and compute camera matrix."""

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_points = []   # 3D points in real world
    img_points = []   # 2D points in image plane

    # Prepare object points (0,0,0), (1,0,0), ...
    objp = np.zeros((BOARD_H * BOARD_W, 3), np.float32)
    objp[:, :2] = (
        np.mgrid[0:BOARD_W, 0:BOARD_H].T.reshape(-1, 2) * SQUARE_SIZE_MM
    )

    cap = cv2.VideoCapture(device_index)
    print(f"Calibration: Press SPACE to capture, 'q' to finish and calibrate.")
    print(f"Need at least 20 captures for reliable calibration.")
    captured = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (BOARD_W, BOARD_H), None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, (BOARD_W, BOARD_H), corners, found)
            cv2.putText(display, "DETECTED — press SPACE to capture",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "No pattern detected",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"Captures: {captured}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" ") and found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(corners2)
            captured += 1
            print(f"  Captured position {captured}")

    cap.release()
    cv2.destroyAllWindows()

    if captured < 10:
        print("Not enough captures. Need at least 10.")
        return

    print("Computing calibration...")
    h, w = gray.shape
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )

    print(f"  RMS reprojection error: {ret:.4f}px  (< 1.0 is good)")
    np.savez(output_path,
             camera_matrix=mtx,
             dist_coefficients=dist)
    print(f"  Saved to: {output_path}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    run_calibration()
