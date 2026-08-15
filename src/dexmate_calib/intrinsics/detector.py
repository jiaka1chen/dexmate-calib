from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from dexmate_calib.boards.config import BoardProfile


@dataclass(frozen=True)
class Detection:
    charuco_corners: np.ndarray
    charuco_ids: np.ndarray
    marker_count: int
    corner_count: int
    coverage_fraction: float
    sharpness: float
    centroid_xy: tuple[float, float]
    orientation_rad: float

    def signature(self, width: int, height: int) -> np.ndarray:
        return np.array(
            [
                self.centroid_xy[0] / width,
                self.centroid_xy[1] / height,
                math.sqrt(max(0.0, self.coverage_fraction)),
                0.5 * math.sin(2.0 * self.orientation_rad),
                0.5 * math.cos(2.0 * self.orientation_rad),
            ],
            dtype=np.float64,
        )


class CharucoDetector:
    def __init__(self, profile: BoardProfile) -> None:
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("opencv-contrib-python is required") from exc
        self.cv2 = cv2
        self.profile = profile
        self.board, self.dictionary = profile.create_opencv_board()
        detector_params = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "CharucoParameters"):
            charuco_params = cv2.aruco.CharucoParameters()
            self.detector = cv2.aruco.CharucoDetector(self.board, charuco_params, detector_params)
        else:  # pragma: no cover - OpenCV < 4.7 compatibility
            self.detector = None
            self.aruco_detector = cv2.aruco.ArucoDetector(self.dictionary, detector_params)

    def detect(self, image_bgr: np.ndarray) -> Detection | None:
        cv2 = self.cv2
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, marker_corners, marker_ids = self.detector.detectBoard(gray)
        else:  # pragma: no cover
            marker_corners, marker_ids, _ = self.aruco_detector.detectMarkers(gray)
            if marker_ids is None:
                return None
            _, corners, ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, self.board
            )
        if corners is None or ids is None or len(corners) < 4:
            return None

        points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
        ids_flat = np.asarray(ids, dtype=np.int32).reshape(-1)
        order = np.argsort(ids_flat)
        points = points[order]
        ids_flat = ids_flat[order]
        x_min, y_min = points.min(axis=0)
        x_max, y_max = points.max(axis=0)
        height, width = gray.shape
        coverage = float(max(0.0, x_max - x_min) * max(0.0, y_max - y_min) / (width * height))

        pad = 8
        x0 = max(0, int(x_min) - pad)
        y0 = max(0, int(y_min) - pad)
        x1 = min(width, int(x_max) + pad + 1)
        y1 = min(height, int(y_max) + pad + 1)
        roi = gray[y0:y1, x0:x1]
        sharpness = float(cv2.Laplacian(roi, cv2.CV_64F).var()) if roi.size else 0.0

        centered = points - points.mean(axis=0, keepdims=True)
        covariance = centered.T @ centered
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        orientation = float(math.atan2(float(axis[1]), float(axis[0])))

        return Detection(
            charuco_corners=points.reshape(-1, 1, 2),
            charuco_ids=ids_flat.reshape(-1, 1),
            marker_count=0 if marker_ids is None else len(marker_ids),
            corner_count=len(points),
            coverage_fraction=coverage,
            sharpness=sharpness,
            centroid_xy=(float(points[:, 0].mean()), float(points[:, 1].mean())),
            orientation_rad=orientation,
        )

    def draw(self, image_bgr: np.ndarray, detection: Detection | None) -> np.ndarray:
        output = image_bgr.copy()
        if detection is not None:
            self.cv2.aruco.drawDetectedCornersCharuco(
                output, detection.charuco_corners, detection.charuco_ids
            )
        return output

    def calibration_points(self, detection: Detection) -> tuple[np.ndarray, np.ndarray]:
        chessboard = np.asarray(self.board.getChessboardCorners(), dtype=np.float32).reshape(-1, 3)
        ids = detection.charuco_ids.reshape(-1)
        if np.any(ids < 0) or np.any(ids >= len(chessboard)):
            raise ValueError("Detected ChArUco corner ID is outside board geometry")
        return chessboard[ids].copy(), detection.charuco_corners.reshape(-1, 2).copy()
