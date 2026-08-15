from __future__ import annotations

import csv
import json
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from dexmate_calib.boards.config import load_board_profile
from dexmate_calib.intrinsics.detector import CharucoDetector


@dataclass(frozen=True)
class Observation:
    image_name: str
    object_points: np.ndarray
    image_points: np.ndarray


@dataclass(frozen=True)
class CalibrationFit:
    rms: float
    camera_matrix: np.ndarray
    distortion: np.ndarray
    rvecs: tuple[np.ndarray, ...]
    tvecs: tuple[np.ndarray, ...]
    per_view_errors: tuple[float, ...]


def _calibration_flags(cv2) -> int:
    return (
        cv2.CALIB_USE_INTRINSIC_GUESS
        | cv2.CALIB_ZERO_TANGENT_DIST
        | cv2.CALIB_FIX_K1
        | cv2.CALIB_FIX_K2
        | cv2.CALIB_FIX_K3
        | cv2.CALIB_FIX_K4
        | cv2.CALIB_FIX_K5
        | cv2.CALIB_FIX_K6
    )


def calibrate_observations(
    observations: Sequence[Observation], image_size: tuple[int, int]
) -> CalibrationFit:
    import cv2

    if len(observations) < 3:
        raise ValueError("At least 3 observations are required")
    width, height = image_size
    focal_guess = float(max(width, height))
    camera_matrix = np.array(
        [[focal_guess, 0.0, (width - 1) / 2], [0.0, focal_guess, (height - 1) / 2], [0, 0, 1]],
        dtype=np.float64,
    )
    distortion = np.zeros((8, 1), dtype=np.float64)
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        [np.asarray(obs.object_points, dtype=np.float32) for obs in observations],
        [np.asarray(obs.image_points, dtype=np.float32) for obs in observations],
        (width, height),
        camera_matrix,
        distortion,
        flags=_calibration_flags(cv2),
    )
    errors = []
    for obs, rvec, tvec in zip(observations, rvecs, tvecs):
        projected, _ = cv2.projectPoints(
            obs.object_points, rvec, tvec, camera_matrix, np.zeros((5, 1))
        )
        delta = projected.reshape(-1, 2) - obs.image_points.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(delta * delta, axis=1)))))
    return CalibrationFit(
        rms=float(rms),
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        distortion=np.zeros(5, dtype=np.float64),
        rvecs=tuple(np.asarray(v) for v in rvecs),
        tvecs=tuple(np.asarray(v) for v in tvecs),
        per_view_errors=tuple(errors),
    )


def _load_observations(session_dir: Path, detector: CharucoDetector):
    import cv2

    manifest = yaml.safe_load((session_dir / "manifest.yaml").read_text(encoding="utf-8"))
    camera = manifest["camera"]
    width, height = int(camera["width"]), int(camera["height"])
    if camera.get("zed_mode") != "HD1200" or (width, height) != (1920, 1200):
        raise ValueError(
            f"This calibration profile requires HD1200 1920x1200; session is {width}x{height}"
        )
    if camera.get("image_geometry") != "rectified":
        raise ValueError("This solver is only for rectified LEFT images")

    records = [
        json.loads(line)
        for line in (session_dir / "frames.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    observations: list[Observation] = []
    rejected: list[dict] = []
    for record in records:
        image_path = session_dir / record["image"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None or image.shape[1::-1] != (width, height):
            rejected.append({"image": record["image"], "reason": "unreadable_or_wrong_size"})
            continue
        detection = detector.detect(image)
        if detection is None or detection.corner_count < 12:
            rejected.append({"image": record["image"], "reason": "insufficient_corners"})
            continue
        object_points, image_points = detector.calibration_points(detection)
        observations.append(Observation(record["image"], object_points, image_points))
    return manifest, (width, height), observations, rejected


def _robust_fit(
    observations: list[Observation],
    image_size: tuple[int, int],
    max_view_error_px: float,
    max_rounds: int = 5,
) -> tuple[CalibrationFit, list[Observation], list[dict]]:
    active = list(observations)
    rejected: list[dict] = []
    for _ in range(max_rounds):
        fit = calibrate_observations(active, image_size)
        errors = np.asarray(fit.per_view_errors)
        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)))
        robust_limit = median + 3.5 * max(1.4826 * mad, 0.05)
        limit = min(max_view_error_px, robust_limit)
        bad = [index for index, error in enumerate(errors) if error > limit]
        if not bad or len(active) - len(bad) < 12:
            return fit, active, rejected
        bad_set = set(bad)
        for index in bad:
            rejected.append(
                {
                    "image": active[index].image_name,
                    "reason": "reprojection_outlier",
                    "error_px": float(errors[index]),
                    "threshold_px": limit,
                }
            )
        active = [obs for index, obs in enumerate(active) if index not in bad_set]
    return calibrate_observations(active, image_size), active, rejected


def _held_out_errors(
    train: list[Observation], held_out: list[Observation], image_size: tuple[int, int]
) -> tuple[CalibrationFit, list[float]]:
    import cv2

    fit = calibrate_observations(train, image_size)
    errors: list[float] = []
    for obs in held_out:
        ok, rvec, tvec = cv2.solvePnP(
            obs.object_points,
            obs.image_points,
            fit.camera_matrix,
            np.zeros((5, 1)),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            continue
        projected, _ = cv2.projectPoints(
            obs.object_points, rvec, tvec, fit.camera_matrix, np.zeros((5, 1))
        )
        delta = projected.reshape(-1, 2) - obs.image_points.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(delta * delta, axis=1)))))
    return fit, errors


def solve_session(
    session: str | Path,
    *,
    max_view_error_px: float = 0.8,
    min_views: int = 20,
) -> Path:
    session_dir = Path(session).expanduser().resolve()
    manifest_path = session_dir / "manifest.yaml"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"Session manifest not found: {manifest_path}")
    initial_manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    board_path = session_dir / initial_manifest["board"]["profile_file"]
    board = load_board_profile(board_path)
    if board.sha256 != initial_manifest["board"]["profile_sha256"]:
        raise ValueError("Session board profile hash does not match manifest")
    detector = CharucoDetector(board)
    manifest, image_size, observations, detection_rejections = _load_observations(
        session_dir, detector
    )
    if len(observations) < min_views:
        raise ValueError(f"Need at least {min_views} valid views; found {len(observations)}")

    fit, inliers, reprojection_rejections = _robust_fit(observations, image_size, max_view_error_px)
    if len(inliers) < min_views:
        raise ValueError(f"Only {len(inliers)} views remain after outlier rejection")

    held_indices = set(range(0, len(inliers), 5))
    held_out = [obs for index, obs in enumerate(inliers) if index in held_indices]
    train = [obs for index, obs in enumerate(inliers) if index not in held_indices]
    _, held_errors = _held_out_errors(train, held_out, image_size)

    split_a = inliers[::2]
    split_b = inliers[1::2]
    split_metrics = None
    if len(split_a) >= 6 and len(split_b) >= 6:
        fit_a = calibrate_observations(split_a, image_size)
        fit_b = calibrate_observations(split_b, image_size)
        split_metrics = {
            "fx_relative_difference": float(
                abs(fit_a.camera_matrix[0, 0] - fit_b.camera_matrix[0, 0])
                / ((fit_a.camera_matrix[0, 0] + fit_b.camera_matrix[0, 0]) / 2)
            ),
            "fy_relative_difference": float(
                abs(fit_a.camera_matrix[1, 1] - fit_b.camera_matrix[1, 1])
                / ((fit_a.camera_matrix[1, 1] + fit_b.camera_matrix[1, 1]) / 2)
            ),
            "principal_point_difference_px": float(
                np.linalg.norm(fit_a.camera_matrix[:2, 2] - fit_b.camera_matrix[:2, 2])
            ),
        }

    held_median = float(np.median(held_errors)) if held_errors else None
    stability_ok = split_metrics is not None and (
        split_metrics["fx_relative_difference"] < 0.005
        and split_metrics["fy_relative_difference"] < 0.005
        and split_metrics["principal_point_difference_px"] < 3.0
    )
    quality_gates = {
        "enough_views": len(inliers) >= 25,
        "rms_below_0_5_px": fit.rms < 0.5,
        "held_out_median_below_0_5_px": held_median is not None and held_median < 0.5,
        "split_stability": stability_ok,
    }

    results_dir = session_dir / "results"
    results_dir.mkdir(exist_ok=True)
    width, height = image_size
    result = {
        "schema": "dexmate_calib.intrinsics.v1",
        "camera": {
            "name": "head_left",
            "serial": manifest["camera"].get("camera_serial"),
            "zed_mode": "HD1200",
            "width": width,
            "height": height,
            "image_view": "LEFT",
            "image_geometry": "rectified",
        },
        "model": {
            "type": "pinhole",
            "distortion_model": "none",
            "K": fit.camera_matrix.tolist(),
            "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
            "fx": float(fit.camera_matrix[0, 0]),
            "fy": float(fit.camera_matrix[1, 1]),
            "cx": float(fit.camera_matrix[0, 2]),
            "cy": float(fit.camera_matrix[1, 2]),
        },
        "board": {"name": board.name, "profile_sha256": board.sha256},
        "quality": {
            "rms_reprojection_error_px": fit.rms,
            "views_detected": len(observations),
            "views_used": len(inliers),
            "held_out_views": len(held_errors),
            "held_out_median_error_px": held_median,
            "held_out_max_error_px": max(held_errors) if held_errors else None,
            "split_stability": split_metrics,
            "gates": quality_gates,
            "all_gates_pass": all(quality_gates.values()),
            "thresholds": {
                "max_view_error_px": max_view_error_px,
                "min_views": min_views,
            },
        },
        "rejected_views": detection_rejections + reprojection_rejections,
    }
    stem = f"intrinsics_head_left_HD1200_{width}x{height}"
    json_path = results_dir / f"{stem}.json"
    yaml_path = results_dir / f"{stem}.yaml"
    json_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    yaml_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    np.save(results_dir / "K.npy", fit.camera_matrix)

    with (results_dir / "reprojection_errors.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.writer(handle)
        writer.writerow(["image", "rms_error_px"])
        for obs, error in zip(inliers, fit.per_view_errors):
            writer.writerow([obs.image_name, f"{error:.9f}"])
    return yaml_path
