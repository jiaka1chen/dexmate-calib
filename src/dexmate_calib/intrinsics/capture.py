from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import yaml

from dexmate_calib.boards.config import BoardProfile
from dexmate_calib.intrinsics.detector import CharucoDetector, Detection
from dexmate_calib.streaming.zed_stream import ZedStreamClient


@dataclass(frozen=True)
class CaptureSettings:
    output_root: Path
    max_samples: int = 40
    min_corners: int = 20
    min_coverage: float = 0.025
    min_sharpness: float = 80.0
    min_diversity_distance: float = 0.08
    cooldown_s: float = 0.8
    preview: bool = True
    auto_capture: bool = True


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _quality_reason(detection: Detection | None, settings: CaptureSettings) -> str | None:
    if detection is None:
        return "board_not_detected"
    if detection.corner_count < settings.min_corners:
        return f"corners<{settings.min_corners}"
    if detection.coverage_fraction < settings.min_coverage:
        return f"coverage<{settings.min_coverage:.3f}"
    if detection.sharpness < settings.min_sharpness:
        return f"sharpness<{settings.min_sharpness:.1f}"
    return None


def _write_manifest(
    path: Path,
    *,
    created_at_utc: str,
    status: str,
    board: BoardProfile,
    board_snapshot: Path,
    identity: dict | None,
    accepted: int,
    frame_index: int,
    settings: CaptureSettings,
) -> None:
    manifest = {
        "schema": "dexmate_calib.intrinsic_session.v1",
        "created_at_utc": created_at_utc,
        "updated_at_utc": _utc_now(),
        "status": status,
        "camera": {
            "name": "head_left",
            "zed_mode": "HD1200",
            "expected_resolution": {"width": 1920, "height": 1200},
            "image_view": "LEFT",
            "image_geometry": "rectified",
            "distortion_model": "none",
            **(identity or {}),
        },
        "streamer": {
            "host": "192.168.50.22",
            "port": 30000,
            "expected_command": (
                "./build/zed_streamer --clean --jpeg-quality 100 --max-fps 30 "
                "--resolution HD1200 --no-right --no-depth --no-pc --no-imu"
            ),
            "remote_process_verified": False,
        },
        "board": {
            "name": board.name,
            "profile_file": board_snapshot.name,
            "profile_sha256": board.sha256,
        },
        "capture": {
            "accepted_samples": accepted,
            "observed_stream_frames": frame_index,
            "settings": {
                "max_samples": settings.max_samples,
                "min_corners": settings.min_corners,
                "min_coverage": settings.min_coverage,
                "min_sharpness": settings.min_sharpness,
                "min_diversity_distance": settings.min_diversity_distance,
                "cooldown_s": settings.cooldown_s,
            },
        },
    }
    path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")


def capture_session(
    board: BoardProfile,
    client: ZedStreamClient,
    settings: CaptureSettings,
) -> Path:
    try:
        import cv2
        import numpy as np
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("OpenCV and NumPy are required for capture") from exc

    detector = CharucoDetector(board)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    session_dir = settings.output_root.expanduser().resolve() / f"{stamp}_head_left_HD1200"
    raw_dir = session_dir / "raw"
    preview_dir = session_dir / "previews"
    raw_dir.mkdir(parents=True, exist_ok=False)
    preview_dir.mkdir(parents=True, exist_ok=True)

    board_snapshot = session_dir / "board_profile.yaml"
    board_snapshot.write_text(yaml.safe_dump(board.data, sort_keys=False), encoding="utf-8")
    records_path = session_dir / "frames.jsonl"
    manifest_path = session_dir / "manifest.yaml"
    created_at_utc = _utc_now()

    signatures: list[np.ndarray] = []
    accepted = 0
    frame_index = 0
    last_capture_monotonic = -1e9
    identity: dict | None = None
    stop_requested = False

    _write_manifest(
        manifest_path,
        created_at_utc=created_at_utc,
        status="in_progress",
        board=board,
        board_snapshot=board_snapshot,
        identity=identity,
        accepted=accepted,
        frame_index=frame_index,
        settings=settings,
    )

    with records_path.open("w", encoding="utf-8") as records, client:
        while accepted < settings.max_samples and not stop_requested:
            frame = client.receive()
            frame_index += 1
            image = frame.decode_left_bgr()
            height, width = image.shape[:2]
            if identity is None:
                identity = {
                    "protocol": frame.protocol,
                    "camera_serial": frame.serial_number,
                    "width": width,
                    "height": height,
                    "channel_mask": frame.channel_mask,
                }

            detection = detector.detect(image)
            reason = _quality_reason(detection, settings)
            signature = detection.signature(width, height) if detection is not None else None
            if reason is None and signatures:
                nearest = min(float(np.linalg.norm(signature - old)) for old in signatures)
                if nearest < settings.min_diversity_distance:
                    reason = f"redundant<{settings.min_diversity_distance:.3f}"

            now = time.monotonic()
            eligible = reason is None and now - last_capture_monotonic >= settings.cooldown_s
            manual_capture = False
            display = detector.draw(image, detection)
            status = f"saved {accepted}/{settings.max_samples} | " + (
                "ready" if reason is None else reason
            )
            cv2.putText(
                display,
                status,
                (24, 42),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 220, 0) if reason is None else (0, 170, 255),
                2,
                cv2.LINE_AA,
            )
            if settings.preview:
                scale = min(1.0, 1280.0 / width, 800.0 / height)
                shown = cv2.resize(display, None, fx=scale, fy=scale) if scale < 1.0 else display
                cv2.imshow("Dexmate HD1200 intrinsic capture", shown)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    stop_requested = True
                elif key == ord(" "):
                    manual_capture = True

            should_save = eligible and (settings.auto_capture or manual_capture)
            if should_save and frame.left_jpeg is not None and detection is not None:
                filename = f"frame_{accepted:04d}_{frame.source_timestamp_ns}.jpg"
                image_path = raw_dir / filename
                image_path.write_bytes(frame.left_jpeg)
                preview_path = preview_dir / f"frame_{accepted:04d}.jpg"
                cv2.imwrite(str(preview_path), display)
                record = {
                    "sample_index": accepted,
                    "stream_frame_index": frame_index,
                    "image": str(image_path.relative_to(session_dir)),
                    "preview": str(preview_path.relative_to(session_dir)),
                    "source_timestamp_ns": frame.source_timestamp_ns,
                    "receive_timestamp_ns": frame.receive_timestamp_ns,
                    "protocol": frame.protocol,
                    "camera_serial": frame.serial_number,
                    "width": width,
                    "height": height,
                    "channel_mask": frame.channel_mask,
                    "corner_count": detection.corner_count,
                    "marker_count": detection.marker_count,
                    "coverage_fraction": detection.coverage_fraction,
                    "sharpness": detection.sharpness,
                    "centroid_xy": list(detection.centroid_xy),
                    "orientation_rad": detection.orientation_rad,
                }
                records.write(json.dumps(record, separators=(",", ":")) + "\n")
                records.flush()
                signatures.append(signature)
                accepted += 1
                last_capture_monotonic = now
                _write_manifest(
                    manifest_path,
                    created_at_utc=created_at_utc,
                    status="in_progress",
                    board=board,
                    board_snapshot=board_snapshot,
                    identity=identity,
                    accepted=accepted,
                    frame_index=frame_index,
                    settings=settings,
                )

    if settings.preview:
        cv2.destroyAllWindows()
    _write_manifest(
        manifest_path,
        created_at_utc=created_at_utc,
        status="complete" if accepted >= settings.max_samples else "stopped_by_user",
        board=board,
        board_snapshot=board_snapshot,
        identity=identity,
        accepted=accepted,
        frame_index=frame_index,
        settings=settings,
    )
    return session_dir
