from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

import yaml

from dexmate_calib.boards.config import (
    available_board_profiles,
    load_board_profile,
    resolve_board_profile,
)
from dexmate_calib.diagnostics.doctor import doctor_network, doctor_stream, print_report
from dexmate_calib.intrinsics.capture import CaptureSettings, capture_session
from dexmate_calib.intrinsics.detector import CharucoDetector
from dexmate_calib.intrinsics.solve import solve_session
from dexmate_calib.remote.streamer import RemoteStreamerManager
from dexmate_calib.streaming.zed_stream import ZedStreamClient


def _optional_serial(value: str) -> int | None:
    parsed = int(value)
    return None if parsed == 0 else parsed


def _cmd_board(args: argparse.Namespace) -> int:
    if args.board_command == "list":
        for profile in available_board_profiles():
            aliases = ", ".join(profile.data.get("aliases", [])) or "-"
            print(f"{profile.name}\taliases={aliases}\tsha256={profile.sha256[:12]}")
        return 0
    if args.board_command == "show":
        profile = resolve_board_profile(args.board)
        print(yaml.safe_dump(profile.data, sort_keys=False), end="")
        print(f"sha256: {profile.sha256}")
        return 0
    if args.board_command == "validate":
        profile = load_board_profile(args.path)
        profile.create_opencv_board()
        print(f"OK: {profile.name} ({profile.sha256})")
        return 0
    if args.board_command == "verify":
        import cv2

        profile = resolve_board_profile(args.board)
        image = cv2.imread(str(Path(args.image).expanduser()), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(f"Cannot read image: {args.image}")
        detection = CharucoDetector(profile).detect(image)
        result = {
            "ok": detection is not None,
            "board": profile.name,
            "expected_markers": profile.expected_marker_count,
            "expected_corners": profile.expected_corner_count,
            "detected_markers": detection.marker_count if detection else 0,
            "detected_corners": detection.corner_count if detection else 0,
        }
        print(json.dumps(result, indent=2))
        return 0 if detection is not None else 2
    raise AssertionError(args.board_command)


def _cmd_doctor(args: argparse.Namespace) -> int:
    if args.doctor_command == "network":
        report = doctor_network()
        print_report(report)
        nano_ssh_ok = report["nano_ssh_direct"]["ok"] or report["nano_ssh_proxy"]["ok"]
        return 0 if nano_ssh_ok else 2
    report = doctor_stream(
        args.host,
        args.port,
        frames=args.frames,
        expected_serial=args.expected_serial,
        expected_width=args.width,
        expected_height=args.height,
    )
    print_report(report)
    return 0


def _cmd_intrinsics(args: argparse.Namespace) -> int:
    if args.intrinsics_command in {"capture", "quickstart"}:
        if args.manual and args.no_preview:
            raise ValueError("--manual requires the preview window; remove --no-preview")
        if args.detect_fps < 0:
            raise ValueError("--detect-fps must be >= 0")
        board = resolve_board_profile(args.board)
        client = ZedStreamClient(
            args.host,
            args.port,
            expected_serial=args.expected_serial,
            expected_width=1920,
            expected_height=1200,
        )
        settings = CaptureSettings(
            output_root=Path(args.output),
            max_samples=args.samples,
            min_corners=args.min_corners,
            min_coverage=args.min_coverage,
            min_sharpness=args.min_sharpness,
            min_grid_rows=args.min_grid_rows,
            min_grid_cols=args.min_grid_cols,
            min_board_bbox_fraction=args.min_board_bbox_fraction,
            min_diversity_distance=args.min_diversity,
            cooldown_s=args.cooldown,
            detection_fps=args.detect_fps,
            preview=not args.no_preview,
            auto_capture=not args.manual,
        )
        if args.intrinsics_command == "capture":
            session = capture_session(board, client, settings)
            print(session)
            return 0

        manager = RemoteStreamerManager(
            route_preference=args.ssh_route,
            use_sudo=not args.no_sudo,
        )
        started_here = False
        try:
            started_here = manager.ensure_started(
                args.host,
                args.port,
                timeout_s=args.startup_timeout,
            )
            stream_report = doctor_stream(
                args.host,
                args.port,
                frames=args.preflight_frames,
                expected_serial=args.expected_serial,
                expected_width=1920,
                expected_height=1200,
            )
            print("Stream preflight passed:")
            print_report(stream_report)
            settings = replace(
                settings,
                streamer_started_by_quickstart=started_here,
                streamer_ssh_route=manager.select_route().name,
            )
            session = capture_session(board, client, settings)
        finally:
            if manager.process is not None:
                manager.stop_attached(host=args.host, port=args.port)
        print(session)
        return 0
    result = solve_session(
        args.session,
        max_view_error_px=args.max_view_error,
        min_views=args.min_views,
        max_views=args.max_views,
        cross_validation_folds=args.cross_validation_folds,
    )
    print(result)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="dexcalib")
    sub = parser.add_subparsers(dest="command", required=True)

    board = sub.add_parser("board", help="Inspect and verify ChArUco board profiles")
    board_sub = board.add_subparsers(dest="board_command", required=True)
    board_sub.add_parser("list")
    show = board_sub.add_parser("show")
    show.add_argument("board")
    validate = board_sub.add_parser("validate")
    validate.add_argument("path")
    verify = board_sub.add_parser("verify")
    verify.add_argument("--board", default="dexmate-10x7")
    verify.add_argument("--image", required=True)

    doctor = sub.add_parser("doctor", help="Read-only network and stream checks")
    doctor_sub = doctor.add_subparsers(dest="doctor_command", required=True)
    doctor_sub.add_parser("network")
    stream = doctor_sub.add_parser("stream")
    stream.add_argument("--host", default="192.168.50.22")
    stream.add_argument("--port", type=int, default=30000)
    stream.add_argument("--frames", type=int, default=20)
    stream.add_argument("--expected-serial", type=_optional_serial, default=59595115)
    stream.add_argument("--width", type=int, default=1920)
    stream.add_argument("--height", type=int, default=1200)

    intrinsics = sub.add_parser("intrinsics", help="Capture and solve head intrinsics")
    intrinsics_sub = intrinsics.add_subparsers(dest="intrinsics_command", required=True)
    capture = intrinsics_sub.add_parser("capture")
    _add_capture_arguments(capture)
    quickstart = intrinsics_sub.add_parser(
        "quickstart", help="SSH-start streamer, validate it, capture, then stop it"
    )
    _add_capture_arguments(quickstart)
    quickstart.add_argument(
        "--ssh-route",
        choices=("auto", "direct", "proxy"),
        default="auto",
    )
    quickstart.add_argument(
        "--no-sudo",
        action="store_true",
        help="Run streamer as dexmate-nano instead of prompting for remote sudo",
    )
    quickstart.add_argument("--startup-timeout", type=float, default=30.0)
    quickstart.add_argument("--preflight-frames", type=int, default=10)
    solve = intrinsics_sub.add_parser("solve")
    solve.add_argument("session")
    solve.add_argument("--min-views", type=int, default=20)
    solve.add_argument("--max-view-error", type=float, default=0.8)
    solve.add_argument("--max-views", type=int, default=40)
    solve.add_argument("--cross-validation-folds", type=int, default=5)
    return parser


def _add_capture_arguments(command: argparse.ArgumentParser) -> None:
    command.add_argument("--board", default="dexmate-10x7")
    command.add_argument("--host", default="192.168.50.22")
    command.add_argument("--port", type=int, default=30000)
    command.add_argument("--expected-serial", type=_optional_serial, default=59595115)
    command.add_argument("--output", default="calibration_data/head_left")
    command.add_argument("--samples", type=int, default=40)
    command.add_argument("--min-corners", type=int, default=20)
    command.add_argument("--min-coverage", type=float, default=0.025)
    command.add_argument("--min-sharpness", type=float, default=80.0)
    command.add_argument("--min-grid-rows", type=int, default=3)
    command.add_argument("--min-grid-cols", type=int, default=3)
    command.add_argument(
        "--min-board-bbox",
        dest="min_board_bbox_fraction",
        type=float,
        default=0.12,
    )
    command.add_argument("--min-diversity", type=float, default=0.08)
    command.add_argument("--cooldown", type=float, default=0.8)
    command.add_argument(
        "--detect-fps",
        type=float,
        default=10.0,
        help="Auto-mode ChArUco processing rate; 0 processes every stream frame",
    )
    command.add_argument("--manual", action="store_true")
    command.add_argument("--no-preview", action="store_true")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "board":
            return _cmd_board(args)
        if args.command == "doctor":
            return _cmd_doctor(args)
        if args.command == "intrinsics":
            return _cmd_intrinsics(args)
        raise AssertionError(args.command)
    except (ConnectionError, OSError, RuntimeError, TypeError, ValueError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
