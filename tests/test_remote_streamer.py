from __future__ import annotations

import signal
import subprocess
from unittest.mock import Mock

from dexmate_calib.cli import build_parser
from dexmate_calib.remote.streamer import RemoteStreamerManager, SSHRoute


def completed(code: int, stderr: str = "") -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(["ssh"], code, stdout="", stderr=stderr)


def test_fixed_remote_command_is_hd1200_left_only_without_clean() -> None:
    manager = RemoteStreamerManager()
    command = manager._remote_streamer_command()
    assert "sudo" in command
    assert "--resolution HD1200" in command
    assert "--no-right" in command
    assert "--no-depth" in command
    assert "--no-pc" in command
    assert "--no-imu" in command
    assert "--clean" not in command
    assert "systemctl" not in command


def test_no_sudo_mode_is_supported() -> None:
    command = RemoteStreamerManager(use_sudo=False)._remote_streamer_command()
    assert "sudo" not in command
    assert "exec /home/dexmate-nano/zed_stream/build/zed_streamer" in command


def test_auto_route_falls_back_to_thor_proxy(monkeypatch) -> None:
    manager = RemoteStreamerManager()

    def fake_probe(route: SSHRoute):
        return completed(0) if route.name == "proxy" else completed(255, "direct failed")

    monkeypatch.setattr(manager, "_probe", fake_probe)
    route = manager.select_route()
    assert route.name == "proxy"
    assert route.proxy_command is not None
    assert "BatchMode=yes" in route.proxy_command
    assert "-W %h:%p dexmate@192.168.50.20" in route.proxy_command


def test_existing_stream_is_not_started_or_owned(monkeypatch) -> None:
    manager = RemoteStreamerManager(route_preference="direct")
    monkeypatch.setattr(manager, "select_route", lambda: manager.direct_route)
    monkeypatch.setattr(manager, "port_is_open", lambda *_args, **_kwargs: True)
    start = Mock()
    monkeypatch.setattr(manager, "start_attached", start)
    assert manager.ensure_started("192.168.50.22", 30000) is False
    start.assert_not_called()


def test_attached_start_uses_tty_and_keeps_local_process(monkeypatch) -> None:
    manager = RemoteStreamerManager(route_preference="direct")
    manager.route = manager.direct_route
    child = Mock()
    popen = Mock(return_value=child)
    monkeypatch.setattr(subprocess, "Popen", popen)
    manager.start_attached()
    command = popen.call_args.args[0]
    assert "-tt" in command
    assert "BatchMode=yes" in command
    assert "--resolution HD1200" in command[-1]
    assert manager.process is child


def test_stop_signals_only_managed_ssh_process() -> None:
    manager = RemoteStreamerManager()
    child = Mock()
    child.poll.return_value = None
    child.wait.return_value = 0
    manager.process = child
    manager.stop_attached()
    child.send_signal.assert_called_once_with(signal.SIGINT)
    assert manager.process is None


def test_quickstart_cli_defaults_to_safe_fixed_profile() -> None:
    args = build_parser().parse_args(["intrinsics", "quickstart"])
    assert args.board == "dexmate-10x7"
    assert args.host == "192.168.50.22"
    assert args.port == 30000
    assert args.expected_serial == 59595115
    assert args.ssh_route == "auto"
    assert args.no_sudo is False
