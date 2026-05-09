#!/usr/bin/env python3
"""
External watchdog for the ZED BodyFusion sample.

The watchdog intentionally treats ZED_BodyFusion as a black box: it launches the
process, watches for unexpected exits, exposes TCP/UDP control commands, and
derives camera health from the application's existing console output.
"""

from __future__ import annotations

import argparse
import configparser
import ctypes
import datetime as _dt
import json
import os
import pathlib
import re
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time
from typing import Any, Optional


SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "watchdog_config.json"

APP_RESTART_COMMAND = "restart"
SYSTEM_RESTART_COMMANDS = {"reboot", "system_restart", "pc_restart"}
SYSTEM_COMMANDS = {"sleep", "shutdown", "wake", *SYSTEM_RESTART_COMMANDS}
READ_ONLY_COMMANDS = {"health", "status", "ping"}

SUBSCRIBED_RE = re.compile(r"\[Startup\]\s+ZED\s+(\d+)\s+subscribed\.")
ALL_CAMERAS_RE = re.compile(r"\[Startup\]\s+All\s+(\d+)\s+configured cameras are available\.")
DEGRADED_RE = re.compile(r"\[Startup\]\s+Running in degraded mode with\s+(\d+)/(\d+)\s+configured cameras\.")
LOST_RE = re.compile(r"\[Watchdog\]\s+Lost ZED\s+(\d+)\s+after")
CALIBRATION_RE = re.compile(r"Using .* calibration file .*:\s+(.+)$")


def now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def coerce_path(value: str, base: pathlib.Path) -> pathlib.Path:
    path = pathlib.Path(value).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def is_elevated() -> bool:
    if os.name == "nt":
        try:
            return bool(ctypes.windll.shell32.IsUserAnAdmin())
        except Exception:
            return False
    return hasattr(os, "geteuid") and os.geteuid() == 0


def load_json_config(path: pathlib.Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def parse_ini_calibration_file(config_path: pathlib.Path) -> Optional[pathlib.Path]:
    parser = configparser.ConfigParser()
    parser.read(config_path, encoding="utf-8")
    if not parser.has_option("input", "calibration_file"):
        return None
    raw = parser.get("input", "calibration_file").strip()
    if not raw:
        return None
    return coerce_path(raw, config_path.parent)


def app_search_roots(executable: pathlib.Path) -> list[pathlib.Path]:
    roots: list[pathlib.Path] = []
    current = executable.parent
    for _ in range(3):
        if current and current not in roots:
            roots.append(current)
        if current.parent == current:
            break
        current = current.parent
    return roots


def find_default_app_config(executable: pathlib.Path) -> Optional[pathlib.Path]:
    for root in app_search_roots(executable):
        for name in ("zed_bodyfusion.ini", "bodyfusion.ini"):
            candidate = root / name
            if candidate.is_file():
                return candidate
    return None


def find_latest_calibration(executable: pathlib.Path) -> Optional[pathlib.Path]:
    newest_path: Optional[pathlib.Path] = None
    newest_mtime = -1.0
    for root in app_search_roots(executable):
        if not root.is_dir():
            continue
        for candidate in root.glob("calib_*.json"):
            if not candidate.is_file():
                continue
            mtime = candidate.stat().st_mtime
            if mtime > newest_mtime:
                newest_path = candidate
                newest_mtime = mtime
    return newest_path


def count_cameras_in_calibration(path: pathlib.Path) -> Optional[int]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None

    serials: set[int] = set()

    def walk(value: Any) -> None:
        if isinstance(value, dict):
            if "serial_number" in value:
                try:
                    serials.add(int(value["serial_number"]))
                except (TypeError, ValueError):
                    pass
            for child in value.values():
                walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)

    walk(data)
    return len(serials) if serials else None


class WatchdogConfig:
    def __init__(self, raw: dict[str, Any], config_path: pathlib.Path):
        base = config_path.parent
        default_exe = (SCRIPT_DIR.parent / "build" / "Release" / "ZED_BodyFusion.exe").resolve()

        self.app_executable = coerce_path(str(raw.get("app_executable", default_exe)), base)
        self.app_args = [str(arg) for arg in raw.get("app_args", [])]
        workdir = raw.get("app_working_dir")
        self.app_working_dir = coerce_path(str(workdir), base) if workdir else self.app_executable.parent

        self.autostart = bool(raw.get("autostart", True))
        self.auto_restart = bool(raw.get("auto_restart", True))
        self.restart_delay_seconds = float(raw.get("restart_delay_seconds", 3))
        self.stop_grace_seconds = float(raw.get("stop_grace_seconds", 8))

        self.bind_host = str(raw.get("bind_host", "127.0.0.1"))
        self.tcp_port = int(raw.get("tcp_port", 8765))
        self.udp_port = int(raw.get("udp_port", 8765))
        self.auth_token = str(raw.get("auth_token", ""))
        self.auth_required_for_health = bool(raw.get("auth_required_for_health", False))
        self.require_elevation_for_system_commands = bool(raw.get("require_elevation_for_system_commands", True))

        self.wake_macs = [str(mac) for mac in raw.get("wake_macs", [])]
        self.wake_broadcast = str(raw.get("wake_broadcast", "255.255.255.255"))
        self.wake_port = int(raw.get("wake_port", 9))

        log_dir_raw = raw.get("log_dir", SCRIPT_DIR / "logs")
        self.log_dir = coerce_path(str(log_dir_raw), base)

    def to_dict(self) -> dict[str, Any]:
        return {
            "app_executable": str(self.app_executable),
            "app_args": self.app_args,
            "app_working_dir": str(self.app_working_dir),
            "autostart": self.autostart,
            "auto_restart": self.auto_restart,
            "bind_host": self.bind_host,
            "tcp_port": self.tcp_port,
            "udp_port": self.udp_port,
            "log_dir": str(self.log_dir),
        }


class BodyFusionSupervisor:
    def __init__(self, config: WatchdogConfig):
        self.config = config
        self.lock = threading.RLock()
        self.process: Optional[subprocess.Popen[str]] = None
        self.monitor_stop = threading.Event()
        self.desired_running = config.autostart
        self.restart_count = 0
        self.started_at: Optional[float] = None
        self.last_exit_code: Optional[int] = None
        self.last_exit_at: Optional[str] = None
        self.last_output_line = ""
        self.last_output_at: Optional[str] = None
        self.expected_cameras = self.infer_expected_cameras()
        self.subscribed_cameras: Optional[int] = None
        self.subscribed_serials: set[str] = set()
        self.lost_serials: set[str] = set()
        self.last_camera_report = ""
        self.elevated = is_elevated()
        self.monitor_thread = threading.Thread(target=self._monitor_loop, name="process-monitor", daemon=True)

    def start_monitoring(self) -> None:
        self.monitor_thread.start()
        if self.config.autostart:
            self.start_app(reason="autostart")

    def shutdown(self) -> None:
        self.monitor_stop.set()
        self.stop_app(reason="watchdog_shutdown")

    def infer_expected_cameras(self) -> Optional[int]:
        calibration = self.infer_calibration_file()
        if calibration:
            return count_cameras_in_calibration(calibration)
        return None

    def infer_calibration_file(self) -> Optional[pathlib.Path]:
        if self.config.app_args:
            first = coerce_path(self.config.app_args[0], self.config.app_working_dir)
            suffix = first.suffix.lower()
            if suffix == ".json" and first.is_file():
                return first
            if suffix in {".ini", ".cfg", ".conf"} and first.is_file():
                return parse_ini_calibration_file(first) or find_latest_calibration(self.config.app_executable)

        app_config = find_default_app_config(self.config.app_executable)
        if app_config:
            calibration = parse_ini_calibration_file(app_config)
            if calibration:
                return calibration
        return find_latest_calibration(self.config.app_executable)

    def _reset_run_health(self) -> None:
        self.started_at = time.time()
        self.last_exit_code = None
        self.last_exit_at = None
        self.subscribed_cameras = None
        self.subscribed_serials.clear()
        self.lost_serials.clear()
        self.last_camera_report = ""
        self.expected_cameras = self.infer_expected_cameras()

    def start_app(self, reason: str = "api") -> dict[str, Any]:
        with self.lock:
            if self.process and self.process.poll() is None:
                return self.response(True, "already_running", reason=reason, **self.health_payload())
            if not self.config.app_executable.exists():
                return self.response(False, "app_executable_not_found", path=str(self.config.app_executable))

            self.config.log_dir.mkdir(parents=True, exist_ok=True)
            timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = self.config.log_dir / f"bodyfusion_{timestamp}.log"
            log_handle = log_path.open("a", encoding="utf-8", buffering=1)

            creationflags = 0
            if os.name == "nt":
                creationflags = subprocess.CREATE_NEW_PROCESS_GROUP

            command = [str(self.config.app_executable), *self.config.app_args]
            try:
                self.process = subprocess.Popen(
                    command,
                    cwd=str(self.config.app_working_dir),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    bufsize=1,
                    creationflags=creationflags,
                )
            except Exception as exc:
                log_handle.close()
                self.process = None
                return self.response(False, "start_failed", error=str(exc))

            self.desired_running = True
            self._reset_run_health()
            threading.Thread(
                target=self._read_process_output,
                args=(self.process, log_handle, log_path),
                name="process-output",
                daemon=True,
            ).start()
            return self.response(True, "started", reason=reason, log_file=str(log_path), **self.health_payload())

    def stop_app(self, reason: str = "api") -> dict[str, Any]:
        with self.lock:
            self.desired_running = False
            process = self.process
            if not process or process.poll() is not None:
                return self.response(True, "already_stopped", reason=reason, **self.health_payload())

        self._terminate_process(process)
        with self.lock:
            return self.response(True, "stopped", reason=reason, **self.health_payload())

    def restart_app(self) -> dict[str, Any]:
        self.stop_app(reason="restart")
        result = self.start_app(reason="restart")
        if result.get("ok"):
            with self.lock:
                self.restart_count += 1
        return result

    def _terminate_process(self, process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        try:
            if os.name == "nt":
                os.kill(process.pid, signal.CTRL_BREAK_EVENT)
            else:
                process.send_signal(signal.SIGINT)
            process.wait(timeout=self.config.stop_grace_seconds)
            return
        except Exception:
            pass

        try:
            process.terminate()
            process.wait(timeout=self.config.stop_grace_seconds)
            return
        except Exception:
            pass

        try:
            process.kill()
            process.wait(timeout=2)
        except Exception:
            pass

    def _monitor_loop(self) -> None:
        while not self.monitor_stop.is_set():
            time.sleep(1)
            with self.lock:
                process = self.process
                desired = self.desired_running
                auto_restart = self.config.auto_restart

            if not process:
                continue

            exit_code = process.poll()
            if exit_code is None:
                continue

            with self.lock:
                if self.process is process:
                    self.last_exit_code = exit_code
                    self.last_exit_at = now_iso()
                    self.process = None

            if desired and auto_restart and not self.monitor_stop.is_set():
                time.sleep(max(0.0, self.config.restart_delay_seconds))
                with self.lock:
                    if not self.desired_running or self.process is not None:
                        continue
                    self.restart_count += 1
                self.start_app(reason="auto_restart")

    def _read_process_output(
        self,
        process: subprocess.Popen[str],
        log_handle: Any,
        log_path: pathlib.Path,
    ) -> None:
        try:
            assert process.stdout is not None
            for line in process.stdout:
                log_handle.write(line)
                self._parse_output_line(line.rstrip("\r\n"))
        finally:
            try:
                log_handle.close()
            except Exception:
                pass
            with self.lock:
                self.last_camera_report = self.last_camera_report or f"log ended: {log_path}"

    def _parse_output_line(self, line: str) -> None:
        if not line:
            return
        with self.lock:
            self.last_output_line = line
            self.last_output_at = now_iso()

            calibration_match = CALIBRATION_RE.search(line)
            if calibration_match:
                path = pathlib.Path(calibration_match.group(1).strip())
                count = count_cameras_in_calibration(path)
                if count is not None:
                    self.expected_cameras = count

            if "[Recovery] Restarting fusion session" in line:
                self.subscribed_cameras = None
                self.subscribed_serials.clear()
                self.lost_serials.clear()
                self.last_camera_report = "internal fusion session restart"

            subscribed_match = SUBSCRIBED_RE.search(line)
            if subscribed_match:
                serial = subscribed_match.group(1)
                self.subscribed_serials.add(serial)
                self.lost_serials.discard(serial)
                self.subscribed_cameras = len(self.subscribed_serials)
                self.last_camera_report = f"{self.subscribed_cameras}/{self.expected_cameras or '?'} subscribed"

            all_match = ALL_CAMERAS_RE.search(line)
            if all_match:
                count = int(all_match.group(1))
                self.subscribed_cameras = count
                self.expected_cameras = count
                self.last_camera_report = f"all {count} configured cameras available"

            degraded_match = DEGRADED_RE.search(line)
            if degraded_match:
                self.subscribed_cameras = int(degraded_match.group(1))
                self.expected_cameras = int(degraded_match.group(2))
                self.last_camera_report = f"degraded: {self.subscribed_cameras}/{self.expected_cameras} cameras"

            lost_match = LOST_RE.search(line)
            if lost_match:
                serial = lost_match.group(1)
                self.lost_serials.add(serial)
                self.subscribed_serials.discard(serial)
                if self.subscribed_cameras is not None:
                    self.subscribed_cameras = max(0, self.subscribed_cameras - 1)
                self.last_camera_report = f"lost camera {serial}"

    def health_payload(self) -> dict[str, Any]:
        process = self.process
        running = bool(process and process.poll() is None)
        uptime = round(time.time() - self.started_at, 3) if running and self.started_at else None
        app_state = "running" if running else "stopped"
        if not running and self.last_exit_code is not None:
            app_state = "exited"
        return {
            "watchdog": "running",
            "app": app_state,
            "pid": process.pid if running and process else None,
            "uptime_seconds": uptime,
            "desired_running": self.desired_running,
            "auto_restart": self.config.auto_restart,
            "restart_count": self.restart_count,
            "last_exit_code": self.last_exit_code,
            "last_exit_at": self.last_exit_at,
            "expected_cameras": self.expected_cameras,
            "subscribed_cameras": self.subscribed_cameras,
            "lost_cameras": sorted(self.lost_serials),
            "last_camera_report": self.last_camera_report,
            "last_output": self.last_output_line,
            "last_output_at": self.last_output_at,
            "elevated": self.elevated,
            "tcp": {"host": self.config.bind_host, "port": self.config.tcp_port},
            "udp": {"host": self.config.bind_host, "port": self.config.udp_port},
        }

    def response(self, ok: bool, status: str, **extra: Any) -> dict[str, Any]:
        payload = {"ok": ok, "status": status, "timestamp": now_iso()}
        payload.update(extra)
        return payload

    def handle_command(self, request: dict[str, Any]) -> dict[str, Any]:
        command = str(request.get("command", "")).strip().lower()
        token = str(request.get("token", ""))
        args = request.get("args", [])
        if not command:
            return self.response(False, "missing_command")

        needs_auth = command not in READ_ONLY_COMMANDS or self.config.auth_required_for_health
        if self.config.auth_token and needs_auth and token != self.config.auth_token:
            return self.response(False, "unauthorized")

        if command in {"health", "status"}:
            return self.response(True, "ok", **self.health_payload())
        if command == "ping":
            return self.response(True, "pong", **self.health_payload())
        if command == "start":
            return self.start_app()
        if command == "stop":
            return self.stop_app()
        if command == APP_RESTART_COMMAND:
            return self.restart_app()
        if command in SYSTEM_COMMANDS:
            return self.run_system_command(command, args)
        return self.response(False, "unknown_command", command=command)

    def run_system_command(self, command: str, args: Any) -> dict[str, Any]:
        if self.config.require_elevation_for_system_commands and not self.elevated:
            return self.response(False, "admin_required", command=command)

        if command == "sleep":
            return self._run_power_command(["rundll32.exe", "powrprof.dll,SetSuspendState", "0,1,0"], command)
        if command == "shutdown":
            return self._run_power_command(["shutdown.exe", "/s", "/t", "0"], command)
        if command in SYSTEM_RESTART_COMMANDS:
            return self._run_power_command(["shutdown.exe", "/r", "/t", "0"], command)
        if command == "wake":
            macs = [str(item) for item in args] if isinstance(args, list) and args else self.config.wake_macs
            return self._wake_on_lan(macs)
        return self.response(False, "unknown_system_command", command=command)

    def _run_power_command(self, command_line: list[str], command: str) -> dict[str, Any]:
        if os.name != "nt":
            return self.response(False, "unsupported_platform", command=command, platform=os.name)
        try:
            subprocess.Popen(command_line, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            return self.response(False, "system_command_failed", command=command, error=str(exc))
        return self.response(True, "system_command_started", command=command)

    def _wake_on_lan(self, macs: list[str]) -> dict[str, Any]:
        if not macs:
            return self.response(False, "wake_requires_mac", detail="Configure wake_macs or pass MAC addresses in JSON args.")
        sent: list[str] = []
        for mac in macs:
            try:
                packet = build_magic_packet(mac)
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    sock.sendto(packet, (self.config.wake_broadcast, self.config.wake_port))
            except Exception as exc:
                return self.response(False, "wake_failed", mac=mac, error=str(exc))
            sent.append(mac)
        return self.response(True, "wake_sent", macs=sent)


def build_magic_packet(mac: str) -> bytes:
    clean = re.sub(r"[^0-9A-Fa-f]", "", mac)
    if len(clean) != 12:
        raise ValueError(f"Invalid MAC address: {mac}")
    payload = bytes.fromhex(clean)
    return b"\xff" * 6 + payload * 16


def parse_request(data: bytes, auth_token: str) -> tuple[dict[str, Any], bool]:
    text = data.decode("utf-8", errors="replace").strip()
    if not text:
        return {"command": ""}, False

    is_http = text.startswith(("GET ", "POST "))
    if is_http:
        first_line = text.splitlines()[0]
        parts = first_line.split()
        path = parts[1] if len(parts) > 1 else "/health"
        command = path.lstrip("/").split("?", 1)[0] or "health"
        return {"command": command}, True

    if text.startswith("{"):
        try:
            payload = json.loads(text)
            if "cmd" in payload and "command" not in payload:
                payload["command"] = payload["cmd"]
            return payload, False
        except json.JSONDecodeError:
            return {"command": "", "parse_error": "invalid_json"}, False

    parts = text.split()
    token = ""
    command = parts[0]
    args = parts[1:]

    for part in list(parts):
        if part.startswith("token="):
            token = part.split("=", 1)[1]
            parts.remove(part)
            command = parts[0] if parts else ""
            args = parts[1:]
            break

    if auth_token and command == auth_token and args:
        token = command
        command = args[0]
        args = args[1:]

    return {"command": command, "args": args, "token": token}, False


class ThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def make_tcp_handler(supervisor: BodyFusionSupervisor):
    class TCPHandler(socketserver.BaseRequestHandler):
        def handle(self) -> None:
            self.request.settimeout(2)
            chunks: list[bytes] = []
            while True:
                try:
                    chunk = self.request.recv(4096)
                except socket.timeout:
                    break
                if not chunk:
                    break
                chunks.append(chunk)
                if b"\n" in chunk or b"\r\n\r\n" in b"".join(chunks):
                    break
            payload, is_http = parse_request(b"".join(chunks), supervisor.config.auth_token)
            response = supervisor.handle_command(payload)
            body = json.dumps(response, separators=(",", ":")) + "\n"
            if is_http:
                wire = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/json\r\n"
                    f"Content-Length: {len(body.encode('utf-8'))}\r\n"
                    "Connection: close\r\n\r\n"
                    f"{body}"
                )
            else:
                wire = body
            self.request.sendall(wire.encode("utf-8"))

    return TCPHandler


def make_udp_handler(supervisor: BodyFusionSupervisor):
    class UDPHandler(socketserver.BaseRequestHandler):
        def handle(self) -> None:
            data = self.request[0]
            socket_obj = self.request[1]
            payload, _ = parse_request(data, supervisor.config.auth_token)
            response = supervisor.handle_command(payload)
            socket_obj.sendto((json.dumps(response, separators=(",", ":")) + "\n").encode("utf-8"), self.client_address)

    return UDPHandler


def serve(supervisor: BodyFusionSupervisor) -> None:
    tcp_server = ThreadingTCPServer((supervisor.config.bind_host, supervisor.config.tcp_port), make_tcp_handler(supervisor))
    udp_server = socketserver.ThreadingUDPServer((supervisor.config.bind_host, supervisor.config.udp_port), make_udp_handler(supervisor))

    threads = [
        threading.Thread(target=tcp_server.serve_forever, name="tcp-api", daemon=True),
        threading.Thread(target=udp_server.serve_forever, name="udp-api", daemon=True),
    ]
    for thread in threads:
        thread.start()

    print(json.dumps(supervisor.response(True, "watchdog_ready", **supervisor.health_payload()), indent=2), flush=True)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        tcp_server.shutdown()
        udp_server.shutdown()
        supervisor.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="ZED BodyFusion external watchdog")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="Path to watchdog JSON config")
    parser.add_argument("--check-config", action="store_true", help="Load config, print resolved values, and exit")
    args = parser.parse_args()

    config_path = pathlib.Path(args.config).expanduser().resolve()
    raw = load_json_config(config_path)
    config = WatchdogConfig(raw, config_path)
    supervisor = BodyFusionSupervisor(config)

    if args.check_config:
        payload = supervisor.response(True, "config_ok", config=config.to_dict(), health=supervisor.health_payload())
        print(json.dumps(payload, indent=2))
        return 0

    supervisor.start_monitoring()
    serve(supervisor)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
