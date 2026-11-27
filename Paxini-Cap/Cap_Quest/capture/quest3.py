#!/usr/bin/env python3
"""Launch XRoboToolKit on Quest 3 and print tracking data received on the PC.

The script first prepares the Quest through USB ADB (or pairs/connects a
wireless ADB session via mDNS), then reads head, controller, and hand data
through XRoboToolkit PC Service and its Python SDK. The XRoboToolKit APK must
remain open with tracking and data sending enabled.

Shared with Paxini-Dex (``Dex_Quest``) up to ``collect_sample``; Cap has since
diverged: this copy also returns ``body_timestamp_ns`` /
``motion_timestamp_ns`` / ``body_joints_timestamp_ns`` for the recorder, drops
the Dex ``received_at`` field, and adds the wireless ADB pairing/mDNS flow
(``pair_and_connect``) plus ``pc_service_command()`` that Dex does not have.

``XrtReader.collect_sample`` returns four SDK-provided timestamps and nothing
else that looks like a clock: ``sdk_timestamp_ns`` (``get_time_stamp_ns``),
``body_timestamp_ns``, ``motion_timestamp_ns`` and
``body_joints_timestamp_ns`` (per-joint list). Host-side read times are
deliberately not kept, so a row can only be dated by the SDK itself.
"""

from __future__ import annotations

import argparse
import importlib.util
import ipaddress
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
# capture/ → Cap_Quest → Paxini-Cap，sdk 在 Paxini-Cap 下
PROJECT_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_APK = (
    PROJECT_ROOT
    / "sdk"
    / "xrobotoolkit"
    / "XRoboToolkit-Quest-1.0.1.apk"
)
DEFAULT_IP_FILE = SCRIPT_DIR / ".quest_ip"
DEFAULT_PACKAGE = "com.xrobotoolkit.client.quest"
DEFAULT_PC_SERVICE_SCRIPT = Path("/opt/apps/roboticsservice/runService.sh")
POSE_FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw")
BUTTON_METHODS = {
    "A": "get_A_button",
    "B": "get_B_button",
    "X": "get_X_button",
    "Y": "get_Y_button",
    "left_menu": "get_left_menu_button",
    "right_menu": "get_right_menu_button",
    "left_axis_click": "get_left_axis_click",
    "right_axis_click": "get_right_axis_click",
}
STOP_EVENT = threading.Event()


class QuestSetupError(RuntimeError):
    """Raised when the Quest or PC service cannot be prepared."""


def log(message: str) -> None:
    print(message, flush=True)


def warn(message: str) -> None:
    print(f"[警告] {message}", file=sys.stderr, flush=True)


def error(message: str) -> None:
    print(f"[错误] {message}", file=sys.stderr, flush=True)


def parse_mdns_services(text: str) -> dict[str, list[tuple[str, int]]]:
    """Parse ``adb mdns services`` output into connect/pairing endpoints.

    Each data line looks like ``adb-XXXX\t_adb-tls-connect._tcp\tHOST:PORT``;
    adb versions differ in whitespace, so split on runs of whitespace and keep
    the last token as the endpoint.
    """
    endpoints: dict[str, list[tuple[str, int]]] = {"connect": [], "pairing": []}
    for line in text.splitlines():
        parts = line.split()
        service = next((part for part in parts if part.startswith("_adb-tls-")), None)
        if service is None:
            continue
        host, _, port_text = parts[-1].rpartition(":")
        if not host:
            continue
        try:
            port = int(port_text)
        except ValueError:
            continue
        if service.startswith("_adb-tls-connect"):
            key = "connect"
        elif service.startswith("_adb-tls-pairing"):
            key = "pairing"
        else:
            continue
        if (host, port) not in endpoints[key]:
            endpoints[key].append((host, port))
    return endpoints


def is_wireless_serial(serial: str) -> bool:
    """ADB serials for wireless transports: ``host:port`` or mDNS ``*.tcp`` names."""
    return ":" in serial or serial.endswith("._tcp")


def interactive_ask(prompt: str) -> str:
    """Read one line from the terminal; refuse to guess when stdin is not a TTY."""
    if not sys.stdin.isatty():
        raise QuestSetupError(
            "无线配对需要交互输入，但当前 stdin 不是终端；"
            "请直接在终端运行，或先手动 adb pair"
        )
    return input(prompt).strip()


def ask_to_unplug() -> None:
    log("=" * 68)
    log("无线 ADB 已连接并验证成功，现在可以拔掉 Quest 的 USB 数据线。")
    log("XRoboToolKit 的头显数据通过 Wi-Fi 发送到电脑上的 PC Service。")
    log("请保持 Quest 和电脑在同一局域网，并在 APK 中开启 Tracking 和 Send。")
    log("=" * 68)


@dataclass(frozen=True)
class QuestSetup:
    serial: str
    ip: str | None
    wireless: bool
    package: str


class QuestAdb:
    def __init__(self, adb: str, apk: Path, package: str, adb_port: int) -> None:
        self.adb = adb
        self.apk = apk
        self.package = package
        self.adb_port = adb_port

    def run(
        self,
        args: Sequence[str],
        *,
        check: bool = True,
        timeout: float = 60.0,
        quiet: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        command = [self.adb, *args]
        if not quiet:
            log(f"[ADB] {shlex.join(command)}")
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except FileNotFoundError as exc:
            raise QuestSetupError(f"找不到 adb：{self.adb}") from exc
        except subprocess.TimeoutExpired as exc:
            raise QuestSetupError(
                f"ADB 命令超时（{timeout:g} 秒）：{shlex.join(command)}"
            ) from exc

        output = "\n".join(
            part.strip() for part in (result.stdout, result.stderr) if part.strip()
        )
        if output and not quiet:
            print(output, flush=True)
        if check and result.returncode != 0:
            raise QuestSetupError(
                f"ADB 命令失败（退出码 {result.returncode}）：{shlex.join(command)}"
            )
        return result

    def start_server(self) -> None:
        self.run(["start-server"], timeout=15.0)

    def devices(self) -> dict[str, str]:
        result = self.run(["devices", "-l"], timeout=15.0)
        devices: dict[str, str] = {}
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line or line.startswith("List of devices"):
                continue
            fields = line.split()
            if len(fields) < 2:
                continue
            serial, state = fields[0], fields[1]
            if serial.startswith("*"):
                continue
            devices[serial] = state
        return devices

    def state(self, serial: str) -> str | None:
        result = self.run(
            ["-s", serial, "get-state"],
            check=False,
            timeout=10.0,
            quiet=True,
        )
        if result.returncode != 0:
            return None
        state = result.stdout.strip()
        return state or None

    def connect_wireless(self, ip: str, wait_seconds: float = 8.0) -> str:
        serial = f"{ip}:{self.adb_port}"
        self.run(["connect", serial], check=False, timeout=15.0)
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline and not STOP_EVENT.is_set():
            if self.state(serial) == "device":
                return serial
            time.sleep(0.5)
        raise QuestSetupError(
            f"无法建立无线 ADB 连接 {serial}。请确认 Quest 已连接 Wi-Fi，"
            "与电脑位于同一局域网，并允许 adb 调试。"
        )

    def mdns_services(self) -> dict[str, list[tuple[str, int]]]:
        """Discover paired/connectable and pairable ADB endpoints via mDNS."""
        result = self.run(["mdns", "services"], check=False, timeout=10.0)
        return parse_mdns_services(
            (result.stdout or "") + "\n" + (result.stderr or "")
        )

    @staticmethod
    def local_ipv4_prefixes() -> list[str]:
        """Return ``a.b.c.`` prefixes of this host's IPv4 networks."""
        try:
            result = subprocess.run(
                ["ip", "-4", "-o", "addr", "show"],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            return []
        return [
            f"{match.group(1)}."
            for match in re.finditer(
                r"inet (\d{1,3}\.\d{1,3}\.\d{1,3})\.\d{1,3}/", result.stdout
            )
        ]

    def ip_on_local_network(self, ip: str | None) -> bool:
        """Whether ``ip`` is worth a legacy ``adb connect`` attempt."""
        if not ip:
            return False
        prefixes = self.local_ipv4_prefixes()
        return not prefixes or any(ip.startswith(prefix) for prefix in prefixes)

    def wireless_candidates(self, saved_ip: str | None) -> list[tuple[str, int]]:
        """Connect endpoints worth trying, saved IP first, then the legacy port."""
        services = self.mdns_services()
        candidates: list[tuple[str, int]] = []
        for host, port in services["connect"]:
            if saved_ip and host == saved_ip:
                candidates.append((host, port))
        for host, port in services["connect"]:
            if (host, port) not in candidates:
                candidates.append((host, port))
        # Only try the old fixed port when the saved IP is on a local subnet; a
        # stale IP from another network would otherwise stall for the full timeout.
        if saved_ip and self.ip_on_local_network(saved_ip):
            legacy = (saved_ip, self.adb_port)
            if legacy not in candidates:
                candidates.append(legacy)
        return candidates

    def connect_wireless_candidate(
        self,
        host: str,
        port: int,
        wait_seconds: float = 4.0,
    ) -> str | None:
        """Try one wireless endpoint; return the serial when it reaches ``device``."""
        serial = f"{host}:{port}"
        if self.state(serial) == "device":
            log(f"[INFO] 复用无线 ADB：{serial}")
            self._save_ip(host)
            return serial
        log(f"[INFO] 尝试无线 ADB：{serial}")
        try:
            self.run(["connect", serial], check=False, timeout=5.0)
        except QuestSetupError as exc:
            warn(f"无线 ADB 连接失败：{exc}")
            return None
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline and not STOP_EVENT.is_set():
            if self.state(serial) == "device":
                self._save_ip(host)
                return serial
            time.sleep(0.5)
        return None

    def pair_and_connect(
        self,
        saved_ip: str | None = None,
        ask: Callable[[str], str] = interactive_ask,
    ) -> str:
        """Connect over wireless ADB, pairing only when the headset asks for it.

        Already-authorized connections are tried first. When pairing is needed,
        the code comes from the headset screen; ports are discovered over mDNS
        and only asked for when mDNS exposes nothing.
        """
        for serial, state in self.devices().items():
            if state == "device" and is_wireless_serial(serial):
                ip = serial.rsplit(":", 1)[0] if ":" in serial else self.detect_quest_ip(serial)
                if ip:
                    self._save_ip(ip)
                log(f"[INFO] 复用已连接的无线 ADB：{serial}")
                return serial

        for host, port in self.wireless_candidates(saved_ip):
            serial = self.connect_wireless_candidate(host, port)
            if serial is not None:
                return serial

        pairing = list(self.mdns_services()["pairing"])
        if saved_ip:
            pairing.sort(key=lambda item: item[0] != saved_ip)
        if pairing:
            host, port = pairing[0]
            log(f"[INFO] 发现 Quest 配对服务：{host}:{port}")
        else:
            host, port = self._ask_pairing_endpoint(ask)
            # The user may already be paired and only lacking mDNS; a connect to
            # the endpoint shown in the headset is cheap and needs no code.
            serial = self.connect_wireless_candidate(host, port)
            if serial is not None:
                return serial

        code = ask(f"输入头显上显示的 6 位配对码（{host}:{port}）: ").strip()
        if not code:
            raise QuestSetupError("配对码为空，已中止无线配对")
        log(f"[INFO] 正在与 {host}:{port} 配对 ...")
        # quiet: the pairing code must not end up in the terminal transcript.
        try:
            result = self.run(
                ["pair", f"{host}:{port}", code],
                check=False,
                quiet=True,
                timeout=60.0,
            )
        except QuestSetupError:
            raise QuestSetupError(
                "adb pair 超时或失败；确认配对码和端口来自同一块头显屏幕"
            ) from None
        if "successfully paired" not in (result.stdout + result.stderr).lower():
            raise QuestSetupError(
                "adb pair 未返回 Successfully paired；确认配对码和端口来自同一块"
                "头显屏幕且没有过期"
            )

        log("[INFO] 配对成功，等待无线 ADB 端口就绪 ...")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not STOP_EVENT.is_set():
            for candidate_host, candidate_port in self.wireless_candidates(host):
                serial = self.connect_wireless_candidate(candidate_host, candidate_port)
                if serial is not None:
                    return serial
            time.sleep(1.0)

        # mDNS may be unavailable even though pairing succeeded; ask once for the
        # "connect" endpoint shown next to the pairing dialog.
        endpoint = ask("无线调试页的“连接用”IP:端口（留空放弃）: ").strip()
        if endpoint:
            candidate_host, _, port_text = endpoint.rpartition(":")
            try:
                candidate_port = int(port_text)
            except ValueError as exc:
                raise QuestSetupError(f"连接端口无效：{endpoint}") from exc
            serial = self.connect_wireless_candidate(candidate_host or host, candidate_port)
            if serial is not None:
                return serial
        raise QuestSetupError(
            "配对成功但未能建立无线 ADB；确认头显无线调试仍开启、与电脑在同一个"
            " Wi-Fi，然后重试或插 USB 线"
        )

    @staticmethod
    def _ask_pairing_endpoint(ask: Callable[[str], str]) -> tuple[str, int]:
        endpoint = ask(
            "头显无线调试页的 IP:端口（首次配对填配对端口；已配对填连接端口）: "
        ).strip()
        host, _, port_text = endpoint.rpartition(":")
        if not host:
            host = endpoint
            port_text = ask("端口: ").strip()
        try:
            port = int(port_text)
        except ValueError as exc:
            raise QuestSetupError(f"配对端口无效：{port_text}") from exc
        return host, port

    def _prepare_wireless_device(self, serial: str) -> QuestSetup:
        ip = (
            serial.rsplit(":", 1)[0]
            if ":" in serial
            else self.detect_quest_ip(serial)
        )
        self._ensure_app(serial)
        self._launch_app(serial)
        if ip:
            self._save_ip(ip)
        return QuestSetup(serial=serial, ip=ip, wireless=True, package=self.package)

    def shell(self, serial: str, *command: str, check: bool = True) -> str:
        result = self.run(
            ["-s", serial, "shell", *command],
            check=check,
            timeout=15.0,
        )
        return result.stdout

    def detect_quest_ip(self, serial: str) -> str | None:
        sources = (
            ("ip", "-4", "route"),
            ("ip", "route"),
            ("ip", "-4", "addr", "show", "wlan0"),
            ("ifconfig", "wlan0"),
            ("ip", "-4", "route", "get", "8.8.8.8"),
        )
        ipv4_pattern = re.compile(r"(?<![\d.])(\d{1,3}(?:\.\d{1,3}){3})(?![\d.])")
        source_pattern = re.compile(
            r"\bsrc\s+(\d{1,3}(?:\.\d{1,3}){3})(?![\d.])"
        )
        address_pattern = re.compile(
            r"\binet(?:\s+addr:)?\s+(\d{1,3}(?:\.\d{1,3}){3})(?![\d.])"
        )
        for command in sources:
            output = self.shell(serial, *command, check=False)
            candidates = source_pattern.findall(output)
            candidates.extend(address_pattern.findall(output))
            for candidate in candidates:
                if not ipv4_pattern.fullmatch(candidate):
                    continue
                try:
                    address = ipaddress.ip_address(candidate)
                except ValueError:
                    continue
                if address.is_loopback or address.is_link_local:
                    continue
                return str(address)
        return None

    def choose_device(
        self,
        requested_serial: str | None,
        requested_ip: str | None,
        saved_ip: str | None,
        allow_usb_only: bool,
        keep_usb_adb: bool = False,
        prefer_wireless: bool = False,
    ) -> QuestSetup:
        devices = self.devices()
        if prefer_wireless and requested_serial is None:
            usb_present = any(
                ":" not in serial and state == "device"
                for serial, state in devices.items()
            )
            services = self.mdns_services()
            wireless_visible = bool(
                requested_ip
                or self.ip_on_local_network(saved_ip)
                or services["connect"]
                or services["pairing"]
            )
            # With a USB cable attached and no wireless service advertising, do not
            # block on a pairing prompt: the cable already works.
            if wireless_visible or not usb_present:
                try:
                    serial = self.pair_and_connect(saved_ip or requested_ip)
                except QuestSetupError as exc:
                    warn(f"无线 ADB 未连上：{exc}")
                else:
                    log(f"[INFO] Quest 无线 ADB 已连接：{serial}")
                    return self._prepare_wireless_device(serial)
            else:
                warn("未发现 Wireless debugging 服务，回退 USB ADB")

        if requested_serial:
            state = devices.get(requested_serial) or self.state(requested_serial)
            if state != "device":
                raise QuestSetupError(
                    f"指定的 Quest 序列号 {requested_serial} 状态不是 device："
                    f"{state or '未连接'}"
                )
            if ":" in requested_serial:
                self._ensure_app(requested_serial)
                self._launch_app(requested_serial)
                ip = requested_serial.rsplit(":", 1)[0]
                self._save_ip(ip)
                return QuestSetup(
                    serial=requested_serial,
                    ip=ip,
                    wireless=True,
                    package=self.package,
                )
            return self._prepare_selected_device(
                requested_serial,
                requested_ip=requested_ip,
                allow_usb_only=allow_usb_only,
                keep_usb_adb=keep_usb_adb,
            )

        connected = [serial for serial, state in devices.items() if state == "device"]
        wireless = [serial for serial in connected if is_wireless_serial(serial)]
        if wireless:
            serial = wireless[0]
            log(f"[INFO] 复用已有无线 ADB 连接：{serial}")
            return self._prepare_wireless_device(serial)

        usb = [serial for serial in connected if ":" not in serial]
        if usb:
            serial = self._choose_usb_device(usb)
            log(f"[INFO] 检测到 USB ADB 设备：{serial}")
            return self._prepare_selected_device(
                serial,
                requested_ip=requested_ip,
                allow_usb_only=allow_usb_only,
                keep_usb_adb=keep_usb_adb,
            )

        problem_states = {
            serial: state
            for serial, state in devices.items()
            if state != "device"
        }
        candidate_ip = requested_ip or saved_ip
        if candidate_ip:
            log(f"[INFO] 尝试使用 Quest IP：{candidate_ip}")
            try:
                serial = self.connect_wireless(candidate_ip)
                self._ensure_app(serial)
                self._launch_app(serial)
                self._save_ip(candidate_ip)
                return QuestSetup(
                    serial=serial,
                    ip=candidate_ip,
                    wireless=True,
                    package=self.package,
                )
            except QuestSetupError as exc:
                if not usb:
                    details = f"\n当前设备状态：{problem_states}" if problem_states else ""
                    raise QuestSetupError(f"{exc}{details}") from exc

        if problem_states:
            raise QuestSetupError(
                "检测到 Quest 但 ADB 状态不可用："
                f"{problem_states}。请在头显内允许 USB 调试，"
                "必要时撤销 USB 调试授权后重新插线。"
            )

        raise QuestSetupError(
            "未检测到 Quest。插 USB 线并允许 USB 调试，或加 --quest-wireless 走无线："
            "先在头显里开启 Wireless debugging（会显示配对码），再运行并输入配对码。"
        )

    def _choose_usb_device(self, serials: list[str]) -> str:
        if len(serials) == 1:
            return serials[0]

        quest_devices: list[str] = []
        details: list[str] = []
        for serial in serials:
            model = self.shell(serial, "getprop", "ro.product.model", check=False)
            model = model.strip() or "未知型号"
            details.append(f"{serial} ({model})")
            if "quest" in model.lower() or "oculus" in model.lower():
                quest_devices.append(serial)
        if len(quest_devices) == 1:
            return quest_devices[0]
        devices_text = "、".join(details)
        raise QuestSetupError(
            f"检测到多个 USB ADB 设备：{devices_text}。请使用 --serial 指定 Quest。"
        )

    def _prepare_selected_device(
        self,
        serial: str,
        requested_ip: str | None,
        allow_usb_only: bool,
        keep_usb_adb: bool = False,
    ) -> QuestSetup:
        ip = requested_ip or self.detect_quest_ip(serial)
        self._ensure_app(serial)
        self._launch_app(serial)

        # PXCap/Quest 同步采集只需要 ADB 来安装和启动 APK。USB ADB 可以一直
        # 保留，Quest 跟踪数据仍通过局域网发送到 PC Service。
        if keep_usb_adb and not requested_ip:
            if ip:
                log(
                    f"[INFO] 保留 USB ADB，Quest Wi-Fi IP={ip}；"
                    "跟踪数据仍通过局域网发送"
                )
            else:
                warn(
                    "保留 USB ADB，但未检测到 Quest 的 Wi-Fi IPv4。"
                    "XRoboToolKit 跟踪数据仍需要 Quest 与电脑网络互通；"
                    "后续等待数据阶段会确认实际链路。"
                )
            return QuestSetup(
                serial=serial,
                ip=ip,
                wireless=False,
                package=self.package,
            )

        if not ip:
            if allow_usb_only:
                warn(
                    "未检测到 Quest 的 Wi-Fi IPv4 地址，继续使用 USB ADB。"
                    "此模式无法拔线。"
                )
                return QuestSetup(
                    serial=serial,
                    ip=None,
                    wireless=False,
                    package=self.package,
                )
            raise QuestSetupError(
                "已通过 USB 连接，但无法获取 Quest 的 Wi-Fi IPv4 地址。"
                "请先在头显中连接与电脑相同的 Wi-Fi。若只想通过 USB 运行，"
                "请添加 --allow-usb-only。"
            )

        log(f"[INFO] Quest IP：{ip}，正在切换到无线 ADB {ip}:{self.adb_port} ...")
        self.run(
            ["-s", serial, "tcpip", str(self.adb_port)],
            timeout=15.0,
        )
        time.sleep(1.0)
        wireless_serial = self.connect_wireless(ip)
        self._save_ip(ip)
        ask_to_unplug()
        return QuestSetup(
            serial=wireless_serial,
            ip=ip,
            wireless=True,
            package=self.package,
        )

    def _ensure_app(self, serial: str) -> None:
        package_path = self.shell(
            serial,
            "pm",
            "path",
            self.package,
            check=False,
        )
        if "package:" in package_path:
            log(f"[INFO] Quest 已安装 {self.package}")
            return

        if not self.apk_available:
            raise QuestSetupError(
                f"Quest 未安装 {self.package}，且找不到 APK：{self.apk}"
            )
        log(f"[INFO] 正在安装 {self.apk} ...")
        result = self.run(
            ["-s", serial, "install", "-r", str(self.apk)],
            timeout=180.0,
        )
        if "Success" not in (result.stdout + result.stderr):
            raise QuestSetupError("APK 安装命令结束，但未返回 Success。")

    def _launch_app(self, serial: str) -> None:
        log(f"[INFO] 在 Quest 上启动 {self.package} ...")
        result = self.run(
            [
                "-s",
                serial,
                "shell",
                "monkey",
                "-p",
                self.package,
                "-c",
                "android.intent.category.LAUNCHER",
                "1",
            ],
            check=False,
            timeout=20.0,
        )
        output = f"{result.stdout}\n{result.stderr}"
        failed_markers = (
            "No activities found",
            "monkey aborted",
            "Error:",
        )
        if result.returncode != 0 or any(
            marker.lower() in output.lower() for marker in failed_markers
        ):
            raise QuestSetupError(
                f"ADB 无法启动 Quest APK {self.package}。"
                f"monkey 输出：{output.strip() or '空'}"
            )

        # monkey 返回成功只表示启动请求已注入。进程存在与否作为辅助检查；
        # 最终是否真正启动将以 PC Service 是否收到有效跟踪数据为准。
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not STOP_EVENT.is_set():
            pids = self.shell(serial, "pidof", self.package, check=False).strip()
            if pids:
                log(f"[INFO] Quest APK 进程已启动：pid={pids}")
                return
            time.sleep(0.5)
        warn(
            "ADB monkey 已执行，但暂未查到 APK 进程。"
            "继续等待 PC Service 收到 Quest 数据。"
        )

    @staticmethod
    def _save_ip(ip: str) -> None:
        try:
            DEFAULT_IP_FILE.write_text(f"{ip}\n", encoding="utf-8")
            log(f"[INFO] Quest IP 已保存到 {DEFAULT_IP_FILE}")
        except OSError as exc:
            warn(f"无法保存 Quest IP：{exc}")

    @property
    def apk_available(self) -> bool:
        return self.apk.is_file()


def find_adb(argument: str | None) -> str:
    requested = argument or os.environ.get("ADB") or "adb"
    path = Path(requested).expanduser()
    if path.parent != Path(".") or path.is_absolute():
        if path.is_file() and os.access(path, os.X_OK):
            return str(path.resolve())
        raise QuestSetupError(f"ADB 可执行文件不存在或不可执行：{path}")
    adb = shutil.which(requested)
    if not adb:
        raise QuestSetupError(
            "找不到 adb。请安装 Android platform-tools，"
            "或通过 --adb 指定 adb 路径。"
        )
    return adb


def saved_quest_ip() -> str | None:
    """Return the last successfully used Quest Wi-Fi IP, if it was saved."""
    if not DEFAULT_IP_FILE.is_file():
        return None
    try:
        return DEFAULT_IP_FILE.read_text(encoding="utf-8").strip() or None
    except OSError:
        return None


def make_adapter(args: argparse.Namespace) -> QuestAdb:
    return QuestAdb(
        find_adb(args.adb),
        Path(args.apk).expanduser().resolve(),
        args.package,
        args.adb_port,
    )


def prepare_quest(
    args: argparse.Namespace,
    *,
    keep_usb_adb: bool = False,
    prefer_wireless: bool = False,
) -> QuestSetup:
    adapter = make_adapter(args)
    adapter.start_server()
    return adapter.choose_device(
        requested_serial=args.serial,
        requested_ip=args.quest_ip,
        saved_ip=saved_quest_ip(),
        allow_usb_only=args.allow_usb_only,
        keep_usb_adb=keep_usb_adb,
        prefer_wireless=prefer_wireless,
    )


def pc_service_running() -> bool:
    proc_root = Path("/proc")
    if not proc_root.is_dir():
        return True
    for process_dir in proc_root.iterdir():
        if not process_dir.name.isdigit():
            continue
        try:
            command_line = (process_dir / "cmdline").read_bytes()
        except OSError:
            continue
        if b"RoboticsServiceProcess" in command_line:
            return True
    return False


def pc_service_command(service_script: Path) -> list[str]:
    """Build the argv that launches the vendor PC Service script.

    ``runService.sh`` ships without a shebang and uses ``BASH_SOURCE``, so
    exec'ing it directly fails with ENOEXEC ("Exec format error"). Running it
    through bash is what the vendor's own instructions do; a missing bash is a
    configuration error, not a reason to fall back to the broken direct exec.
    """
    bash = shutil.which("bash")
    if bash is None:
        raise QuestSetupError(
            "找不到 bash，无法启动厂商的 runService.sh（脚本没有 shebang，"
            "不能直接执行）"
        )
    return [bash, str(service_script)]


def maybe_start_pc_service(args: argparse.Namespace) -> None:
    if args.no_pc_service_check:
        return

    if pc_service_running():
        log("[INFO] XRoboToolkit PC Service 正在运行")
        return

    service_script = Path(args.pc_service_script).expanduser()
    if not args.start_pc_service:
        warn(
            "未检测到 RoboticsServiceProcess。请先在另一个终端运行："
            f"{service_script}；或者给本脚本添加 --start-pc-service。"
        )
        return

    if not service_script.is_file():
        raise QuestSetupError(f"找不到 PC Service 启动脚本：{service_script}")
    log(f"[INFO] 启动 XRoboToolkit PC Service：{service_script}")
    try:
        subprocess.Popen(
            pc_service_command(service_script),
            cwd=str(service_script.parent),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except OSError as exc:
        raise QuestSetupError(f"PC Service 启动失败：{exc}") from exc
    time.sleep(2.0)
    if not pc_service_running():
        warn(
            "PC Service 启动命令已执行，但暂未检测到 RoboticsServiceProcess。"
            "请检查程序窗口和系统日志。"
        )


def import_sdk() -> Any:
    if importlib.util.find_spec("xrobotoolkit_sdk") is None:
        raise QuestSetupError(
            "未安装 xrobotoolkit_sdk。请运行："
            "python -m pip install --upgrade xrobotoolkit-sdk"
        )
    try:
        import xrobotoolkit_sdk as xrt
    except Exception as exc:
        raise QuestSetupError(f"导入 xrobotoolkit_sdk 失败：{exc}") from exc
    return xrt


def to_builtin(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    item_method = getattr(value, "item", None)
    if callable(item_method) and not hasattr(value, "tolist"):
        try:
            scalar = item_method()
        except (TypeError, ValueError):
            pass
        else:
            if scalar is not value:
                return to_builtin(scalar)
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if hasattr(value, "tolist"):
        return to_builtin(value.tolist())
    if isinstance(value, dict):
        return {str(key): to_builtin(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_builtin(item) for item in value]
    try:
        return to_builtin(float(value))
    except (TypeError, ValueError):
        return str(value)


def pose_info(raw_pose: Any, available: bool = True) -> dict[str, Any]:
    if not available:
        return {"available": False, "tracked": False, "pose_xyzw": None}
    values = to_builtin(raw_pose)
    if not isinstance(values, list) or len(values) < 7:
        return {
            "available": True,
            "tracked": False,
            "pose_xyzw": values,
        }
    try:
        numeric = [float(value) for value in values[:7]]
    except (TypeError, ValueError):
        numeric = []
    tracked = (
        len(numeric) == 7
        and all(math.isfinite(value) for value in numeric)
        and math.sqrt(sum(value * value for value in numeric[3:7])) > 0.5
    )
    return {
        "available": True,
        "tracked": tracked,
        "pose_xyzw": numeric if len(numeric) == 7 else values,
    }


class XrtReader:
    def __init__(self, xrt: Any) -> None:
        self.xrt = xrt
        self.errors: dict[str, str] = {}
        self._ensure_methods(
            (
                "get_headset_pose",
                "get_left_controller_pose",
                "get_right_controller_pose",
            )
        )
        try:
            init_result = xrt.init()
        except Exception as exc:
            raise QuestSetupError(
                "xrobotoolkit_sdk.init() 失败。请确认 PC Service 已启动，"
                f"并检查其后端库是否可加载：{exc}"
            ) from exc
        if init_result is False:
            raise QuestSetupError("xrobotoolkit_sdk.init() 返回失败。")
        log("[INFO] XRoboToolkit Python SDK 初始化完成")

    def _ensure_methods(self, names: Sequence[str]) -> None:
        missing = [name for name in names if not callable(getattr(self.xrt, name, None))]
        if missing:
            raise QuestSetupError(
                "当前 xrobotoolkit_sdk 版本缺少必要接口："
                + ", ".join(missing)
            )

    def call(self, method_name: str, default: Any = None) -> Any:
        method = getattr(self.xrt, method_name, None)
        if not callable(method):
            return default
        try:
            return method()
        except Exception as exc:
            if method_name not in self.errors:
                self.errors[method_name] = str(exc)
                warn(f"读取 {method_name} 失败：{exc}")
            return default

    def close(self) -> None:
        close_method = getattr(self.xrt, "close", None)
        if callable(close_method):
            try:
                close_method()
            except Exception as exc:
                warn(f"关闭 XRoboToolkit SDK 时出现异常：{exc}")

    def collect_sample(self, sequence: int) -> dict[str, Any]:
        head_available = callable(getattr(self.xrt, "get_headset_pose", None))
        left_available = callable(getattr(self.xrt, "get_left_controller_pose", None))
        right_available = callable(getattr(self.xrt, "get_right_controller_pose", None))

        timestamp_ns = self.call("get_time_stamp_ns")
        body_timestamp_ns = self.call("get_body_timestamp_ns", -1)
        motion_timestamp_ns = self.call("get_motion_timestamp_ns", -1)
        body_joints_timestamp = self.call("get_body_joints_timestamp")
        head = pose_info(self.call("get_headset_pose"), head_available)
        left_controller = self._collect_controller("left", left_available)
        right_controller = self._collect_controller("right", right_available)
        left_hand = self._collect_hand("left")
        right_hand = self._collect_hand("right")
        buttons = {
            name: bool(self.call(method_name, False))
            for name, method_name in BUTTON_METHODS.items()
            if callable(getattr(self.xrt, method_name, None))
        }

        tracked = bool(
            head["tracked"]
            or left_controller["pose"]["tracked"]
            or right_controller["pose"]["tracked"]
            or left_hand["active"]
            or right_hand["active"]
        )
        return {
            "sequence": sequence,
            "sdk_timestamp_ns": to_builtin(timestamp_ns),
            "body_timestamp_ns": to_builtin(body_timestamp_ns),
            "motion_timestamp_ns": to_builtin(motion_timestamp_ns),
            "body_joints_timestamp_ns": to_builtin(body_joints_timestamp),
            "tracked": tracked,
            "head": head,
            "controllers": {
                "left": left_controller,
                "right": right_controller,
            },
            "buttons": buttons,
            "hands": {
                "left": left_hand,
                "right": right_hand,
            },
            "pose_order": list(POSE_FIELDS),
        }

    def _collect_controller(self, side: str, available: bool) -> dict[str, Any]:
        pose = pose_info(
            self.call(f"get_{side}_controller_pose"),
            available,
        )
        return {
            "pose": pose,
            "grip": to_builtin(self.call(f"get_{side}_grip")),
            "trigger": to_builtin(self.call(f"get_{side}_trigger")),
            "axis": to_builtin(self.call(f"get_{side}_axis")),
        }

    def _collect_hand(self, side: str) -> dict[str, Any]:
        active_method = f"get_{side}_hand_is_active"
        state_method = f"get_{side}_hand_tracking_state"
        active = bool(self.call(active_method, False))
        joints = to_builtin(self.call(state_method)) if active else None
        joint_count = len(joints) if isinstance(joints, list) else 0
        return {
            "active": active,
            "joint_count": joint_count,
            "joints_pose_xyzw": joints,
        }


def format_number(value: Any) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "?"
    return f"{number:+.3f}"


def format_pose(pose: dict[str, Any]) -> str:
    if not pose.get("available"):
        return "API不可用"
    if not pose.get("tracked"):
        return "untracked"
    values = pose.get("pose_xyzw")
    if not isinstance(values, list) or len(values) < 7:
        return "格式无效"
    positions = ",".join(format_number(value) for value in values[:3])
    quaternion = ",".join(format_number(value) for value in values[3:7])
    return f"pos=({positions}) quat(xyzw)=({quaternion})"


def format_buttons(buttons: dict[str, bool]) -> str:
    if not buttons:
        return ""
    values = " ".join(f"{name}={int(pressed)}" for name, pressed in buttons.items())
    return f" buttons=[{values}]"


def print_sample(sample: dict[str, Any], hand_joints: str) -> None:
    print(
        f"\n[#{sample['sequence']:06d}] sdk_ns={sample['sdk_timestamp_ns']} "
        f"tracked={sample['tracked']}",
        flush=True,
    )
    print(f"  head: {format_pose(sample['head'])}", flush=True)

    for side in ("left", "right"):
        controller = sample["controllers"][side]
        grip = controller.get("grip")
        trigger = controller.get("trigger")
        axis = controller.get("axis")
        print(
            f"  controller.{side}: {format_pose(controller['pose'])}"
            f" grip={format_number(grip)} trigger={format_number(trigger)}"
            f" axis={axis}",
            flush=True,
        )
    button_text = format_buttons(sample["buttons"]).strip()
    if button_text:
        print(f"  {button_text}", flush=True)

    for side in ("left", "right"):
        hand = sample["hands"][side]
        print(
            f"  hand.{side}: active={hand['active']} "
            f"joints={hand['joint_count']}",
            flush=True,
        )
        joints = hand.get("joints_pose_xyzw")
        if hand_joints == "full" and isinstance(joints, list):
            for index, joint in enumerate(joints):
                print(
                    f"    joint[{index:02d}]: {format_pose(pose_info(joint))}",
                    flush=True,
                )
        elif hand_joints == "summary" and isinstance(joints, list) and joints:
            print(f"    joint[00]: {format_pose(pose_info(joints[0]))}", flush=True)


def read_and_print(args: argparse.Namespace) -> int:
    xrt = import_sdk()
    reader: XrtReader | None = None
    jsonl_file = None
    if args.jsonl:
        jsonl_path = Path(args.jsonl).expanduser().resolve()
        jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        jsonl_file = jsonl_path.open("w", encoding="utf-8")
        log(f"[INFO] JSONL 将写入：{jsonl_path}")

    started = time.monotonic()
    deadline = started + args.duration if args.duration > 0 else None
    sequence = 0
    received_tracking = False
    wait_warned = False
    sample_period = 1.0 / args.rate

    try:
        reader = XrtReader(xrt)
        log(
            "[INFO] 正在等待 Quest 数据。请保持 APK 打开，并确认 Tracking 和 Send 已开启。"
        )
        while not STOP_EVENT.is_set():
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break

            sequence += 1
            sample = reader.collect_sample(sequence)
            print_sample(sample, args.hand_joints)
            if jsonl_file is not None:
                jsonl_file.write(
                    json.dumps(sample, ensure_ascii=False, allow_nan=False) + "\n"
                )
                jsonl_file.flush()

            if sample["tracked"] and not received_tracking:
                received_tracking = True
                log(
                    "[INFO] 已通过 PC Service 收到 Quest 跟踪数据，"
                    "无线数据链路工作正常。"
                )
            if not sample["tracked"] and not wait_warned and now - started > 8.0:
                wait_warned = True
                warn(
                    "SDK 已运行，但尚未收到有效跟踪数据。请检查 PC Service、"
                    "Quest 网络、APK 的 Tracking/Send 开关以及手柄是否已唤醒。"
                )

            next_time = now + sample_period
            while (
                not STOP_EVENT.is_set()
                and time.monotonic() < next_time
            ):
                time.sleep(min(0.05, max(0.0, next_time - time.monotonic())))
    finally:
        if reader is not None:
            reader.close()
        if jsonl_file is not None:
            jsonl_file.close()

    elapsed = time.monotonic() - started
    log(
        f"[INFO] 已停止，共打印 {sequence} 个样本，运行 {elapsed:.1f} 秒；"
        f"是否收到有效跟踪数据：{received_tracking}"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "通过 USB 或无线 ADB 配置 Quest 3，启动 XRoboToolKit APK，"
            "并打印 PC Service 收到的 head/controller/hand 数据。"
            "无线模式用 --wireless（自动发现/配对）或 --pair（只配对）。"
        )
    )
    parser.add_argument("--apk", default=str(DEFAULT_APK), help="Quest APK 路径")
    parser.add_argument("--package", default=DEFAULT_PACKAGE, help="APK 包名")
    parser.add_argument("--adb", help="adb 可执行文件路径，默认从 PATH 查找")
    parser.add_argument("--serial", help="已有 ADB 设备序列号，可避免多设备歧义")
    parser.add_argument("--quest-ip", help="Quest 的局域网 IPv4 地址")
    parser.add_argument(
        "--adb-port",
        type=int,
        default=5555,
        help="无线 ADB 端口，默认 5555（旧式 adb tcpip 用）",
    )
    parser.add_argument(
        "--wireless",
        action="store_true",
        help="优先尝试无线 ADB（mDNS 自动发现/按提示输入配对码），失败回退 USB",
    )
    parser.add_argument(
        "--pair",
        action="store_true",
        help="只做无线 ADB 配对/连接（按提示输入配对码），成功后退出",
    )
    parser.add_argument(
        "--skip-adb",
        action="store_true",
        help="不安装、不启动 APK，也不配置无线 ADB",
    )
    parser.add_argument(
        "--prepare-only",
        action="store_true",
        help="只安装/启动 APK 并准备无线 ADB，不读取 SDK 数据",
    )
    parser.add_argument(
        "--allow-usb-only",
        action="store_true",
        help="无法获取 Wi-Fi IP 时仍继续使用 USB，不允许拔线",
    )
    parser.add_argument(
        "--start-pc-service",
        action="store_true",
        help="未运行时自动启动 PC Service",
    )
    parser.add_argument(
        "--pc-service-script",
        default=str(DEFAULT_PC_SERVICE_SCRIPT),
        help="PC Service 启动脚本路径",
    )
    parser.add_argument(
        "--no-pc-service-check",
        action="store_true",
        help="不检查 PC Service 进程",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="运行秒数；0 表示运行到 Ctrl+C，默认 0",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=2.0,
        help="终端打印频率（Hz），默认 2",
    )
    parser.add_argument(
        "--hand-joints",
        choices=("none", "summary", "full"),
        default="summary",
        help="手部关节打印级别，JSONL 始终包含完整数据",
    )
    parser.add_argument(
        "--jsonl",
        help="将每个样本完整写入指定的 JSONL 文件",
    )
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.duration < 0:
        parser.error("--duration 必须大于或等于 0")
    if args.rate <= 0:
        parser.error("--rate 必须大于 0")
    if not 1 <= args.adb_port <= 65535:
        parser.error("--adb-port 必须在 1..65535 范围内")
    if args.prepare_only and args.skip_adb:
        parser.error("--prepare-only 与 --skip-adb 不能同时使用")
    if args.pair and args.skip_adb:
        parser.error("--pair 与 --skip-adb 不能同时使用")


def handle_stop_signal(signum: int, _frame: Any) -> None:
    if not STOP_EVENT.is_set():
        log(f"\n[INFO] 收到信号 {signum}，正在停止 ...")
        STOP_EVENT.set()


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    signal.signal(signal.SIGINT, handle_stop_signal)
    signal.signal(signal.SIGTERM, handle_stop_signal)

    try:
        if args.pair:
            adapter = make_adapter(args)
            adapter.start_server()
            serial = adapter.pair_and_connect(saved_quest_ip())
            log(f"[INFO] 无线 ADB 已连接：{serial}")
            return 0

        if not args.prepare_only and importlib.util.find_spec("xrobotoolkit_sdk") is None:
            raise QuestSetupError(
                "未安装 xrobotoolkit_sdk。请运行："
                "python -m pip install --upgrade xrobotoolkit-sdk"
            )

        if not args.prepare_only:
            maybe_start_pc_service(args)

        setup = None
        if not args.skip_adb:
            setup = prepare_quest(args, prefer_wireless=args.wireless)
            if setup.wireless:
                log(f"[INFO] Quest 无线 ADB：{setup.serial}")
            else:
                log(f"[INFO] Quest USB ADB：{setup.serial}")
        else:
            log("[INFO] 已跳过 ADB 操作，直接读取 PC Service 数据")

        if args.prepare_only:
            log("[INFO] Quest 准备完成。戴上头显并确保 Tracking 和 Send 已开启。")
            return 0

        return read_and_print(args)
    except QuestSetupError as exc:
        error(str(exc))
        return 2


if __name__ == "__main__":
    sys.exit(main())
