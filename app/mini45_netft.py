from __future__ import annotations

import math
import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Iterable
from urllib.request import urlopen
import xml.etree.ElementTree as ET

from .models import ForceSample, utc_timestamp


RDT_MAGIC = 0x1234
CMD_STOP = 0x0000
CMD_START_REALTIME = 0x0002
CMD_START_BUFFERED = 0x0003
CMD_BIAS = 0x0042


@dataclass
class Mini45Log:
    level: str
    message: str


def fetch_netft_config(ip: str, timeout_s: float = 2.0) -> dict[str, str]:
    url = f"http://{ip}/netftapi2.xml"
    with urlopen(url, timeout=timeout_s) as response:
        data = response.read()
    root = ET.fromstring(data)
    wanted = {"cfgcpf", "cfgcpt", "scfgfu", "scfgtu", "comrdte", "comrdtrate", "comrdtbsiz", "runstat"}
    values: dict[str, str] = {}
    for elem in root.iter():
        tag = elem.tag.rsplit("}", 1)[-1].lower()
        if tag in wanted and elem.text is not None:
            values[tag] = elem.text.strip()
    return values


def build_rdt_command(command: int, count: int = 0) -> bytes:
    return struct.pack("!HHI", RDT_MAGIC, command, count)


def parse_rdt_packet(
    data: bytes,
    monotonic_s: float | None = None,
    force_counts_per_unit: float = 1_000_000.0,
    torque_counts_per_unit: float = 1_000_000.0,
    force_signs: Iterable[float] = (1.0, 1.0, 1.0),
    torque_signs: Iterable[float] = (1.0, 1.0, 1.0),
) -> list[ForceSample]:
    now = time.monotonic() if monotonic_s is None else monotonic_s
    fs = list(force_signs)
    ts = list(torque_signs)
    samples: list[ForceSample] = []

    if len(data) >= 36 and len(data) % 36 == 0:
        record_size = 36
        fmt = "!IIIiiiiii"
        has_ft_sequence = True
    elif len(data) >= 32 and len(data) % 32 == 0:
        record_size = 32
        fmt = "!IIiiiiii"
        has_ft_sequence = False
    else:
        return samples

    for offset in range(0, len(data), record_size):
        fields = struct.unpack(fmt, data[offset : offset + record_size])
        if has_ft_sequence:
            rdt_sequence, _ft_sequence, status, fx, fy, fz, mx, my, mz = fields
        else:
            rdt_sequence, status, fx, fy, fz, mx, my, mz = fields
        samples.append(
            ForceSample(
                timestamp=utc_timestamp(),
                monotonic_s=now,
                fx=fs[0] * fx / force_counts_per_unit,
                fy=fs[1] * fy / force_counts_per_unit,
                fz=fs[2] * fz / force_counts_per_unit,
                mx=ts[0] * mx / torque_counts_per_unit,
                my=ts[1] * my / torque_counts_per_unit,
                mz=ts[2] * mz / torque_counts_per_unit,
                sequence=rdt_sequence,
                status=status,
            )
        )
    return samples


class Mini45NetFTAdapter:
    def __init__(
        self,
        ip: str,
        port: int = 49152,
        force_counts_per_unit: float = 1_000_000.0,
        torque_counts_per_unit: float = 1_000_000.0,
        force_signs: Iterable[float] = (1.0, 1.0, 1.0),
        torque_signs: Iterable[float] = (1.0, 1.0, 1.0),
    ):
        self.ip = ip
        self.port = port
        self.force_counts_per_unit = force_counts_per_unit
        self.torque_counts_per_unit = torque_counts_per_unit
        self.force_signs = list(force_signs)
        self.torque_signs = list(torque_signs)
        self.out_queue: queue.Queue[ForceSample | Mini45Log] = queue.Queue()
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._first_packet_seen = False
        self._started_at = 0.0
        self._last_timeout_log = 0.0
        self._last_packet_time = 0.0
        self._last_sequence: int | None = None
        self._last_status: int | None = None

    def start(self) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(0.5)
        self._socket.sendto(build_rdt_command(CMD_START_REALTIME, 0), (self.ip, self.port))
        local_ip, local_port = self._socket.getsockname()
        self._started_at = time.monotonic()
        self._first_packet_seen = False
        self._last_timeout_log = 0.0
        self._last_packet_time = 0.0
        self._last_sequence = None
        self._last_status = None
        self.out_queue.put(
            Mini45Log(
                "info",
                f"已向 {self.ip}:{self.port} 发送 RDT 启动命令，本机接收端口 {local_ip}:{local_port}，等待第一帧 UDP 数据",
            )
        )
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def bias(self) -> None:
        if self._socket:
            self._socket.sendto(build_rdt_command(CMD_BIAS, 0), (self.ip, self.port))
            self.out_queue.put(Mini45Log("info", "已发送 RDT 软件清零/偏置命令"))

    def stop(self) -> None:
        self._stop.set()
        if self._socket:
            try:
                self._socket.sendto(build_rdt_command(CMD_STOP, 0), (self.ip, self.port))
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._socket:
            self._socket.close()
            self._socket = None

    def _run(self) -> None:
        assert self._socket is not None
        while not self._stop.is_set():
            try:
                data, _addr = self._socket.recvfrom(2048)
                now = time.monotonic()
                self._last_packet_time = now
                if not self._first_packet_seen:
                    self._first_packet_seen = True
                    self.out_queue.put(Mini45Log("info", f"已收到第一帧 RDT UDP 数据，包长 {len(data)} 字节，来源 {_addr[0]}:{_addr[1]}"))
                samples = parse_rdt_packet(
                    data,
                    monotonic_s=now,
                    force_counts_per_unit=self.force_counts_per_unit,
                    torque_counts_per_unit=self.torque_counts_per_unit,
                    force_signs=self.force_signs,
                    torque_signs=self.torque_signs,
                )
                if not samples:
                    self.out_queue.put(Mini45Log("error", f"RDT 数据包长度异常：{len(data)} 字节，应为 36 字节的整数倍"))
                for sample in samples:
                    if self._last_sequence is not None and sample.sequence is not None:
                        expected = (self._last_sequence + 1) & 0xFFFFFFFF
                        if sample.sequence != expected:
                            self.out_queue.put(Mini45Log("warning", f"RDT 序号不连续：期望 {expected}，收到 {sample.sequence}"))
                    if sample.sequence is not None:
                        self._last_sequence = sample.sequence
                    if sample.status and sample.status != self._last_status:
                        self.out_queue.put(Mini45Log("warning", f"Mini45 系统状态码非零：0x{sample.status:08X}"))
                    self._last_status = sample.status
                    self.out_queue.put(sample)
            except socket.timeout:
                now = time.monotonic()
                if not self._first_packet_seen and now - self._started_at > 2.0 and now - self._last_timeout_log > 2.0:
                    self._last_timeout_log = now
                    self.out_queue.put(
                        Mini45Log(
                            "warning",
                            "发送 RDT 启动命令后仍未收到 UDP 数据。请检查 NETBA IP、电脑网卡同网段、防火墙、RDT 是否启用，以及是否有其他客户端占用了 RDT",
                        )
                    )
                elif self._first_packet_seen and now - self._last_packet_time > 1.0 and now - self._last_timeout_log > 2.0:
                    self._last_timeout_log = now
                    self.out_queue.put(Mini45Log("warning", "Mini45 UDP 数据超过 1 秒未更新"))
                continue
            except Exception as exc:
                self.out_queue.put(Mini45Log("error", f"Net F/T 通信错误：{exc}"))
                time.sleep(0.2)


class Mini45Simulator:
    def __init__(self, rate_hz: int = 100):
        self.rate_hz = rate_hz
        self.out_queue: queue.Queue[ForceSample | Mini45Log] = queue.Queue()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._start = time.monotonic()
        self._sequence = 0

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def bias(self) -> None:
        self._start = time.monotonic()

    def _run(self) -> None:
        period = 1.0 / max(float(self.rate_hz), 1.0)
        while not self._stop.is_set():
            t = time.monotonic() - self._start
            self._sequence += 1
            fx = 0.2 * math.sin(t * 0.7)
            fy = 0.15 * math.sin(t * 0.5)
            fz = 1.0 + 0.4 * math.sin(t * 0.3)
            self.out_queue.put(
                ForceSample(
                    timestamp=utc_timestamp(),
                    monotonic_s=time.monotonic(),
                    fx=fx,
                    fy=fy,
                    fz=fz,
                    mx=0.01 * fx,
                    my=0.01 * fy,
                    mz=0.0,
                    sequence=self._sequence,
                    status=0,
                    source="mini45_sim",
                )
            )
            time.sleep(period)
