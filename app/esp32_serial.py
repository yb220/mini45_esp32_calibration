from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover - tests can run without pyserial
    serial = None

from .models import CapSample, utc_timestamp
from .acquisition_profiles import get_acquisition_profile


@dataclass
class Esp32Log:
    level: str
    message: str


@dataclass
class Esp32ProfileStatus:
    name: str
    cnt: int
    cavg: int
    nominal_hz: float


def _parse_profile_status(text: str) -> Esp32ProfileStatus | None:
    if not text.startswith("L:PROFILE,"):
        return None
    parts = text.split(",")
    if len(parts) < 5:
        return None
    values = {}
    for token in parts[2:]:
        if "=" in token:
            key, value = token.split("=", 1)
            values[key.strip().lower()] = value.strip()
    try:
        return Esp32ProfileStatus(
            name=parts[1].strip().upper(),
            cnt=int(values["cnt"]),
            cavg=int(values["cavg"]),
            nominal_hz=float(values["nominal_hz"]),
        )
    except (KeyError, ValueError):
        return None


def parse_cap_line(line: str, monotonic_s: Optional[float] = None) -> CapSample | Esp32Log | Esp32ProfileStatus | None:
    text = line.strip()
    if not text:
        return None

    profile_status = _parse_profile_status(text)
    if profile_status:
        return profile_status
    if text.startswith("L:"):
        return Esp32Log("info", text[2:])
    if text.startswith("E:"):
        return Esp32Log("error", text[2:])

    now = time.monotonic() if monotonic_s is None else monotonic_s

    if text.startswith("DATA0,") or text.startswith("DATA,"):
        parts = text.split(",")
        if len(parts) != 6:
            return Esp32Log("error", f"Invalid DATA field count: {text}")
        try:
            values = [float(value) for value in parts[1:]]
        except ValueError:
            return Esp32Log("error", f"DATA contains non-numeric fields: {text}")
        return CapSample(utc_timestamp(), now, *values)

    if text.startswith("CAP,"):
        parts = text.split(",")
        if len(parts) not in {8, 12}:
            return Esp32Log("error", f"Invalid CAP field count: {text}")
        try:
            esp_ms = int(parts[1])
            seq = int(parts[2])
            values = [float(value) for value in parts[3:8]]
            profile = parts[8].strip().upper() if len(parts) == 12 else ""
            cnt = int(parts[9]) if len(parts) == 12 else None
            cavg = int(parts[10]) if len(parts) == 12 else None
            nominal_hz = float(parts[11]) if len(parts) == 12 else None
        except ValueError:
            return Esp32Log("error", f"CAP contains non-numeric fields: {text}")
        return CapSample(
            utc_timestamp(),
            now,
            *values,
            esp_ms=esp_ms,
            sequence=seq,
            cap_profile=profile,
            mc1081_cnt=cnt,
            mc1081_cavg=cavg,
            cap_nominal_hz=nominal_hz,
        )

    return Esp32Log("debug", f"Unrecognized serial line: {text}")


class Esp32SerialAdapter:
    def __init__(self, port: str, baud: int = 115200, mode: str = "stream", rate_hz: int = 50):
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.port = port
        self.baud = baud
        self.mode = mode
        self.rate_hz = rate_hz
        self.out_queue: queue.Queue[CapSample | Esp32Log | Esp32ProfileStatus] = queue.Queue(maxsize=2000)
        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self.current_profile: Esp32ProfileStatus | None = None
        self._profile_sample_times: deque[float] = deque(maxlen=30)

    def _put_output(self, item: CapSample | Esp32Log | Esp32ProfileStatus) -> None:
        try:
            self.out_queue.put_nowait(item)
        except queue.Full:
            try:
                self.out_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.out_queue.put_nowait(item)
            except queue.Full:
                pass

    def start(self) -> None:
        self._serial = serial.Serial(self.port, self.baud, timeout=0.2)
        self._serial.reset_input_buffer()
        self._stop.clear()
        if self.mode == "stream":
            self._serial.write(f"START,{self.rate_hz}\n".encode("ascii"))
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send(self, command: str) -> None:
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("ESP32 serial port is not open")
        with self._write_lock:
            self._serial.write((command.strip() + "\n").encode("ascii"))
            self._serial.flush()

    def set_profile(self, name: str) -> None:
        profile = get_acquisition_profile(name)
        self._profile_sample_times.clear()
        self.send(f"PROFILE,{profile.name}")

    def get_profile(self) -> None:
        self.send("GET_PROFILE")

    def stop(self) -> None:
        self._stop.set()
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(b"STOP\n")
                self._serial.flush()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def _run(self) -> None:
        next_poll = 0.0
        period = 1.0 / max(float(self.rate_hz), 1.0)
        next_stream_start_retry = time.monotonic() + 1.0
        next_profile_query = time.monotonic() + 1.2
        received_cap = False
        profile_queried = False
        while not self._stop.is_set() and self._serial and self._serial.is_open:
            try:
                now = time.monotonic()
                if self.mode == "poll" and now >= next_poll:
                    self._serial.write(b"CAPTURE\n")
                    self._serial.flush()
                    next_poll = now + period
                elif self.mode == "stream" and not received_cap and now >= next_stream_start_retry:
                    # ESP32 打开串口后可能自动复位，启动命令过早会丢失；收到首帧前定时重发。
                    self.send(f"START,{self.rate_hz}")
                    next_stream_start_retry = now + 1.0
                if not profile_queried and now >= next_profile_query:
                    self.get_profile()
                    profile_queried = True

                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                parsed = parse_cap_line(line)
                if isinstance(parsed, Esp32ProfileStatus):
                    self.current_profile = parsed
                    self._profile_sample_times.clear()
                elif isinstance(parsed, CapSample):
                    received_cap = True
                    if self.current_profile and not parsed.cap_profile:
                        parsed.cap_profile = self.current_profile.name
                        parsed.mc1081_cnt = self.current_profile.cnt
                        parsed.mc1081_cavg = self.current_profile.cavg
                        parsed.cap_nominal_hz = self.current_profile.nominal_hz
                    self._profile_sample_times.append(parsed.monotonic_s)
                    if len(self._profile_sample_times) >= 2:
                        duration = self._profile_sample_times[-1] - self._profile_sample_times[0]
                        if duration > 0:
                            parsed.cap_effective_hz = (len(self._profile_sample_times) - 1) / duration
                if parsed is not None:
                    self._put_output(parsed)
            except Exception as exc:
                self._put_output(Esp32Log("error", f"Serial error: {exc}"))
                time.sleep(0.2)
