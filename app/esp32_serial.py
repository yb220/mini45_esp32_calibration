from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover - tests can run without pyserial
    serial = None

from .models import CapSample, utc_timestamp


@dataclass
class Esp32Log:
    level: str
    message: str


def parse_cap_line(line: str, monotonic_s: Optional[float] = None) -> CapSample | Esp32Log | None:
    text = line.strip()
    if not text:
        return None

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
        if len(parts) != 8:
            return Esp32Log("error", f"Invalid CAP field count: {text}")
        try:
            esp_ms = int(parts[1])
            seq = int(parts[2])
            values = [float(value) for value in parts[3:]]
        except ValueError:
            return Esp32Log("error", f"CAP contains non-numeric fields: {text}")
        return CapSample(utc_timestamp(), now, *values, esp_ms=esp_ms, sequence=seq)

    return Esp32Log("debug", f"Unrecognized serial line: {text}")


class Esp32SerialAdapter:
    def __init__(self, port: str, baud: int = 115200, mode: str = "stream", rate_hz: int = 50):
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.port = port
        self.baud = baud
        self.mode = mode
        self.rate_hz = rate_hz
        self.out_queue: queue.Queue[CapSample | Esp32Log] = queue.Queue()
        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._serial = serial.Serial(self.port, self.baud, timeout=0.2)
        self._serial.reset_input_buffer()
        self._stop.clear()
        if self.mode == "stream":
            self._serial.write(f"START,{self.rate_hz}\n".encode("ascii"))
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

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
        while not self._stop.is_set() and self._serial and self._serial.is_open:
            try:
                if self.mode == "poll" and time.monotonic() >= next_poll:
                    self._serial.write(b"CAPTURE\n")
                    self._serial.flush()
                    next_poll = time.monotonic() + period

                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                parsed = parse_cap_line(line)
                if parsed is not None:
                    self.out_queue.put(parsed)
            except Exception as exc:
                self.out_queue.put(Esp32Log("error", f"Serial error: {exc}"))
                time.sleep(0.2)
