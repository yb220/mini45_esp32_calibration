from __future__ import annotations

import queue
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover - tests can run without pyserial
    serial = None


MOTOR_AXES = ("X", "Y", "Z")
FORCE_AXES = ("Fx", "Fy", "Fz")
DEFAULT_FORCE_TO_MOTOR = {"Fx": "Z", "Fy": "Y", "Fz": "X"}
DEFAULT_FORCE_TO_MOTOR_SIGN = {"Fx": 1, "Fy": 1, "Fz": 1}
SCREW_LEAD_MM = 2.0
PULSES_PER_REV = 400
PULSES_PER_MM = PULSES_PER_REV / SCREW_LEAD_MM
MM_PER_PULSE = 1.0 / PULSES_PER_MM
AUTO_MIN_PULSES = 4
MANUAL_DEFAULT_STEP_MM = 0.10
MANUAL_DEFAULT_SPEED_MM_S = 1.0
AUTO_DEFAULT_MIN_STEP_MM = AUTO_MIN_PULSES * MM_PER_PULSE
AUTO_DEFAULT_MAX_STEP_MM = 0.30
AUTO_DEFAULT_GAIN_MM_PER_N = 0.05
AUTO_DEFAULT_SPEED_MM_S = 3.0
AUTO_DEFAULT_INTERVAL_S = 0.15


@dataclass
class MotionMessage:
    level: str
    kind: str
    message: str = ""
    values: dict[str, str] = field(default_factory=dict)


def parse_key_values(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for token in text.split():
        if "=" in token:
            key, value = token.split("=", 1)
            values[key.strip().upper()] = value.strip()
    return values


def parse_motion_line(line: str) -> MotionMessage | None:
    text = line.strip()
    if not text:
        return None

    if text.startswith("OK"):
        return MotionMessage("info", "OK", text[2:].strip(), parse_key_values(text))
    if text.startswith("ERR"):
        return MotionMessage("error", "ERR", text[3:].strip(), parse_key_values(text))
    if text.startswith("L:"):
        return MotionMessage("info", "LOG", text[2:].strip())

    parts = text.split(maxsplit=1)
    kind = parts[0].upper()
    payload = parts[1] if len(parts) > 1 else ""
    if kind in {"POS", "STATE", "LIMIT"}:
        return MotionMessage("info", kind, payload, parse_key_values(payload))
    return MotionMessage("debug", "UNKNOWN", text)


def normalize_axis(axis: str) -> str:
    axis = axis.strip().upper()
    if axis not in MOTOR_AXES:
        raise ValueError(f"Invalid motor axis: {axis}")
    return axis


def normalize_force_axis(axis: str) -> str:
    axis = axis.strip()
    if axis not in FORCE_AXES:
        raise ValueError(f"Invalid force axis: {axis}")
    return axis


def mm_to_pulses(distance_mm: float) -> int:
    return int(round(float(distance_mm) * PULSES_PER_MM))


def pulses_to_mm(pulses: int) -> float:
    return int(pulses) / PULSES_PER_MM


def quantize_mm_to_pulses(distance_mm: float, min_pulses: int = 1) -> float:
    distance = float(distance_mm)
    if distance == 0.0:
        return 0.0
    sign = 1 if distance > 0 else -1
    pulses = abs(mm_to_pulses(distance))
    pulses = max(int(min_pulses), pulses)
    return sign * pulses_to_mm(pulses)


def adaptive_force_step_mm(
    force_error: float,
    tolerance: float,
    min_step_mm: float = AUTO_DEFAULT_MIN_STEP_MM,
    max_step_mm: float = AUTO_DEFAULT_MAX_STEP_MM,
    gain_mm_per_n: float = AUTO_DEFAULT_GAIN_MM_PER_N,
    min_pulses: int = AUTO_MIN_PULSES,
) -> float:
    error = abs(float(force_error))
    if error <= float(tolerance):
        return 0.0
    min_step = max(float(min_step_mm), pulses_to_mm(min_pulses))
    max_step = max(min_step, float(max_step_mm))
    requested = error * max(0.0, float(gain_mm_per_n))
    requested = min(max(requested, min_step), max_step)
    return abs(quantize_mm_to_pulses(requested, min_pulses=min_pulses))


class ArduinoMotionAdapter:
    def __init__(self, port: str, baud: int = 115200):
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.port = port
        self.baud = baud
        self.out_queue: queue.Queue[MotionMessage] = queue.Queue(maxsize=500)
        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()

    def _put_output(self, item: MotionMessage) -> None:
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
        self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
        self._serial.reset_input_buffer()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.send("HELLO")

    def stop(self) -> None:
        try:
            self.stop_all()
        except Exception:
            pass
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def send(self, command: str) -> None:
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("Arduino motion serial port is not open")
        line = command.strip()
        if not line:
            return
        with self._write_lock:
            self._serial.write((line + "\n").encode("ascii"))
            self._serial.flush()

    def set_mode(self, mode: str) -> None:
        mode = mode.strip().upper()
        if mode not in {"MANUAL", "PC"}:
            raise ValueError(f"Invalid motion mode: {mode}")
        self.send(f"MODE {mode}")

    def enable(self, enabled: bool) -> None:
        self.send(f"ENABLE {1 if enabled else 0}")

    def stop_all(self) -> None:
        self.send("STOP")

    def stop_axis(self, axis: str) -> None:
        self.send(f"STOP {normalize_axis(axis)}")

    def home(self, axis: str) -> None:
        axis = axis.strip().upper()
        if axis == "ALL":
            self.send("HOME ALL")
        else:
            self.send(f"HOME {normalize_axis(axis)}")

    def jog(self, axis: str, speed_steps_s: float) -> None:
        self.send(f"JOG {normalize_axis(axis)} {float(speed_steps_s):.3f}")

    def move_steps(self, axis: str, steps: int, speed_steps_s: float) -> None:
        self.send(f"MOVE_STEPS {normalize_axis(axis)} {int(steps)} {float(speed_steps_s):.3f}")

    def move_mm(self, axis: str, distance_mm: float, speed_mm_s: float) -> None:
        self.send(f"MOVE_MM {normalize_axis(axis)} {float(distance_mm):.6f} {float(speed_mm_s):.6f}")

    def query_pos(self) -> None:
        self.send("POS?")

    def query_state(self) -> None:
        self.send("STATE?")

    def query_limits(self) -> None:
        self.send("LIMIT?")

    def _run(self) -> None:
        while not self._stop.is_set() and self._serial and self._serial.is_open:
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                parsed = parse_motion_line(line)
                if parsed is not None:
                    self._put_output(parsed)
            except Exception as exc:
                self._put_output(MotionMessage("error", "SERIAL", f"Arduino motion serial error: {exc}"))
                time.sleep(0.2)


def force_axis_value(force_sample, force_axis: str) -> float:
    axis = normalize_force_axis(force_axis)
    return float(getattr(force_sample, axis.lower()))


def force_axis_target(meta, force_axis: str) -> float:
    axis = normalize_force_axis(force_axis)
    return float(getattr(meta, f"target_{axis.lower()}"))


def force_axis_tolerance(settings, force_axis: str) -> float:
    axis = normalize_force_axis(force_axis)
    return float(getattr(settings, f"tolerance_{axis.lower()}"))


def mapped_motor_delta(
    force_axis: str,
    force_error: float,
    step_mm: float,
    mapping: dict[str, str],
    signs: dict[str, int],
    min_pulses: int = 1,
) -> tuple[str, float]:
    axis = normalize_force_axis(force_axis)
    motor_axis = normalize_axis(mapping.get(axis, DEFAULT_FORCE_TO_MOTOR[axis]))
    sign = 1 if int(signs.get(axis, DEFAULT_FORCE_TO_MOTOR_SIGN[axis])) >= 0 else -1
    direction = 1.0 if force_error > 0 else -1.0
    delta = sign * direction * abs(float(step_mm))
    return motor_axis, quantize_mm_to_pulses(delta, min_pulses=min_pulses)


def parse_axis_position(values: dict[str, str], axis: str) -> Optional[float]:
    axis = normalize_axis(axis)
    key = f"{axis}MM"
    raw = values.get(key)
    if raw is None:
        return None
    if not re.fullmatch(r"[-+]?\d+(\.\d+)?", raw):
        return None
    return float(raw)
