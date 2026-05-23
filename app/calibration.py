from __future__ import annotations

import math
import random
import re
from dataclasses import dataclass

from .models import ExperimentMeta, ForceSample, StabilitySettings


FORCE_AXES = ("Fx", "Fy", "Fz")
FORCE_FIELDS = {"Fx": "fx", "Fy": "fy", "Fz": "fz"}


@dataclass
class CalibrationTarget:
    axis: str
    direction: str
    branch: str
    target_fx: float
    target_fy: float
    target_fz: float
    cycle_index: int = 1
    point_index: int = 1

    def to_meta(self, base: ExperimentMeta) -> ExperimentMeta:
        return ExperimentMeta(
            experiment_id=base.experiment_id,
            cycle_id=f"cycle_{self.cycle_index:03d}",
            branch=self.branch,
            axis=self.axis,
            direction=self.direction,
            preload_n=self.target_fz,
            target_fx=self.target_fx,
            target_fy=self.target_fy,
            target_fz=self.target_fz,
            note=base.note,
        )


@dataclass
class ControlChoice:
    axis: str
    error: float
    measured: float
    target: float
    tolerance: float
    all_in_window: bool


@dataclass
class TrainingTarget:
    target_fx: float
    target_fy: float
    target_fz: float
    target_shear_n: float = 0.0
    target_angle_deg: float | str = ""


@dataclass
class TrainingSegment:
    trajectory_type: str
    phase: str
    axis: str
    direction: str
    branch: str
    start_fx: float
    start_fy: float
    start_fz: float
    end_fx: float
    end_fy: float
    end_fz: float
    duration_s: float
    target_shear_n: float = 0.0
    target_angle_deg: float | str = ""

    def target_at(self, elapsed_s: float) -> TrainingTarget:
        if self.duration_s <= 0:
            ratio = 1.0
        else:
            ratio = min(max(float(elapsed_s) / self.duration_s, 0.0), 1.0)
        fx = self.start_fx + (self.end_fx - self.start_fx) * ratio
        fy = self.start_fy + (self.end_fy - self.start_fy) * ratio
        fz = self.start_fz + (self.end_fz - self.start_fz) * ratio
        shear = math.hypot(fx, fy)
        angle: float | str = self.target_angle_deg
        if shear > 1e-9:
            angle = _round_force((math.degrees(math.atan2(fy, fx)) + 360.0) % 360.0)
        return TrainingTarget(
            target_fx=_round_force(fx),
            target_fy=_round_force(fy),
            target_fz=_round_force(fz),
            target_shear_n=_round_force(shear if shear > 1e-9 else self.target_shear_n),
            target_angle_deg=angle,
        )


def _round_force(value: float) -> float:
    return round(float(value), 6)


def _force_values(max_force: float, step: float) -> list[float]:
    if max_force < 0 or step <= 0:
        raise ValueError("force range must be non-negative and step must be positive")
    count = int(round(max_force / step))
    return [_round_force(i * step) for i in range(count + 1)]


def generate_fz_sequence(max_force: float = 9.0, step: float = 1.0, cycles: int = 3) -> list[CalibrationTarget]:
    values = _force_values(max_force, step)
    down_values = list(reversed(values[:-1]))
    targets: list[CalibrationTarget] = []
    index = 1
    for cycle in range(1, int(cycles) + 1):
        for value in values:
            targets.append(CalibrationTarget("Fz", "positive", "loading", 0.0, 0.0, value, cycle, index))
            index += 1
        for value in down_values:
            targets.append(CalibrationTarget("Fz", "positive", "unloading", 0.0, 0.0, value, cycle, index))
            index += 1
    return targets


def generate_shear_sequence(
    axis: str,
    max_force: float = 3.6,
    step: float = 0.6,
    target_fz: float = 0.0,
    direction_mode: str = "both",
    cycles: int = 3,
) -> list[CalibrationTarget]:
    if axis not in {"Fx", "Fy"}:
        raise ValueError("shear sequence axis must be Fx or Fy")
    modes = {"positive": [("positive", 1.0)], "negative": [("negative", -1.0)], "both": [("positive", 1.0), ("negative", -1.0)]}
    if direction_mode not in modes:
        raise ValueError("direction_mode must be positive, negative, or both")
    values = _force_values(max_force, step)
    down_values = list(reversed(values[:-1]))
    targets: list[CalibrationTarget] = []
    index = 1
    for cycle in range(1, int(cycles) + 1):
        for direction, sign in modes[direction_mode]:
            for value in values:
                fx = sign * value if axis == "Fx" else 0.0
                fy = sign * value if axis == "Fy" else 0.0
                targets.append(CalibrationTarget(axis, direction, "loading", _round_force(fx), _round_force(fy), target_fz, cycle, index))
                index += 1
            for value in down_values:
                fx = sign * value if axis == "Fx" else 0.0
                fy = sign * value if axis == "Fy" else 0.0
                targets.append(CalibrationTarget(axis, direction, "unloading", _round_force(fx), _round_force(fy), target_fz, cycle, index))
                index += 1
    return targets


def parse_force_levels(text: str) -> list[float]:
    tokens = [token for token in re.split(r"[,，;；\s]+", text.strip()) if token]
    values = sorted({_round_force(float(token)) for token in tokens})
    if not values:
        raise ValueError("Fz levels must not be empty")
    if any(value < 0 for value in values):
        raise ValueError("Fz levels must be non-negative")
    return values


def _move_duration(start: tuple[float, float, float], end: tuple[float, float, float], rate_n_s: float) -> float:
    rate = max(float(rate_n_s), 1e-6)
    delta = max(abs(end[0] - start[0]), abs(end[1] - start[1]), abs(end[2] - start[2]))
    return 0.0 if delta <= 1e-9 else max(delta / rate, 0.05)


def _append_move(
    segments: list[TrainingSegment],
    trajectory_type: str,
    current: tuple[float, float, float],
    end: tuple[float, float, float],
    phase: str,
    axis: str,
    direction: str,
    branch: str,
    rate_n_s: float,
    target_angle_deg: float | str = "",
) -> tuple[float, float, float]:
    duration = _move_duration(current, end, rate_n_s)
    if duration <= 0.0:
        return end
    segments.append(
        TrainingSegment(
            trajectory_type=trajectory_type,
            phase=phase,
            axis=axis,
            direction=direction,
            branch=branch,
            start_fx=_round_force(current[0]),
            start_fy=_round_force(current[1]),
            start_fz=_round_force(current[2]),
            end_fx=_round_force(end[0]),
            end_fy=_round_force(end[1]),
            end_fz=_round_force(end[2]),
            duration_s=duration,
            target_shear_n=_round_force(math.hypot(end[0], end[1])),
            target_angle_deg=target_angle_deg,
        )
    )
    return end


def _append_hold(
    segments: list[TrainingSegment],
    trajectory_type: str,
    current: tuple[float, float, float],
    duration_s: float,
    phase: str,
    axis: str,
    direction: str,
    branch: str,
    target_angle_deg: float | str = "",
) -> None:
    if duration_s <= 0:
        return
    segments.append(
        TrainingSegment(
            trajectory_type=trajectory_type,
            phase=phase,
            axis=axis,
            direction=direction,
            branch=branch,
            start_fx=_round_force(current[0]),
            start_fy=_round_force(current[1]),
            start_fz=_round_force(current[2]),
            end_fx=_round_force(current[0]),
            end_fy=_round_force(current[1]),
            end_fz=_round_force(current[2]),
            duration_s=float(duration_s),
            target_shear_n=_round_force(math.hypot(current[0], current[1])),
            target_angle_deg=target_angle_deg,
        )
    )


def _polar_target(shear_n: float, angle_deg: float, fz: float) -> tuple[float, float, float]:
    radians = math.radians(angle_deg)
    return (_round_force(shear_n * math.cos(radians)), _round_force(shear_n * math.sin(radians)), _round_force(fz))


def generate_training_trajectory(
    fz_levels: list[float],
    shear_max: float,
    trajectory_type: str,
    force_rate_n_s: float,
    hold_s: float,
    recovery_s: float,
) -> list[TrainingSegment]:
    if shear_max < 0:
        raise ValueError("shear_max must be non-negative")
    segments: list[TrainingSegment] = []
    current = (0.0, 0.0, 0.0)
    rng = random.Random(20260522)

    for fz in fz_levels:
        current = _append_move(segments, trajectory_type, current, (0.0, 0.0, fz), "preload", "combined", "none", "loading", force_rate_n_s)

        if trajectory_type == "fx_roundtrip":
            moves = [
                ((shear_max, 0.0, fz), "positive", "loading"),
                ((0.0, 0.0, fz), "positive", "unloading"),
                ((-shear_max, 0.0, fz), "negative", "loading"),
                ((0.0, 0.0, fz), "negative", "unloading"),
            ]
            for end, direction, branch in moves:
                current = _append_move(segments, trajectory_type, current, end, "moving", "combined", direction, branch, force_rate_n_s)
                _append_hold(segments, trajectory_type, current, hold_s, "holding", "combined", direction, branch)

        elif trajectory_type == "fy_roundtrip":
            moves = [
                ((0.0, shear_max, fz), "positive", "loading"),
                ((0.0, 0.0, fz), "positive", "unloading"),
                ((0.0, -shear_max, fz), "negative", "loading"),
                ((0.0, 0.0, fz), "negative", "unloading"),
            ]
            for end, direction, branch in moves:
                current = _append_move(segments, trajectory_type, current, end, "moving", "combined", direction, branch, force_rate_n_s)
                _append_hold(segments, trajectory_type, current, hold_s, "holding", "combined", direction, branch)

        elif trajectory_type == "diagonal_roundtrip":
            for angle in (45.0, 135.0, 225.0, 315.0):
                current = _append_move(segments, trajectory_type, current, _polar_target(shear_max, angle, fz), "moving", "combined", f"angle_{int(angle):03d}", "loading", force_rate_n_s, angle)
                _append_hold(segments, trajectory_type, current, hold_s, "holding", "combined", f"angle_{int(angle):03d}", "loading", angle)
                current = _append_move(segments, trajectory_type, current, (0.0, 0.0, fz), "moving", "combined", f"angle_{int(angle):03d}", "unloading", force_rate_n_s, angle)
                _append_hold(segments, trajectory_type, current, hold_s, "holding", "combined", f"angle_{int(angle):03d}", "unloading", angle)

        elif trajectory_type == "circular_shear":
            current = _append_move(segments, trajectory_type, current, _polar_target(shear_max, 0.0, fz), "moving", "combined", "circular", "loading", force_rate_n_s, 0.0)
            for angle in (45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0, 360.0):
                current = _append_move(segments, trajectory_type, current, _polar_target(shear_max, angle, fz), "moving", "combined", "circular", "loading", force_rate_n_s, angle)
            current = _append_move(segments, trajectory_type, current, (0.0, 0.0, fz), "moving", "combined", "circular", "unloading", force_rate_n_s)
            _append_hold(segments, trajectory_type, current, hold_s, "holding", "combined", "circular", "unloading")

        elif trajectory_type == "random_perturb":
            for index in range(8):
                radius = shear_max * rng.uniform(0.2, 1.0)
                angle = rng.uniform(0.0, 360.0)
                current = _append_move(segments, trajectory_type, current, _polar_target(radius, angle, fz), "moving", "combined", f"random_{index + 1:02d}", "loading", force_rate_n_s, _round_force(angle))
            current = _append_move(segments, trajectory_type, current, (0.0, 0.0, fz), "moving", "combined", "random", "unloading", force_rate_n_s)

        else:
            raise ValueError(f"unknown trajectory_type: {trajectory_type}")

        current = _append_move(segments, trajectory_type, current, (0.0, 0.0, 0.0), "recovery", "combined", "none", "unloading", force_rate_n_s)
        _append_hold(segments, trajectory_type, current, recovery_s, "recovery", "combined", "none", "unloading")

    return segments


def target_for_axis(meta: ExperimentMeta, axis: str) -> float:
    return {"Fx": meta.target_fx, "Fy": meta.target_fy, "Fz": meta.target_fz}[axis]


def tolerance_for_axis(settings: StabilitySettings, axis: str) -> float:
    return {"Fx": settings.tolerance_fx, "Fy": settings.tolerance_fy, "Fz": settings.tolerance_fz}[axis]


def choose_control_axis(force: ForceSample, meta: ExperimentMeta, settings: StabilitySettings) -> ControlChoice:
    choices = []
    all_in_window = True
    for axis in FORCE_AXES:
        measured = float(getattr(force, FORCE_FIELDS[axis]))
        target = target_for_axis(meta, axis)
        tolerance = max(tolerance_for_axis(settings, axis), 1e-6)
        error = target - measured
        normalized = abs(error) / tolerance
        if abs(error) > tolerance:
            all_in_window = False
        choices.append((normalized, axis, error, measured, target, tolerance))
    _, axis, error, measured, target, tolerance = max(choices, key=lambda item: item[0])
    return ControlChoice(axis, error, measured, target, tolerance, all_in_window)

