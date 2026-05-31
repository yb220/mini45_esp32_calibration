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
    trajectory_type: str
    phase: str
    axis: str
    direction: str
    branch: str
    target_fx: float
    target_fy: float
    target_fz: float
    target_shear_n: float = 0.0
    target_angle_deg: float | str = ""


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


def _append_training_target(
    targets: list[TrainingTarget],
    trajectory_type: str,
    force: tuple[float, float, float],
    phase: str,
    axis: str,
    direction: str,
    branch: str,
    target_angle_deg: float | str = "",
) -> None:
    shear = math.hypot(force[0], force[1])
    angle: float | str = target_angle_deg
    if shear > 1e-9 and angle == "":
        angle = _round_force((math.degrees(math.atan2(force[1], force[0])) + 360.0) % 360.0)
    targets.append(
        TrainingTarget(
            trajectory_type=trajectory_type,
            phase=phase,
            axis=axis,
            direction=direction,
            branch=branch,
            target_fx=_round_force(force[0]),
            target_fy=_round_force(force[1]),
            target_fz=_round_force(force[2]),
            target_shear_n=_round_force(shear),
            target_angle_deg=angle,
        )
    )


def _append_target_line(
    targets: list[TrainingTarget],
    trajectory_type: str,
    current: tuple[float, float, float],
    end: tuple[float, float, float],
    phase: str,
    axis: str,
    direction: str,
    branch: str,
    target_step_n: float,
    target_angle_deg: float | str = "",
) -> tuple[float, float, float]:
    distance = math.sqrt(sum((end[index] - current[index]) ** 2 for index in range(3)))
    if distance <= 1e-9:
        return end
    step = max(float(target_step_n), 0.02)
    count = max(1, int(math.ceil(distance / step)))
    for point in range(1, count + 1):
        ratio = point / count
        force = (
            current[0] + (end[0] - current[0]) * ratio,
            current[1] + (end[1] - current[1]) * ratio,
            current[2] + (end[2] - current[2]) * ratio,
        )
        _append_training_target(
            targets,
            trajectory_type=trajectory_type,
            phase=phase,
            axis=axis,
            direction=direction,
            branch=branch,
            force=force,
            target_angle_deg=target_angle_deg,
        )
    return end


def _polar_target(shear_n: float, angle_deg: float, fz: float) -> tuple[float, float, float]:
    radians = math.radians(angle_deg)
    return (_round_force(shear_n * math.cos(radians)), _round_force(shear_n * math.sin(radians)), _round_force(fz))


def generate_training_trajectory(
    fz_levels: list[float],
    shear_max: float,
    trajectory_type: str,
    target_step_n: float = 0.2,
    random_points: int = 30,
    rng: random.Random | None = None,
) -> list[TrainingTarget]:
    if shear_max < 0:
        raise ValueError("shear_max must be non-negative")
    if random_points <= 0:
        raise ValueError("random_points must be positive")
    targets: list[TrainingTarget] = []
    current = (0.0, 0.0, 0.0)
    rng = rng or random.Random()

    for fz in fz_levels:
        current = _append_target_line(targets, trajectory_type, current, (0.0, 0.0, fz), "preload", "combined", "none", "loading", target_step_n)

        if trajectory_type == "fx_roundtrip":
            moves = [
                ((shear_max, 0.0, fz), "positive", "loading"),
                ((0.0, 0.0, fz), "positive", "unloading"),
                ((-shear_max, 0.0, fz), "negative", "loading"),
                ((0.0, 0.0, fz), "negative", "unloading"),
            ]
            for end, direction, branch in moves:
                current = _append_target_line(targets, trajectory_type, current, end, "target", "combined", direction, branch, target_step_n)

        elif trajectory_type == "fy_roundtrip":
            moves = [
                ((0.0, shear_max, fz), "positive", "loading"),
                ((0.0, 0.0, fz), "positive", "unloading"),
                ((0.0, -shear_max, fz), "negative", "loading"),
                ((0.0, 0.0, fz), "negative", "unloading"),
            ]
            for end, direction, branch in moves:
                current = _append_target_line(targets, trajectory_type, current, end, "target", "combined", direction, branch, target_step_n)

        elif trajectory_type == "diagonal_roundtrip":
            for angle in (45.0, 135.0, 225.0, 315.0):
                current = _append_target_line(targets, trajectory_type, current, _polar_target(shear_max, angle, fz), "target", "combined", f"angle_{int(angle):03d}", "loading", target_step_n, angle)
                current = _append_target_line(targets, trajectory_type, current, (0.0, 0.0, fz), "target", "combined", f"angle_{int(angle):03d}", "unloading", target_step_n, angle)

        elif trajectory_type == "random_perturb":
            for index in range(int(random_points)):
                # sqrt(random) 使随机点在剪切圆盘面积内近似均匀分布，而不是集中在圆心。
                radius = shear_max * math.sqrt(rng.random())
                angle = rng.uniform(0.0, 360.0)
                target = _polar_target(radius, angle, fz)
                _append_training_target(targets, trajectory_type, target, "target", "combined", f"random_{index + 1:02d}", "loading", _round_force(angle))
                current = target
            current = _append_target_line(targets, trajectory_type, current, (0.0, 0.0, fz), "target", "combined", "random", "unloading", target_step_n)

        else:
            raise ValueError(f"unknown trajectory_type: {trajectory_type}")

        current = _append_target_line(targets, trajectory_type, current, (0.0, 0.0, 0.0), "recovery", "combined", "none", "unloading", target_step_n)

    return targets


def training_target_reached(force: ForceSample, target: TrainingTarget, arrival_window_n: float) -> bool:
    window = max(float(arrival_window_n), 0.0)
    return (
        abs(float(force.fx) - target.target_fx) <= window
        and abs(float(force.fy) - target.target_fy) <= window
        and abs(float(force.fz) - target.target_fz) <= window
    )


def training_target_timed_out(elapsed_s: float, max_wait_s: float) -> bool:
    return float(elapsed_s) >= max(float(max_wait_s), 0.0)


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
