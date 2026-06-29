from __future__ import annotations

import math
import random
import re
from dataclasses import dataclass

from .models import ExperimentMeta, ForceSample, StabilitySettings


FORCE_AXES = ("Fx", "Fy", "Fz")
FORCE_FIELDS = {"Fx": "fx", "Fy": "fy", "Fz": "fz"}
STATIC_COLLECTION_MODES = frozenset({"sequence", "static_full", "static_full_retest"})
STATIC_FULL_FLOWS = ("fz", "fx", "fy", "diagonal")


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


def advance_ramped_force_target(
    current_target: tuple[float, float, float],
    final_target: tuple[float, float, float],
    measured_force: tuple[float, float, float],
    rate_n_s: float,
    elapsed_s: float,
    max_lag_n: float = 0.20,
) -> tuple[float, float, float]:
    current = tuple(float(value) for value in current_target)
    final = tuple(float(value) for value in final_target)
    measured = tuple(float(value) for value in measured_force)
    lag = math.sqrt(sum((current[index] - measured[index]) ** 2 for index in range(3)))
    if lag > max(float(max_lag_n), 0.0):
        return current
    delta = tuple(final[index] - current[index] for index in range(3))
    distance = math.sqrt(sum(value * value for value in delta))
    if distance <= 1e-12:
        return final
    max_step = max(float(rate_n_s), 0.0) * max(float(elapsed_s), 0.0)
    if max_step >= distance:
        return final
    scale = max_step / distance
    return tuple(_round_force(current[index] + delta[index] * scale) for index in range(3))


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


def generate_three_axis_sequence(
    fz_max_force: float = 9.0,
    fz_step: float = 1.0,
    shear_max_force: float = 3.6,
    shear_step: float = 0.6,
    target_fz: float = 0.0,
    shear_direction_mode: str = "both",
    cycles: int = 3,
) -> list[CalibrationTarget]:
    return (
        generate_fz_sequence(fz_max_force, fz_step, cycles)
        + generate_shear_sequence("Fx", shear_max_force, shear_step, target_fz, shear_direction_mode, cycles)
        + generate_shear_sequence("Fy", shear_max_force, shear_step, target_fz, shear_direction_mode, cycles)
    )


def parse_force_levels(text: str) -> list[float]:
    tokens = [token for token in re.split(r"[,，;；\s]+", text.strip()) if token]
    values = sorted({_round_force(float(token)) for token in tokens})
    if not values:
        raise ValueError("Fz levels must not be empty")
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Fz levels must be finite")
    if any(value < 0 for value in values):
        raise ValueError("Fz levels must be non-negative")
    return values


def parse_angles_deg(text: str) -> list[float]:
    """Parse comma/semicolon/space-separated angle values in degrees."""
    tokens = [token for token in re.split(r"[,，;；\s]+", text.strip()) if token]
    if not tokens:
        raise ValueError("angles must not be empty")
    parsed = [_round_force(float(token)) for token in tokens]
    if any(not math.isfinite(value) for value in parsed):
        raise ValueError("angles must be finite")
    values = sorted({value % 360.0 for value in parsed})
    if not values:
        raise ValueError("no valid angles")
    return values


def is_static_collection_mode(mode: str) -> bool:
    """Return whether *mode* advances through StaticPointCollector."""
    return str(mode) in STATIC_COLLECTION_MODES


def validate_force_targets(
    targets: list[CalibrationTarget],
    limits: tuple[float, float, float],
) -> None:
    """Reject non-finite targets and targets outside configured force limits."""
    for target in targets:
        values = (target.target_fx, target.target_fy, target.target_fz)
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError(f"point {target.point_index} contains a non-finite force target")
        for axis, value, limit in zip(FORCE_AXES, values, limits):
            if abs(float(value)) > float(limit):
                raise ValueError(
                    f"point {target.point_index} target {axis}={float(value):.3f} N "
                    f"exceeds safety limit {float(limit):.3f} N"
                )


def static_full_target_shear_n(target: CalibrationTarget) -> float:
    return _round_force(math.hypot(float(target.target_fx), float(target.target_fy)))


def static_full_target_angle_deg(target: CalibrationTarget) -> float | None:
    if target.axis != "combined":
        return None
    if str(target.direction).startswith("diag_"):
        try:
            return _round_force(float(str(target.direction).split("_", 1)[1]) % 360.0)
        except (IndexError, ValueError):
            pass
    shear = static_full_target_shear_n(target)
    if shear <= 1e-12:
        return None
    return _round_force((math.degrees(math.atan2(target.target_fy, target.target_fx)) + 360.0) % 360.0)


def _matches_float_filter(value: float, allowed: set[float], tolerance: float) -> bool:
    return any(abs(float(value) - candidate) <= tolerance for candidate in allowed)


def filter_static_full_targets(
    targets: list[CalibrationTarget],
    *,
    axes: set[str] | None = None,
    branches: set[str] | None = None,
    directions: set[str] | None = None,
    cycles: set[int] | None = None,
    preload_levels: set[float] | None = None,
    shear_levels: set[float] | None = None,
    diagonal_angles_deg: set[float] | None = None,
    tolerance: float = 1e-6,
) -> list[CalibrationTarget]:
    """Return all-static targets matching retest filters, preserving source order."""
    normalized_axes = {str(value) for value in axes} if axes else None
    normalized_branches = {str(value) for value in branches} if branches else None
    normalized_directions = {str(value) for value in directions} if directions else None
    normalized_cycles = {int(value) for value in cycles} if cycles else None
    normalized_preloads = {_round_force(value) for value in preload_levels} if preload_levels else None
    normalized_shear = {_round_force(value) for value in shear_levels} if shear_levels else None
    normalized_angles = {_round_force(value % 360.0) for value in diagonal_angles_deg} if diagonal_angles_deg else None

    selected: list[CalibrationTarget] = []
    for target in targets:
        if normalized_axes is not None and target.axis not in normalized_axes:
            continue
        if normalized_branches is not None and target.branch not in normalized_branches:
            continue
        if normalized_directions is not None and target.direction not in normalized_directions:
            continue
        if normalized_cycles is not None and int(target.cycle_index) not in normalized_cycles:
            continue
        if normalized_preloads is not None and not _matches_float_filter(target.target_fz, normalized_preloads, tolerance):
            continue
        if normalized_shear is not None and not _matches_float_filter(static_full_target_shear_n(target), normalized_shear, tolerance):
            continue
        if normalized_angles is not None:
            angle = static_full_target_angle_deg(target)
            if angle is None or not _matches_float_filter(angle, normalized_angles, tolerance):
                continue
        selected.append(target)
    return selected


def _source_values(
    targets: list[CalibrationTarget],
    *,
    axis: str | None,
    attr: str,
) -> set[float]:
    values: set[float] = set()
    for target in targets:
        if axis is not None and target.axis != axis:
            continue
        values.add(_round_force(float(getattr(target, attr))))
    return values


def _source_shear_values(targets: list[CalibrationTarget]) -> set[float]:
    values = {static_full_target_shear_n(target) for target in targets if target.axis == "combined"}
    if values:
        return values
    return {static_full_target_shear_n(target) for target in targets if target.axis in {"Fx", "Fy"}}


def generate_missing_static_full_diagonal_targets(
    source_targets: list[CalibrationTarget],
    *,
    diagonal_angles_deg: set[float] | None,
    axes: set[str] | None = None,
    branches: set[str] | None = None,
    directions: set[str] | None = None,
    cycles: set[int] | None = None,
    preload_levels: set[float] | None = None,
    shear_levels: set[float] | None = None,
    tolerance: float = 1e-6,
) -> list[CalibrationTarget]:
    """Generate retest-only diagonal targets for requested angles absent from source targets."""
    if not source_targets or not diagonal_angles_deg:
        return []
    if axes is not None and "combined" not in {str(value) for value in axes}:
        return []
    # Direction checkboxes are only meaningful for Fz/Fx/Fy targets. If the user narrows
    # them, do not synthesize diagonal targets with diag_XXX directions.
    if directions is not None:
        return []

    requested_angles = sorted({_round_force(float(value) % 360.0) for value in diagonal_angles_deg})
    existing_angles = {
        angle
        for angle in (static_full_target_angle_deg(target) for target in source_targets)
        if angle is not None
    }
    missing_angles = [
        angle
        for angle in requested_angles
        if not _matches_float_filter(angle, existing_angles, tolerance)
    ]
    if not missing_angles:
        return []

    selected_branches = sorted({str(value) for value in branches}) if branches else ["loading", "unloading"]
    selected_cycles = sorted({int(value) for value in cycles}) if cycles else sorted(
        {int(target.cycle_index) for target in source_targets}
    )
    selected_preloads = sorted({_round_force(value) for value in preload_levels}) if preload_levels else sorted(
        _source_values(source_targets, axis="combined", attr="target_fz")
        or _source_values(source_targets, axis=None, attr="target_fz")
    )
    selected_shear = sorted({_round_force(value) for value in shear_levels}) if shear_levels else sorted(
        _source_shear_values(source_targets)
    )
    if not selected_branches or not selected_cycles or not selected_preloads or not selected_shear:
        return []

    next_index = max((int(target.point_index) for target in source_targets), default=0) + 1
    generated: list[CalibrationTarget] = []
    for fz in selected_preloads:
        for cycle in selected_cycles:
            for angle in missing_angles:
                rad = math.radians(angle)
                cos_a = _round_force(math.cos(rad))
                sin_a = _round_force(math.sin(rad))
                direction = f"diag_{int(round(angle)) % 360:03d}"
                for branch in selected_branches:
                    values = selected_shear if branch == "loading" else list(reversed(selected_shear[:-1]))
                    for shear in values:
                        generated.append(
                            CalibrationTarget(
                                "combined",
                                direction,
                                branch,
                                _round_force(shear * cos_a),
                                _round_force(shear * sin_a),
                                fz,
                                cycle,
                                next_index,
                            )
                        )
                        next_index += 1
    return generated


def generate_static_full_sequence(
    fz_max: float,
    fz_step: float,
    preload_levels: list[float],
    shear_max: float,
    shear_step: float,
    diagonal_angles_deg: list[float],
    cycles: int = 3,
    enabled_flows: set[str] | list[str] | tuple[str, ...] | None = None,
    flow_cycles: dict[str, int] | None = None,
) -> list[CalibrationTarget]:
    """Generate the complete all-static calibration sequence.

    Four phases, each repeated *cycles* times:

    1. **Fz 单轴标定** — pure Fz loading 0→max→0, Fx=Fy=0 (no shear).
       Uses :func:`generate_fz_sequence` directly.
    2. **Fx 单轴加载** — at each preload level, sweep Fx ±shear_max,
       Fy=0, Fz held constant.
    3. **Fy 单轴加载** — same, sweep Fy with Fx=0, Fz held constant.
    4. **斜向加载** — at each preload level, radial sweep along every
       angle in *diagonal_angles_deg*, Fz held constant.

    All targets share :class:`CalibrationTarget` format — plug straight
    into the existing auto‑force / static‑point pipeline.
    """
    selected_flows = set(STATIC_FULL_FLOWS) if enabled_flows is None else {str(flow) for flow in enabled_flows}
    unknown = selected_flows - set(STATIC_FULL_FLOWS)
    if unknown:
        raise ValueError(f"unknown static full flow: {', '.join(sorted(unknown))}")
    if not preload_levels and selected_flows & {"fx", "fy", "diagonal"}:
        raise ValueError("preload_levels must not be empty")
    if shear_max < 0:
        raise ValueError("shear_max must be non-negative")
    if shear_step <= 0:
        raise ValueError("shear_step must be positive")
    if cycles < 1:
        raise ValueError("cycles must be >= 1")
    flow_cycle_counts = {flow: int(cycles) for flow in STATIC_FULL_FLOWS}
    if flow_cycles:
        for flow, count in flow_cycles.items():
            key = str(flow)
            if key not in STATIC_FULL_FLOWS:
                raise ValueError(f"unknown static full flow: {key}")
            if int(count) < 1:
                raise ValueError("flow cycles must be >= 1")
            flow_cycle_counts[key] = int(count)

    shear_values = _force_values(shear_max, shear_step)
    shear_down = list(reversed(shear_values[:-1]))
    targets: list[CalibrationTarget] = []
    index = 1

    # ---- Phase 1: Fz 单轴标定（纯法向，Fx=Fy=0）----
    if "fz" in selected_flows:
        targets.extend(generate_fz_sequence(fz_max, fz_step, flow_cycle_counts["fz"]))
        index = len(targets) + 1

    # ---- Phases 2–4: 预载下剪切（Fz 保持恒定，扫 Fx / Fy / 斜向）----
    for fz in sorted(preload_levels):

        # Phase 2: Fx 单轴加载（Fz 预载不变，Fy=0）
        if "fx" in selected_flows:
            for cycle in range(1, flow_cycle_counts["fx"] + 1):
                for sign, direction in ((1.0, "positive"), (-1.0, "negative")):
                    for value in shear_values:
                        targets.append(CalibrationTarget("Fx", direction, "loading", _round_force(sign * value), 0.0, fz, cycle, index))
                        index += 1
                    for value in shear_down:
                        targets.append(CalibrationTarget("Fx", direction, "unloading", _round_force(sign * value), 0.0, fz, cycle, index))
                        index += 1

        # Phase 3: Fy 单轴加载（Fz 预载不变，Fx=0）
        if "fy" in selected_flows:
            for cycle in range(1, flow_cycle_counts["fy"] + 1):
                for sign, direction in ((1.0, "positive"), (-1.0, "negative")):
                    for value in shear_values:
                        targets.append(CalibrationTarget("Fy", direction, "loading", 0.0, _round_force(sign * value), fz, cycle, index))
                        index += 1
                    for value in shear_down:
                        targets.append(CalibrationTarget("Fy", direction, "unloading", 0.0, _round_force(sign * value), fz, cycle, index))
                        index += 1

        # Phase 4: 斜向加载（Fz 预载不变，径向扫）
        if "diagonal" in selected_flows:
            for cycle in range(1, flow_cycle_counts["diagonal"] + 1):
                for angle in diagonal_angles_deg:
                    rad = math.radians(angle)
                    cos_a = _round_force(math.cos(rad))
                    sin_a = _round_force(math.sin(rad))
                    for value in shear_values:
                        targets.append(CalibrationTarget(
                            "combined", f"diag_{int(angle):03d}", "loading",
                            _round_force(value * cos_a), _round_force(value * sin_a), fz,
                            cycle, index,
                        ))
                        index += 1
                    for value in shear_down:
                        targets.append(CalibrationTarget(
                            "combined", f"diag_{int(angle):03d}", "unloading",
                            _round_force(value * cos_a), _round_force(value * sin_a), fz,
                            cycle, index,
                        ))
                        index += 1

    return targets


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
