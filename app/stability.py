from __future__ import annotations

import math
from statistics import mean, pstdev
from typing import Iterable, Optional

from .models import (
    CAP_FIELDS,
    FORCE_FIELDS,
    CalibrationPoint,
    CombinedSnapshot,
    ExperimentMeta,
    SafetySettings,
    StabilityResult,
    StabilitySettings,
)


def _values(samples: Iterable[CombinedSnapshot], field: str) -> list[float]:
    out = []
    for sample in samples:
        value = getattr(sample, field)
        if value is not None and math.isfinite(value):
            out.append(float(value))
    return out


def _mean(values: list[float]) -> float:
    return mean(values) if values else float("nan")


def _std(values: list[float]) -> float:
    return pstdev(values) if len(values) > 1 else 0.0 if values else float("nan")


def _percentile(values: list[float], percent: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    k = (len(ordered) - 1) * float(percent) / 100.0
    low = math.floor(k)
    high = math.ceil(k)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - k) + ordered[high] * (k - low)


def _trimmed_mean(values: list[float], low_percent: float = 1.0, high_percent: float = 99.0) -> float:
    if not values:
        return float("nan")
    low = _percentile(values, low_percent)
    high = _percentile(values, high_percent)
    kept = [value for value in values if low <= value <= high]
    return _mean(kept)


def _capacitance_stability_reasons(values: list[float], field: str, settings: StabilitySettings) -> list[str]:
    if len(values) < 2:
        return []
    p95p5 = _percentile(values, 95.0) - _percentile(values, 5.0)
    cap_std = _std(values)
    reasons: list[str] = []
    if p95p5 > settings.capacitance_p95p5_max_pf:
        reasons.append(f"{field} capacitance p95-p5 too high")
    if cap_std > settings.capacitance_std_max_pf:
        reasons.append(f"{field} capacitance std too high")
    return reasons


def target_axis_field(axis: str) -> str:
    return {"Fx": "fx", "Fy": "fy", "Fz": "fz"}.get(axis, "fz")


def target_value(meta: ExperimentMeta) -> float:
    return {"Fx": meta.target_fx, "Fy": meta.target_fy, "Fz": meta.target_fz}.get(meta.axis, 0.0)


def target_tolerance(meta: ExperimentMeta, settings: StabilitySettings) -> float:
    return {
        "Fx": settings.tolerance_fx,
        "Fy": settings.tolerance_fy,
        "Fz": settings.tolerance_fz,
    }.get(meta.axis, settings.tolerance_fz)


def full_scale_for_axis(axis: str, settings: StabilitySettings) -> float:
    return {"Fx": settings.fs_fx, "Fy": settings.fs_fy, "Fz": settings.fs_fz}.get(axis, settings.fs_fz)


def _axis_target(meta: ExperimentMeta, axis: str) -> float:
    return {"Fx": meta.target_fx, "Fy": meta.target_fy, "Fz": meta.target_fz}[axis]


def _axis_tolerance(settings: StabilitySettings, axis: str) -> float:
    return {"Fx": settings.tolerance_fx, "Fy": settings.tolerance_fy, "Fz": settings.tolerance_fz}[axis]


def evaluate_stability(
    window_samples: list[CombinedSnapshot],
    meta: ExperimentMeta,
    settings: StabilitySettings,
    safety: SafetySettings,
) -> StabilityResult:
    axis_field = target_axis_field(meta.axis)
    target = target_value(meta)
    tolerance = target_tolerance(meta, settings)
    target_values = _values(window_samples, axis_field)
    measured = _mean(target_values) if target_values else None
    force_std = _std(target_values) if target_values else None
    reasons: list[str] = []

    if not target_values:
        reasons.append("missing target-axis force samples")
        return StabilityResult(False, False, False, "; ".join(reasons), meta.axis, target, measured, force_std)

    in_window = abs(float(measured) - target) <= tolerance
    if not in_window:
        reasons.append("target-axis force outside tolerance window")

    std_limit = max(settings.target_force_std_max_n, full_scale_for_axis(meta.axis, settings) * settings.target_force_std_percent_fs)
    stable = force_std is not None and force_std <= std_limit
    if not stable:
        reasons.append("target-axis force std too high")

    fx = abs(_mean(_values(window_samples, "fx")))
    fy = abs(_mean(_values(window_samples, "fy")))
    fz = abs(_mean(_values(window_samples, "fz")))
    safe = True
    if fx > safety.fx_abs_max_n or fy > safety.fy_abs_max_n or fz > safety.fz_abs_max_n:
        safe = False
        reasons.append("force safety limit exceeded")

    torques = [_mean(_values(window_samples, field)) for field in ("mx", "my", "mz")]
    if any(math.isfinite(v) and abs(v) > settings.torque_abs_max for v in torques):
        safe = False
        reasons.append("torque safety limit exceeded")

    if abs(target) > 1e-9:
        cross_fields = [field for field in ("fx", "fy", "fz") if field != axis_field]
        for field in cross_fields:
            cross = abs(_mean(_values(window_samples, field)))
            if math.isfinite(cross) and cross > abs(target) * settings.cross_axis_ratio_max:
                reasons.append(f"{field} cross-axis force too high")
                break

    for field in CAP_FIELDS:
        values = _values(window_samples, field)
        cap_reasons = _capacitance_stability_reasons(values, field, settings)
        if cap_reasons:
            reasons.extend(cap_reasons)
            break

    return StabilityResult(
        in_window=in_window,
        stable=stable and in_window and safe and not reasons,
        safe=safe,
        reject_reason="; ".join(reasons),
        target_axis=meta.axis,
        target_value=target,
        measured_value=measured,
        force_std=force_std,
    )


def evaluate_three_axis_stability(
    window_samples: list[CombinedSnapshot],
    meta: ExperimentMeta,
    settings: StabilitySettings,
    safety: SafetySettings,
) -> StabilityResult:
    reasons: list[str] = []
    measured_values: dict[str, float] = {}
    force_stds: dict[str, float] = {}
    in_window = True
    stable = True

    for axis, field in (("Fx", "fx"), ("Fy", "fy"), ("Fz", "fz")):
        values = _values(window_samples, field)
        if not values:
            reasons.append(f"missing {axis} force samples")
            in_window = False
            stable = False
            continue
        measured = _mean(values)
        force_std = _std(values)
        measured_values[axis] = measured
        force_stds[axis] = force_std
        target = _axis_target(meta, axis)
        tolerance = _axis_tolerance(settings, axis)
        if abs(measured - target) > tolerance:
            in_window = False
            reasons.append(f"{field} outside tolerance window")
        std_limit = max(settings.target_force_std_max_n, full_scale_for_axis(axis, settings) * settings.target_force_std_percent_fs)
        if force_std > std_limit:
            stable = False
            reasons.append(f"{field} std too high")

    fx = abs(measured_values.get("Fx", _mean(_values(window_samples, "fx"))))
    fy = abs(measured_values.get("Fy", _mean(_values(window_samples, "fy"))))
    fz = abs(measured_values.get("Fz", _mean(_values(window_samples, "fz"))))
    safe = True
    if fx > safety.fx_abs_max_n or fy > safety.fy_abs_max_n or fz > safety.fz_abs_max_n:
        safe = False
        reasons.append("force safety limit exceeded")

    torques = [_mean(_values(window_samples, field)) for field in ("mx", "my", "mz")]
    if any(math.isfinite(v) and abs(v) > settings.torque_abs_max for v in torques):
        safe = False
        reasons.append("torque safety limit exceeded")

    for field in CAP_FIELDS:
        values = _values(window_samples, field)
        cap_reasons = _capacitance_stability_reasons(values, field, settings)
        if cap_reasons:
            reasons.extend(cap_reasons)
            stable = False
            break

    target_axis = meta.axis if meta.axis in {"Fx", "Fy", "Fz"} else "Fz"
    return StabilityResult(
        in_window=in_window,
        stable=stable and in_window and safe and not reasons,
        safe=safe,
        reject_reason="; ".join(reasons),
        target_axis=target_axis,
        target_value=_axis_target(meta, target_axis),
        measured_value=measured_values.get(target_axis),
        force_std=force_stds.get(target_axis),
    )


def build_calibration_point(
    samples: list[CombinedSnapshot],
    meta: ExperimentMeta,
    marker_id: int,
    valid: bool,
    reject_reason: str,
) -> Optional[CalibrationPoint]:
    if not samples:
        return None

    def stat(field: str) -> tuple[float, float, float]:
        vals = _values(samples, field)
        return _mean(vals), _std(vals), _trimmed_mean(vals)

    fx_m, fx_s, fx_t = stat("fx")
    fy_m, fy_s, fy_t = stat("fy")
    fz_m, fz_s, fz_t = stat("fz")
    mx_m, _mx_s, mx_t = stat("mx")
    my_m, _my_s, my_t = stat("my")
    mz_m, _mz_s, mz_t = stat("mz")
    c0_m, c0_s, c0_t = stat("c0")
    c1_m, c1_s, c1_t = stat("c1")
    c2_m, c2_s, c2_t = stat("c2")
    c3_m, c3_s, c3_t = stat("c3")
    c4_m, c4_s, c4_t = stat("c4")
    force_sample_count = sum(
        1
        for sample in samples
        if all(getattr(sample, field) is not None for field in FORCE_FIELDS)
    )
    cap_sample_count = sum(
        1
        for sample in samples
        if all(getattr(sample, field) is not None for field in CAP_FIELDS)
    )

    return CalibrationPoint(
        timestamp_start=samples[0].timestamp,
        timestamp_end=samples[-1].timestamp,
        experiment_id=meta.experiment_id,
        cycle_id=meta.cycle_id,
        branch=meta.branch,
        axis=meta.axis,
        direction=meta.direction,
        preload_N=fz_m if math.isfinite(fz_m) else meta.preload_n,
        target_Fx=meta.target_fx,
        target_Fy=meta.target_fy,
        target_Fz=meta.target_fz,
        Fx_mean=fx_m,
        Fy_mean=fy_m,
        Fz_mean=fz_m,
        Mx_mean=mx_m,
        My_mean=my_m,
        Mz_mean=mz_m,
        Fx_std=fx_s,
        Fy_std=fy_s,
        Fz_std=fz_s,
        C0_mean=c0_m,
        C1_mean=c1_m,
        C2_mean=c2_m,
        C3_mean=c3_m,
        C4_mean=c4_m,
        C0_std=c0_s,
        C1_std=c1_s,
        C2_std=c2_s,
        C3_std=c3_s,
        C4_std=c4_s,
        marker_id=marker_id,
        valid=valid,
        reject_reason=reject_reason,
        note=meta.note,
        Fx_trimmed_mean=fx_t,
        Fy_trimmed_mean=fy_t,
        Fz_trimmed_mean=fz_t,
        Mx_trimmed_mean=mx_t,
        My_trimmed_mean=my_t,
        Mz_trimmed_mean=mz_t,
        C0_trimmed_mean=c0_t,
        C1_trimmed_mean=c1_t,
        C2_trimmed_mean=c2_t,
        C3_trimmed_mean=c3_t,
        C4_trimmed_mean=c4_t,
        cap_sample_count=cap_sample_count,
        force_sample_count=force_sample_count,
    )
