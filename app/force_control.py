from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .arduino_motion import MM_PER_PULSE, mm_to_pulses, quantize_mm_to_pulses
from .models import CombinedSnapshot, SafetySettings


FORCE_AXES = ("Fx", "Fy", "Fz")
FORCE_FIELDS = ("fx", "fy", "fz")
MOTOR_AXES = ("X", "Y", "Z")
K_ROW_LABELS = FORCE_AXES
K_COL_LABELS = MOTOR_AXES


@dataclass
class ForceStats:
    mean: list[float]
    std: list[float]
    count: int


@dataclass
class KIdentificationResult:
    k: list[list[float]]
    before_means: dict[str, list[float]]
    after_means: dict[str, list[float]]
    before_stds: dict[str, list[float]]
    after_stds: dict[str, list[float]]
    deltas_mm: dict[str, float]
    singular_values: list[float]
    condition: float
    noise_norm: float
    valid: bool
    reject_reason: str = ""
    debug: bool = False


@dataclass
class DecoupledControlState:
    previous_error: list[float] | None = None
    trust_scale: float = 1.0


@dataclass
class DecoupledControlSettings:
    max_step_mm: float
    min_pulse: int = 1
    style: str = "standard"
    target_condition: float = 50.0
    safety_fraction: float = 0.50


@dataclass
class DecoupledControlCommand:
    delta_mm: dict[str, float]
    pulses: dict[str, int]
    raw_delta: list[float]
    limited_delta: list[float]
    quantized_delta: list[float]
    damping_eta: float
    trust_scale: float
    error_norm: float
    predicted_delta_force: list[float]
    note: str = ""


def _vec(values) -> list[float]:
    return [float(value) for value in values]


def _zeros() -> list[float]:
    return [0.0, 0.0, 0.0]


def _sub(a, b) -> list[float]:
    return [float(a[i]) - float(b[i]) for i in range(3)]


def _add(a, b) -> list[float]:
    return [float(a[i]) + float(b[i]) for i in range(3)]


def _mul_scalar(a, scale: float) -> list[float]:
    return [float(value) * float(scale) for value in a]


def _dot(a, b) -> float:
    return sum(float(a[i]) * float(b[i]) for i in range(3))


def _norm(a) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in a))


def _transpose(a: list[list[float]]) -> list[list[float]]:
    return [[a[row][col] for row in range(3)] for col in range(3)]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)] for row in range(3)]


def _matvec(a: list[list[float]], x) -> list[float]:
    return [sum(a[row][col] * float(x[col]) for col in range(3)) for row in range(3)]


def _identity() -> list[list[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _solve_3x3(a: list[list[float]], b) -> list[float]:
    # 3×3 高斯消元，矩阵奇异时抛出异常，由 GUI 停止自动力控。
    m = [list(map(float, row)) + [float(b[index])] for index, row in enumerate(a)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(m[row][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise ValueError("解耦矩阵奇异，无法计算位移增量")
        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]
        factor = m[col][col]
        for j in range(col, 4):
            m[col][j] /= factor
        for row in range(3):
            if row == col:
                continue
            factor = m[row][col]
            for j in range(col, 4):
                m[row][j] -= factor * m[col][j]
    return [m[row][3] for row in range(3)]


def _symmetric_eigenvalues_3x3(a: list[list[float]]) -> list[float]:
    # Jacobi 旋转足够稳定，避免为 3×3 小矩阵引入额外数值库依赖。
    m = [list(map(float, row)) for row in a]
    for _ in range(32):
        p, q = 0, 1
        max_off = abs(m[p][q])
        for i, j in ((0, 2), (1, 2)):
            if abs(m[i][j]) > max_off:
                p, q = i, j
                max_off = abs(m[i][j])
        if max_off < 1e-12:
            break
        if abs(m[p][p] - m[q][q]) < 1e-12:
            angle = math.pi / 4.0
        else:
            angle = 0.5 * math.atan2(2.0 * m[p][q], m[q][q] - m[p][p])
        c = math.cos(angle)
        s = math.sin(angle)
        for k in range(3):
            mkp = m[k][p]
            mkq = m[k][q]
            m[k][p] = c * mkp - s * mkq
            m[k][q] = s * mkp + c * mkq
        for k in range(3):
            mpk = m[p][k]
            mqk = m[q][k]
            m[p][k] = c * mpk - s * mqk
            m[q][k] = s * mpk + c * mqk
    return sorted([max(0.0, m[i][i]) for i in range(3)], reverse=True)


def singular_values_3x3(k: list[list[float]]) -> list[float]:
    kt = _transpose(k)
    kt_k = _matmul(kt, k)
    eigenvalues = _symmetric_eigenvalues_3x3(kt_k)
    return [math.sqrt(max(0.0, value)) for value in eigenvalues]


def force_vector_from_sample(sample) -> list[float]:
    return [float(sample.fx), float(sample.fy), float(sample.fz)]


def force_stats(samples: Iterable[CombinedSnapshot]) -> ForceStats:
    values: list[list[float]] = []
    for sample in samples:
        row = [getattr(sample, field) for field in FORCE_FIELDS]
        if all(value is not None and math.isfinite(float(value)) for value in row):
            values.append([float(value) for value in row])
    if not values:
        return ForceStats([float("nan")] * 3, [float("nan")] * 3, 0)
    count = len(values)
    mean = [sum(row[index] for row in values) / count for index in range(3)]
    if count > 1:
        std = [math.sqrt(sum((row[index] - mean[index]) ** 2 for row in values) / count) for index in range(3)]
    else:
        std = _zeros()
    return ForceStats(mean, std, count)


def identify_k_matrix(
    before_means: dict[str, list[float]],
    after_means: dict[str, list[float]],
    before_stds: dict[str, list[float]],
    after_stds: dict[str, list[float]],
    deltas_mm: dict[str, float],
    min_delta_force_n: float = 0.05,
    condition_limit: float = 300.0,
) -> KIdentificationResult:
    columns = []
    noise_terms = []
    reasons = []

    for axis in MOTOR_AXES:
        delta = float(deltas_mm[axis])
        if abs(delta) < MM_PER_PULSE * 0.5:
            reasons.append(f"{axis} 扰动位移过小")
            delta = math.copysign(MM_PER_PULSE, delta if delta != 0 else 1.0)
        delta_force = _sub(after_means[axis], before_means[axis])
        columns.append(_mul_scalar(delta_force, 1.0 / delta))
        delta_force_norm = _norm(delta_force)
        noise = _add(
            [0.0 if math.isnan(float(value)) else float(value) for value in before_stds[axis]],
            [0.0 if math.isnan(float(value)) else float(value) for value in after_stds[axis]],
        )
        noise_terms.append(_norm(noise))
        required = max(float(min_delta_force_n), 3.0 * _norm(noise))
        if delta_force_norm < required:
            reasons.append(f"{axis} 轴力变化过小：{delta_force_norm:.4f} N < {required:.4f} N")

    k = [[columns[col][row] for col in range(3)] for row in range(3)]
    singular_values = singular_values_3x3(k)
    max_sv = singular_values[0] if singular_values else 0.0
    rank = sum(1 for value in singular_values if value > max(max_sv * 1e-9, 1e-12))
    if rank < 3:
        reasons.append("K 矩阵秩不足")

    smallest = singular_values[-1] if singular_values else 0.0
    condition = float("inf") if smallest <= 1e-12 else singular_values[0] / smallest
    if not math.isfinite(condition) or condition > float(condition_limit):
        reasons.append(f"K 条件数过大：{condition:.3g}")

    noise_norm = max(noise_terms) if noise_terms else 0.0
    return KIdentificationResult(
        k=k,
        before_means={axis: _vec(values) for axis, values in before_means.items()},
        after_means={axis: _vec(values) for axis, values in after_means.items()},
        before_stds={axis: _vec(values) for axis, values in before_stds.items()},
        after_stds={axis: _vec(values) for axis, values in after_stds.items()},
        deltas_mm=deltas_mm,
        singular_values=singular_values,
        condition=condition,
        noise_norm=noise_norm,
        valid=not reasons,
        reject_reason="；".join(reasons),
    )


def debug_identity_k() -> KIdentificationResult:
    k = _identity()
    zeros = {axis: _zeros() for axis in MOTOR_AXES}
    deltas = {axis: 1.0 for axis in MOTOR_AXES}
    return KIdentificationResult(
        k=k,
        before_means=zeros,
        after_means=zeros,
        before_stds=zeros,
        after_stds=zeros,
        deltas_mm=deltas,
        singular_values=[1.0, 1.0, 1.0],
        condition=1.0,
        noise_norm=0.0,
        valid=True,
        reject_reason="调试默认矩阵，仅用于无硬件检查",
        debug=True,
    )


def auto_damping_eta(k: list[list[float]], noise_norm: float = 0.0, target_condition: float = 50.0) -> float:
    singular_values = singular_values_3x3(k)
    sigma_max = singular_values[0] if singular_values else 0.0
    sigma_min = singular_values[-1] if singular_values else 0.0
    if sigma_max <= 1e-12:
        return 1.0
    target = max(float(target_condition), 2.0)
    numerator = max(0.0, sigma_max * sigma_max - target * sigma_min * sigma_min)
    eta_condition = math.sqrt(numerator / (target - 1.0)) if numerator > 0.0 else 0.0
    eta_noise = min(0.25 * sigma_max, max(0.0, float(noise_norm)) * 0.5)
    return max(eta_condition, eta_noise, sigma_max * 1e-6)


def damped_pseudoinverse(k: list[list[float]], damping_eta: float) -> list[list[float]]:
    kt = _transpose(k)
    kt_k = _matmul(kt, k)
    a = [[kt_k[row][col] + (float(damping_eta) ** 2 if row == col else 0.0) for col in range(3)] for row in range(3)]
    columns = []
    for col in range(3):
        columns.append(_solve_3x3(a, [kt[row][col] for row in range(3)]))
    return [[columns[col][row] for col in range(3)] for row in range(3)]


def update_trust_scale(
    state: DecoupledControlState,
    error: list[float],
    style: str,
) -> tuple[float, str]:
    limits = {
        "conservative": (0.15, 0.70, 0.25),
        "standard": (0.20, 1.00, 0.40),
        "fast": (0.30, 1.40, 0.60),
    }
    min_scale, max_scale, initial = limits.get(style, limits["standard"])
    note = ""
    error_norm = _norm(error)
    if state.previous_error is None:
        state.trust_scale = initial
        return state.trust_scale, "初始化信任域"

    previous_norm = _norm(state.previous_error)
    if previous_norm <= 1e-9:
        return state.trust_scale, note

    sign_changed = any((state.previous_error[index] * error[index]) < 0.0 for index in range(3))
    if sign_changed or error_norm > previous_norm * 1.05:
        state.trust_scale = max(min_scale, state.trust_scale * 0.50)
        note = "误差增大或换向，减小信任域"
    elif error_norm < previous_norm * 0.80:
        state.trust_scale = min(max_scale, state.trust_scale * 1.20)
        note = "误差下降，放大信任域"
    else:
        state.trust_scale = min(max_scale, max(min_scale, state.trust_scale))
    return state.trust_scale, note


def _scale_to_max_abs(vector: list[float], max_abs: float) -> list[float]:
    max_value = max(abs(float(value)) for value in vector) if vector else 0.0
    if max_value <= float(max_abs) or max_value <= 1e-12:
        return list(vector)
    return _mul_scalar(vector, float(max_abs) / max_value)


def _safety_limited_force_delta(
    desired_delta_force: list[float],
    current_force: list[float],
    safety: SafetySettings,
    fraction: float,
) -> list[float]:
    limits = [safety.fx_abs_max_n, safety.fy_abs_max_n, safety.fz_abs_max_n]
    scale = 1.0
    for index in range(3):
        delta = float(desired_delta_force[index])
        if abs(delta) <= 1e-12:
            continue
        margin = max(0.0, limits[index] - abs(float(current_force[index])))
        allowed = max(0.02, margin * max(0.05, min(float(fraction), 1.0)))
        if abs(delta) > allowed:
            scale = min(scale, allowed / abs(delta))
    return _mul_scalar(desired_delta_force, scale)


def compute_decoupled_command(
    k: list[list[float]],
    target_force: list[float],
    current_force: list[float],
    state: DecoupledControlState,
    settings: DecoupledControlSettings,
    safety: SafetySettings,
    noise_norm: float = 0.0,
) -> DecoupledControlCommand:
    target_force = _vec(target_force)
    current_force = _vec(current_force)
    error = _sub(target_force, current_force)
    eta = auto_damping_eta(k, noise_norm=noise_norm, target_condition=settings.target_condition)
    kt = _transpose(k)
    kt_k = _matmul(kt, k)
    a = [[kt_k[row][col] + (eta * eta if row == col else 0.0) for col in range(3)] for row in range(3)]
    raw_delta = _solve_3x3(a, _matvec(kt, error))
    trust_scale, note = update_trust_scale(state, error, settings.style)
    limited = _mul_scalar(raw_delta, trust_scale)

    # 先限制位移，再限制预测力变化，避免病态矩阵或大误差导致单步过冲。
    limited = _scale_to_max_abs(limited, settings.max_step_mm)
    predicted = _matvec(k, limited)
    max_predicted = max(0.03, _norm(error) * 0.80)
    predicted = _scale_to_max_abs(predicted, max_predicted)
    predicted = _safety_limited_force_delta(predicted, current_force, safety, settings.safety_fraction)
    current_predicted_norm = _norm(_matvec(k, limited))
    if current_predicted_norm > 1e-12 and _norm(predicted) > 1e-12:
        limited = _mul_scalar(limited, min(1.0, _norm(predicted) / current_predicted_norm))

    quantized = []
    for value in limited:
        if abs(float(value)) < MM_PER_PULSE * 0.5:
            quantized.append(0.0)
        else:
            quantized.append(quantize_mm_to_pulses(float(value), min_pulses=settings.min_pulse))

    if all(abs(value) < 1e-12 for value in quantized) and _norm(error) > 1e-9:
        base = limited if _norm(limited) > 0 else raw_delta
        index = max(range(3), key=lambda i: abs(base[i]))
        sign = 1.0 if base[index] >= 0.0 else -1.0
        quantized[index] = sign * MM_PER_PULSE
        note = f"{note}；量化补 1 pulse".strip("；")

    pulses = {axis: mm_to_pulses(quantized[index]) for index, axis in enumerate(MOTOR_AXES)}
    delta_mm = {axis: quantized[index] for index, axis in enumerate(MOTOR_AXES)}
    state.previous_error = list(error)
    return DecoupledControlCommand(
        delta_mm=delta_mm,
        pulses=pulses,
        raw_delta=raw_delta,
        limited_delta=limited,
        quantized_delta=quantized,
        damping_eta=eta,
        trust_scale=trust_scale,
        error_norm=_norm(error),
        predicted_delta_force=_matvec(k, quantized),
        note=note,
    )


def k_to_flat_dict(k: list[list[float]], prefix: str = "K") -> dict[str, float]:
    out: dict[str, float] = {}
    for row, force_axis in enumerate(K_ROW_LABELS):
        for col, motor_axis in enumerate(K_COL_LABELS):
            out[f"{prefix}_{force_axis}_{motor_axis}"] = float(k[row][col])
    return out
