from __future__ import annotations

import csv
import math
from pathlib import Path
from statistics import median, pstdev
from typing import Iterable

from .models import FORCE_FIELDS, ForceSample


ZERO_BIAS = {field: 0.0 for field in FORCE_FIELDS}


def subtract_precomp_bias(sample: ForceSample, bias: dict[str, float]) -> ForceSample:
    """Return a metadata-preserving Mini45 sample with the six-axis bias removed."""
    return ForceSample(
        timestamp=sample.timestamp,
        monotonic_s=sample.monotonic_s,
        fx=sample.fx - float(bias.get("fx", 0.0)),
        fy=sample.fy - float(bias.get("fy", 0.0)),
        fz=sample.fz - float(bias.get("fz", 0.0)),
        mx=sample.mx - float(bias.get("mx", 0.0)),
        my=sample.my - float(bias.get("my", 0.0)),
        mz=sample.mz - float(bias.get("mz", 0.0)),
        sequence=sample.sequence,
        status=sample.status,
        source=sample.source,
    )


def full_workflow_precomp_ready(enabled: bool) -> bool:
    """Single policy point used by the full automatic workflow guard."""
    return enabled is True


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return float(ordered[lower])
    fraction = position - lower
    return float(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction)


def _covered_duration_s(samples: list[ForceSample]) -> float:
    if len(samples) < 2:
        return 0.0
    deltas = [
        current.monotonic_s - previous.monotonic_s
        for previous, current in zip(samples, samples[1:])
        if current.monotonic_s > previous.monotonic_s
    ]
    sample_interval = float(median(deltas)) if deltas else 0.0
    return max(0.0, samples[-1].monotonic_s - samples[0].monotonic_s + sample_interval)


def _sequence_drop_count(samples: Iterable[ForceSample]) -> int | None:
    sequences = [sample.sequence for sample in samples if sample.sequence is not None]
    if len(sequences) < 2:
        return None
    drop_count = 0
    for previous, current in zip(sequences, sequences[1:]):
        # Ignore repeats, reordering and counter wrap; count only observable forward gaps.
        if current > previous + 1:
            drop_count += current - previous - 1
    return drop_count


def compute_precomp_summary(
    samples: list[ForceSample],
    measurement_start_s: float,
    measurement_end_s: float,
    timestamp: str,
    discard_head_s: float = 5.0,
    min_valid_duration_s: float = 10.0,
) -> dict[str, object]:
    """Compute a robust six-axis bias from mapped, unfiltered Mini45 samples."""
    measurement_duration_s = max(0.0, measurement_end_s - measurement_start_s)
    valid_start_s = measurement_start_s + discard_head_s
    after_discard = [
        sample
        for sample in samples
        if valid_start_s <= sample.monotonic_s <= measurement_end_s
    ]
    valid_samples = [
        sample
        for sample in after_discard
        if all(math.isfinite(float(getattr(sample, field))) for field in FORCE_FIELDS)
    ]
    valid_samples.sort(key=lambda sample: sample.monotonic_s)
    valid_duration_s = _covered_duration_s(valid_samples)

    windows: dict[int, list[ForceSample]] = {}
    for sample in valid_samples:
        window_index = int(max(0.0, sample.monotonic_s - valid_start_s) // 1.0)
        windows.setdefault(window_index, []).append(sample)

    window_medians: dict[str, list[float]] = {field: [] for field in FORCE_FIELDS}
    for window_samples in windows.values():
        for field in FORCE_FIELDS:
            window_medians[field].append(float(median(getattr(sample, field) for sample in window_samples)))

    bias = {
        field: float(median(window_medians[field])) if window_medians[field] else 0.0
        for field in FORCE_FIELDS
    }
    std = {}
    spread = {}
    for field in FORCE_FIELDS:
        values = [float(getattr(sample, field)) for sample in valid_samples]
        std[field] = float(pstdev(values)) if len(values) > 1 else 0.0
        spread[field] = _percentile(values, 0.95) - _percentile(values, 0.05)

    reasons: list[str] = []
    quality = "pass"
    if valid_duration_s + 1e-6 < min_valid_duration_s or len(windows) < math.ceil(min_valid_duration_s):
        quality = "fail"
        reasons.append(
            f"丢弃前 {discard_head_s:g}s 后有效数据不足 {min_valid_duration_s:g}s"
        )

    invalid_count = len(after_discard) - len(valid_samples)
    invalid_ratio = invalid_count / len(after_discard) if after_discard else 0.0
    if invalid_ratio > 0.20:
        quality = "fail"
        reasons.append(f"非有限值样本占比过高（{invalid_ratio:.1%}）")
    elif invalid_count and quality != "fail":
        quality = "warning"
        reasons.append(f"已忽略 {invalid_count} 个非有限值样本")

    status_samples = [sample for sample in after_discard if sample.status is not None]
    nonzero_status_count = sum(sample.status != 0 for sample in status_samples)
    nonzero_status_ratio = nonzero_status_count / len(status_samples) if status_samples else 0.0
    if nonzero_status_ratio > 0.10:
        quality = "fail"
        reasons.append(f"Mini45 非零状态占比过高（{nonzero_status_ratio:.1%}）")
    elif nonzero_status_count and quality != "fail":
        quality = "warning"
        reasons.append(f"Mini45 存在 {nonzero_status_count} 个非零状态样本")

    drop_count = _sequence_drop_count(after_discard)
    if drop_count and quality == "pass":
        quality = "warning"
        reasons.append(f"检测到 {drop_count} 个 Mini45 序号丢包")

    summary: dict[str, object] = {
        "timestamp": timestamp,
        "duration_s": measurement_duration_s,
        "discard_head_s": discard_head_s,
        "sample_count": len(samples),
        "valid_sample_count": len(valid_samples),
        "valid_duration_s": valid_duration_s,
        "window_count": len(windows),
        "quality": quality,
        "reject_reason": "；".join(reasons),
        "status_sample_count": len(status_samples),
        "nonzero_status_count": nonzero_status_count,
        "drop_count": "" if drop_count is None else drop_count,
    }
    for field in FORCE_FIELDS:
        summary[f"bias_{field}"] = bias[field]
        summary[f"std_{field}"] = std[field]
        summary[f"p95_p5_{field}"] = spread[field]
    return summary


def save_precomp_summary(summary: dict[str, object], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as summary_file:
        writer = csv.DictWriter(summary_file, fieldnames=list(summary))
        writer.writeheader()
        writer.writerow(summary)
