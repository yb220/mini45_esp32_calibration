from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path
from typing import Any

from .force_control import KIdentificationResult, MOTOR_AXES
from .models import FORCE_FIELDS


CHECKPOINT_FILENAME = "static_full_checkpoint.json"
CHECKPOINT_VERSION = 1


def checkpoint_path(output_dir: Path) -> Path:
    return Path(output_dir) / CHECKPOINT_FILENAME


def save_checkpoint(output_dir: Path, payload: dict[str, Any]) -> Path:
    path = checkpoint_path(output_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    document = dict(payload)
    document["version"] = CHECKPOINT_VERSION
    document["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)
    return path


def load_checkpoint(output_dir: Path) -> dict[str, Any]:
    path = checkpoint_path(output_dir)
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("全静态检查点格式无效")
    if int(document.get("version", 0)) != CHECKPOINT_VERSION:
        raise ValueError(f"不支持的全静态检查点版本：{document.get('version')}")
    targets = document.get("targets")
    if not isinstance(targets, list) or not targets:
        raise ValueError("全静态检查点缺少目标序列")
    return document


def legacy_completed_point_count(output_dir: Path) -> int:
    calibration_path = Path(output_dir) / "calibration_points.csv"
    marker_path = Path(output_dir) / "markers.csv"
    calibration_count = 0
    marker_count = 0
    if calibration_path.exists():
        with calibration_path.open(encoding="utf-8-sig", newline="") as source:
            calibration_count = sum(1 for row in csv.DictReader(source) if any(str(value).strip() for value in row.values()))
    if marker_path.exists():
        with marker_path.open(encoding="utf-8-sig", newline="") as source:
            marker_count = sum(
                1
                for row in csv.DictReader(source)
                if row.get("branch") in {"loading", "unloading"}
            )
    return max(calibration_count, marker_count)


def legacy_last_marker_id(output_dir: Path) -> int:
    path = Path(output_dir) / "markers.csv"
    if not path.exists():
        return 0
    with path.open(encoding="utf-8-sig", newline="") as source:
        values = [int(row["marker_id"]) for row in csv.DictReader(source) if str(row.get("marker_id", "")).isdigit()]
    return max(values, default=0)


def load_last_force_mapping(output_dir: Path) -> dict[str, str] | None:
    path = Path(output_dir) / "force_frame_mapping.csv"
    if not path.exists():
        return None
    with path.open(encoding="utf-8-sig", newline="") as source:
        rows = list(csv.DictReader(source))
    return rows[-1] if rows else None


def load_latest_precomp(output_dir: Path) -> tuple[dict[str, float], str] | None:
    output_dir = Path(output_dir)
    candidates = sorted(
        list(output_dir.glob("mini45_precomp_summary_*.csv"))
        + list((output_dir.parent / "precomp").glob("mini45_precomp_summary_*.csv"))
    )
    for path in reversed(candidates):
        with path.open(encoding="utf-8-sig", newline="") as source:
            row = next(csv.DictReader(source), None)
        if not row or row.get("quality") == "fail":
            continue
        bias = {field: float(row[f"bias_{field}"]) for field in FORCE_FIELDS}
        if all(math.isfinite(value) for value in bias.values()):
            return bias, str(row.get("quality") or "warning")
    return None


def _as_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "是"}


def load_last_valid_k_result(output_dir: Path) -> KIdentificationResult | None:
    path = Path(output_dir) / "force_control_k.csv"
    if not path.exists():
        return None
    with path.open(encoding="utf-8-sig", newline="") as source:
        rows = list(csv.DictReader(source))
    for row in reversed(rows):
        if not _as_bool(row.get("valid")):
            continue
        try:
            k = [
                [float(row[f"K_{force_axis}_{motor_axis}"]) for motor_axis in MOTOR_AXES]
                for force_axis in ("Fx", "Fy", "Fz")
            ]
            before = {
                motor_axis: [float(row[f"before_{motor_axis}_{force_axis}"]) for force_axis in ("Fx", "Fy", "Fz")]
                for motor_axis in MOTOR_AXES
            }
            after = {
                motor_axis: [float(row[f"after_{motor_axis}_{force_axis}"]) for force_axis in ("Fx", "Fy", "Fz")]
                for motor_axis in MOTOR_AXES
            }
            deltas = {motor_axis: float(row[f"delta_{motor_axis}_mm"]) for motor_axis in MOTOR_AXES}
            singular = [float(row[f"singular_{index}"]) for index in range(1, 4) if row.get(f"singular_{index}")]
            result = KIdentificationResult(
                k=k,
                before_means=before,
                after_means=after,
                before_stds={axis: [0.0, 0.0, 0.0] for axis in MOTOR_AXES},
                after_stds={axis: [0.0, 0.0, 0.0] for axis in MOTOR_AXES},
                deltas_mm=deltas,
                singular_values=singular,
                condition=float(row.get("condition") or 0.0),
                noise_norm=float(row.get("noise_norm") or 0.0),
                valid=True,
                reject_reason=str(row.get("reject_reason") or ""),
                debug=_as_bool(row.get("debug")),
            )
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for values in result.k for value in values):
            return result
    return None
