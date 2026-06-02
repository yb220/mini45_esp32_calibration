from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from statistics import median

from .models import ForceSample


@dataclass
class ForceFilterSettings:
    enabled: bool = True
    cutoff_hz: float = 3.0
    median_window: int = 5


class ForceLowPassFilter:
    """Mini45 上位机滤波器，用于显示、力控和 K 辨识，不替代原始数据保存。"""

    def __init__(self) -> None:
        self._last: ForceSample | None = None
        self._median_window = 5
        self._history: dict[str, deque[float]] = {
            key: deque(maxlen=5) for key in ("fx", "fy", "fz", "mx", "my", "mz")
        }

    def reset(self) -> None:
        self._last = None
        for values in self._history.values():
            values.clear()

    def update(self, sample: ForceSample, settings: ForceFilterSettings) -> ForceSample:
        if not settings.enabled:
            self.reset()
            return sample

        window = max(1, int(settings.median_window))
        if window % 2 == 0:
            window += 1
        if window != self._median_window:
            self._median_window = window
            self._history = {key: deque(maxlen=window) for key in ("fx", "fy", "fz", "mx", "my", "mz")}
            self._last = None

        med_values = {}
        for field in self._history:
            values = self._history[field]
            values.append(float(getattr(sample, field)))
            med_values[field] = float(median(values))

        if self._last is None or sample.monotonic_s <= self._last.monotonic_s:
            filtered = self._copy_with_values(sample, med_values)
            self._last = filtered
            return filtered

        dt = sample.monotonic_s - self._last.monotonic_s
        if dt > 1.0:
            # 数据长时间中断后直接重置滤波状态，避免旧状态拖累新数据。
            filtered = self._copy_with_values(sample, med_values)
            self._last = filtered
            return filtered

        cutoff_hz = max(0.05, float(settings.cutoff_hz))
        tau = 1.0 / (2.0 * math.pi * cutoff_hz)
        alpha = dt / (tau + dt)
        filtered_values = {}
        for field, med_value in med_values.items():
            previous = float(getattr(self._last, field))
            filtered_values[field] = previous + alpha * (med_value - previous)

        filtered = self._copy_with_values(sample, filtered_values)
        self._last = filtered
        return filtered

    def _copy_with_values(self, sample: ForceSample, values: dict[str, float]) -> ForceSample:
        return ForceSample(
            timestamp=sample.timestamp,
            monotonic_s=sample.monotonic_s,
            fx=values["fx"],
            fy=values["fy"],
            fz=values["fz"],
            mx=values["mx"],
            my=values["my"],
            mz=values["mz"],
            sequence=sample.sequence,
            status=sample.status,
            source=sample.source,
        )
