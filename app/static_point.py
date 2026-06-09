from __future__ import annotations

import math
from dataclasses import dataclass, field

from .acquisition_profiles import STATIC_PRECISION
from .models import CapSample, CombinedSnapshot


def collection_tolerances(
    entry_tolerances: tuple[float, float, float] = (0.05, 0.05, 0.08),
    full_scales: tuple[float, float, float] = (4.0, 4.0, 10.0),
    full_scale_ratios: tuple[float, float, float] = (0.015, 0.015, 0.010),
) -> tuple[float, float, float]:
    """采集保持窗口同时考虑严格进入窗口和各轴满量程。"""
    return tuple(
        max(float(entry), float(full_scale) * float(ratio))
        for entry, full_scale, ratio in zip(entry_tolerances, full_scales, full_scale_ratios)
    )


@dataclass
class StaticPointCollector:
    required_cap_samples: int = 45
    stable_hold_s: float = 5.0
    timeout_s: float = 120.0
    max_retries: int = 2
    out_of_window_grace_s: float = 5.0
    preserve_progress_ratio: float = 0.80
    started_s: float = 0.0
    in_window_since_s: float = 0.0
    outside_window_since_s: float = 0.0
    retry_count: int = 0
    collecting: bool = False
    collection_paused: bool = False
    preserving_progress: bool = False
    cap_samples: list[CombinedSnapshot] = field(default_factory=list)
    seen_sequences: set[int] = field(default_factory=set)

    def begin(self, now_s: float) -> None:
        self.started_s = float(now_s)
        self._reset_collection_progress()

    def _reset_collection_progress(self) -> None:
        self.in_window_since_s = 0.0
        self.outside_window_since_s = 0.0
        self.collecting = False
        self.collection_paused = False
        self.preserving_progress = False
        self.cap_samples.clear()
        self.seen_sequences.clear()

    def update_force_state(
        self,
        now_s: float,
        *,
        in_window: bool,
        stable: bool,
        collection_in_window: bool | None = None,
        collection_stable: bool | None = None,
    ) -> str:
        now = float(now_s)
        if self.collecting:
            hold_ok = bool(collection_in_window if collection_in_window is not None else in_window)
            hold_stable = bool(collection_stable if collection_stable is not None else stable)
            if hold_ok and hold_stable:
                resumed = self.collection_paused
                self.outside_window_since_s = 0.0
                self.collection_paused = False
                self.preserving_progress = False
                return "collection_resumed" if resumed else ""
            if self.outside_window_since_s <= 0.0:
                self.outside_window_since_s = now
                self.collection_paused = True
                return "collection_paused"
            if now - self.outside_window_since_s < self.out_of_window_grace_s:
                self.collection_paused = True
                return ""
            if len(self.cap_samples) >= self.preserve_threshold:
                first_preserve = not self.preserving_progress
                self.collection_paused = True
                self.preserving_progress = True
                return "collection_preserved" if first_preserve else ""
            # 保留本次尝试的总超时起点，避免反复越界不断刷新 120 s 超时。
            self._reset_collection_progress()
            return "collection_reset"

        if not in_window or not stable:
            self.in_window_since_s = 0.0
            return ""
        if self.in_window_since_s <= 0.0:
            self.in_window_since_s = now
        if now - self.in_window_since_s >= self.stable_hold_s:
            self.collecting = True
            self.collection_paused = False
            return "collection_started"
        return ""

    def add_cap_sample(self, sample: CapSample) -> bool:
        if not self.collecting or self.collection_paused or self.complete:
            return False
        if sample.cap_profile != STATIC_PRECISION.name:
            return False
        values = (sample.c0, sample.c1, sample.c2, sample.c3, sample.c4)
        if not all(math.isfinite(float(value)) for value in values):
            return False
        if sample.sequence is None or sample.sequence in self.seen_sequences:
            return False
        self.seen_sequences.add(sample.sequence)
        self.cap_samples.append(CombinedSnapshot.from_cap(sample))
        return True

    @property
    def complete(self) -> bool:
        return len(self.cap_samples) >= self.required_cap_samples

    @property
    def preserve_threshold(self) -> int:
        return max(1, int(math.ceil(self.required_cap_samples * self.preserve_progress_ratio)))

    def timed_out(self, now_s: float) -> bool:
        return self.started_s > 0.0 and float(now_s) - self.started_s >= self.timeout_s

    def retry(self, now_s: float) -> bool:
        if self.retry_count >= self.max_retries:
            return False
        self.retry_count += 1
        self.begin(now_s)
        return True

    @property
    def stable_elapsed_s(self) -> float:
        if self.in_window_since_s <= 0.0:
            return 0.0
        import time

        return max(0.0, time.monotonic() - self.in_window_since_s)

    @property
    def outside_elapsed_s(self) -> float:
        if self.outside_window_since_s <= 0.0:
            return 0.0
        import time

        return max(0.0, time.monotonic() - self.outside_window_since_s)

    @property
    def time_bounds(self) -> tuple[float, float] | None:
        if not self.complete:
            return None
        selected = self.cap_samples[: self.required_cap_samples]
        return selected[0].monotonic_s, selected[-1].monotonic_s

    def selected_cap_samples(self) -> list[CombinedSnapshot]:
        return list(self.cap_samples[: self.required_cap_samples])
