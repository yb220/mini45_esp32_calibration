from __future__ import annotations

import math
from dataclasses import dataclass, field

from .acquisition_profiles import STATIC_PRECISION
from .models import CapSample, CombinedSnapshot


@dataclass
class StaticPointCollector:
    required_cap_samples: int = 45
    stable_hold_s: float = 5.0
    timeout_s: float = 120.0
    max_retries: int = 2
    started_s: float = 0.0
    in_window_since_s: float = 0.0
    retry_count: int = 0
    collecting: bool = False
    cap_samples: list[CombinedSnapshot] = field(default_factory=list)
    seen_sequences: set[int] = field(default_factory=set)

    def begin(self, now_s: float) -> None:
        self.started_s = float(now_s)
        self.in_window_since_s = 0.0
        self.collecting = False
        self.cap_samples.clear()
        self.seen_sequences.clear()

    def update_force_state(self, now_s: float, *, in_window: bool, stable: bool) -> None:
        if not in_window or not stable:
            self.in_window_since_s = 0.0
            self.collecting = False
            self.cap_samples.clear()
            self.seen_sequences.clear()
            return
        if self.in_window_since_s <= 0.0:
            self.in_window_since_s = float(now_s)
        if float(now_s) - self.in_window_since_s >= self.stable_hold_s:
            self.collecting = True

    def add_cap_sample(self, sample: CapSample) -> bool:
        if not self.collecting or self.complete:
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
    def time_bounds(self) -> tuple[float, float] | None:
        if not self.complete:
            return None
        selected = self.cap_samples[: self.required_cap_samples]
        return selected[0].monotonic_s, selected[-1].monotonic_s

    def selected_cap_samples(self) -> list[CombinedSnapshot]:
        return list(self.cap_samples[: self.required_cap_samples])
