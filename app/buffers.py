from __future__ import annotations

from collections import deque
from threading import Lock
from typing import Iterable

from .models import CombinedSnapshot


class SampleBuffer:
    def __init__(self, max_seconds: float = 300.0):
        self.max_seconds = max_seconds
        self._items: deque[CombinedSnapshot] = deque()
        self._lock = Lock()

    def append(self, item: CombinedSnapshot) -> None:
        with self._lock:
            self._items.append(item)
            cutoff = item.monotonic_s - self.max_seconds
            while self._items and self._items[0].monotonic_s < cutoff:
                self._items.popleft()

    def latest(self) -> CombinedSnapshot | None:
        with self._lock:
            return self._items[-1] if self._items else None

    def window(self, end_s: float, seconds: float) -> list[CombinedSnapshot]:
        start_s = end_s - seconds
        with self._lock:
            return [item for item in self._items if start_s <= item.monotonic_s <= end_s]

    def all(self) -> list[CombinedSnapshot]:
        with self._lock:
            return list(self._items)

    def clear(self) -> None:
        with self._lock:
            self._items.clear()

    def extend(self, items: Iterable[CombinedSnapshot]) -> None:
        for item in items:
            self.append(item)
