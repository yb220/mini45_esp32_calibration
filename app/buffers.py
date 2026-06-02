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
        window_items: list[CombinedSnapshot] = []
        with self._lock:
            # 数据按时间顺序追加，取最近窗口时从尾部反向扫描即可，避免每次扫完整缓存。
            for item in reversed(self._items):
                if item.monotonic_s > end_s:
                    continue
                if item.monotonic_s < start_s:
                    break
                window_items.append(item)
        window_items.reverse()
        return window_items

    def all(self) -> list[CombinedSnapshot]:
        with self._lock:
            return list(self._items)

    def clear(self) -> None:
        with self._lock:
            self._items.clear()

    def extend(self, items: Iterable[CombinedSnapshot]) -> None:
        for item in items:
            self.append(item)
