import unittest

from app.buffers import SampleBuffer
from app.models import CombinedSnapshot


class SampleBufferTests(unittest.TestCase):
    def test_window_returns_recent_items_in_time_order(self):
        buffer = SampleBuffer(max_seconds=120.0)
        for index in range(10):
            buffer.append(CombinedSnapshot(timestamp=str(index), monotonic_s=float(index), source="test"))

        items = buffer.window(end_s=9.0, seconds=3.0)

        self.assertEqual([item.monotonic_s for item in items], [6.0, 7.0, 8.0, 9.0])


if __name__ == "__main__":
    unittest.main()
