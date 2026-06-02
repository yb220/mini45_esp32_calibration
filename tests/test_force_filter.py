import unittest

from app.force_filter import ForceFilterSettings, ForceLowPassFilter
from app.models import ForceSample


def sample_at(t: float, fz: float) -> ForceSample:
    return ForceSample("t", t, fx=0.0, fy=0.0, fz=fz, mx=0.0, my=0.0, mz=0.0)


class ForceFilterTests(unittest.TestCase):
    def test_disabled_filter_returns_raw_sample(self):
        filt = ForceLowPassFilter()
        raw = sample_at(1.0, 2.0)
        out = filt.update(raw, ForceFilterSettings(enabled=False))
        self.assertIs(out, raw)

    def test_low_pass_smooths_step_change(self):
        filt = ForceLowPassFilter()
        settings = ForceFilterSettings(enabled=True, cutoff_hz=1.0, median_window=1)
        filt.update(sample_at(1.0, 0.0), settings)
        out = filt.update(sample_at(1.1, 1.0), settings)
        self.assertGreater(out.fz, 0.0)
        self.assertLess(out.fz, 1.0)

    def test_median_window_rejects_single_spike(self):
        filt = ForceLowPassFilter()
        settings = ForceFilterSettings(enabled=True, cutoff_hz=30.0, median_window=3)
        filt.update(sample_at(1.00, 0.0), settings)
        filt.update(sample_at(1.01, 0.0), settings)
        out = filt.update(sample_at(1.02, 10.0), settings)
        self.assertLess(out.fz, 1.0)


if __name__ == "__main__":
    unittest.main()
