import csv
import math
import tempfile
import unittest
from pathlib import Path

from app.mini45_precomp import (
    compute_precomp_summary,
    full_workflow_precomp_ready,
    save_precomp_summary,
    subtract_precomp_bias,
)
from app.models import CombinedSnapshot, ForceSample
from app.recorder import CsvRecorder


BIAS = {
    "fx": 0.12,
    "fy": -0.23,
    "fz": 0.34,
    "mx": 0.004,
    "my": -0.005,
    "mz": 0.006,
}


class Mini45PrecompTests(unittest.TestCase):
    def test_robust_bias_uses_post_discard_window_medians(self):
        samples = []
        rate_hz = 100
        for index in range(60 * rate_hz):
            seconds = index / rate_hz
            warmup = 2.0 if seconds < 5.0 else 0.0
            noise = 0.002 * math.sin(index * 0.31)
            outlier = 20.0 if seconds >= 5.0 and index % rate_hz == 0 else 0.0
            samples.append(
                ForceSample(
                    timestamp="t",
                    monotonic_s=seconds,
                    fx=BIAS["fx"] + warmup + noise + outlier,
                    fy=BIAS["fy"] + warmup - noise,
                    fz=BIAS["fz"] + warmup + noise,
                    mx=BIAS["mx"] + noise * 0.01,
                    my=BIAS["my"] - noise * 0.01,
                    mz=BIAS["mz"] + noise * 0.01,
                    sequence=index,
                    status=0,
                )
            )

        summary = compute_precomp_summary(samples, 0.0, 60.0, "2026-01-01T00:00:00+08:00")

        self.assertEqual(summary["quality"], "pass")
        self.assertEqual(summary["sample_count"], 6000)
        self.assertEqual(summary["valid_sample_count"], 5500)
        self.assertGreaterEqual(summary["valid_duration_s"], 54.99)
        for field, expected in BIAS.items():
            self.assertAlmostEqual(summary[f"bias_{field}"], expected, delta=0.001)

    def test_subtract_bias_preserves_sample_metadata(self):
        sample = ForceSample(
            "t",
            12.5,
            fx=1.12,
            fy=1.77,
            fz=2.34,
            mx=0.104,
            my=0.195,
            mz=0.306,
            sequence=42,
            status=3,
            source="mini45-test",
        )

        corrected = subtract_precomp_bias(sample, BIAS)

        self.assertAlmostEqual(corrected.fx, 1.0)
        self.assertAlmostEqual(corrected.fy, 2.0)
        self.assertAlmostEqual(corrected.fz, 2.0)
        self.assertAlmostEqual(corrected.mx, 0.1)
        self.assertAlmostEqual(corrected.my, 0.2)
        self.assertAlmostEqual(corrected.mz, 0.3)
        self.assertEqual((corrected.sequence, corrected.status, corrected.source), (42, 3, "mini45-test"))
        self.assertAlmostEqual(sample.fx, 1.12)

    def test_full_workflow_requires_completed_precomp(self):
        self.assertFalse(full_workflow_precomp_ready(False))
        self.assertTrue(full_workflow_precomp_ready(True))

    def test_short_measurement_fails_and_summary_can_be_saved(self):
        samples = [
            ForceSample("t", index / 10, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, status=0)
            for index in range(120)
        ]
        summary = compute_precomp_summary(samples, 0.0, 12.0, "t")
        self.assertEqual(summary["quality"], "fail")

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "precomp" / "summary.csv"
            save_precomp_summary(summary, path)
            with path.open(encoding="utf-8-sig") as summary_file:
                row = next(csv.DictReader(summary_file))
            self.assertEqual(row["quality"], "fail")
            self.assertIn("bias_fx", row)
            self.assertIn("p95_p5_mz", row)

    def test_zero_drift_recording_still_keeps_raw_mini45_values(self):
        raw = ForceSample("t", 1.0, 1.12, 1.77, 2.34, 0.104, 0.195, 0.306)
        corrected = subtract_precomp_bias(raw, BIAS)
        snapshot = CombinedSnapshot.from_force(corrected, raw_sample=raw)

        with tempfile.TemporaryDirectory() as tmp:
            recorder = CsvRecorder(Path(tmp))
            recorder.start()
            zero_path = recorder.start_zero_drift_timeseries()
            recorder.write_zero_drift_raw(snapshot)
            recorder.stop()
            with zero_path.open(encoding="utf-8-sig") as zero_file:
                row = next(csv.DictReader(zero_file))

        self.assertAlmostEqual(float(row["fx"]), 1.0)
        self.assertAlmostEqual(float(row["mini45_raw_fx"]), 1.12)


if __name__ == "__main__":
    unittest.main()
