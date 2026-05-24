import csv
import tempfile
import unittest
from pathlib import Path

from app.models import CalibrationPoint, CombinedSnapshot, ExperimentMeta
from app.recorder import (
    CALIBRATION_FIELDS,
    FORCE_CONTROL_K_FIELDS,
    FORCE_CONTROL_LOG_FIELDS,
    MARKER_FIELDS,
    RAW_FIELDS,
    TRAINING_MARKER_FIELDS,
    CsvRecorder,
)


class RecorderTests(unittest.TestCase):
    def test_recorder_writes_schema(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            recorder = CsvRecorder(out)
            recorder.start()
            recorder.write_raw(CombinedSnapshot(timestamp="t", monotonic_s=1.0, source="test", fx=1.0))
            recorder.write_marker(1, ExperimentMeta(experiment_id="e1"))
            recorder.write_calibration_point(
                CalibrationPoint(
                    timestamp_start="a",
                    timestamp_end="b",
                    experiment_id="e1",
                    cycle_id="c1",
                    branch="loading",
                    axis="Fz",
                    direction="none",
                    preload_N=0.0,
                    target_Fx=0.0,
                    target_Fy=0.0,
                    target_Fz=1.0,
                    Fx_mean=0.0,
                    Fy_mean=0.0,
                    Fz_mean=1.0,
                    Mx_mean=0.0,
                    My_mean=0.0,
                    Mz_mean=0.0,
                    Fx_std=0.0,
                    Fy_std=0.0,
                    Fz_std=0.0,
                    C0_mean=1.0,
                    C1_mean=2.0,
                    C2_mean=3.0,
                    C3_mean=4.0,
                    C4_mean=5.0,
                    C0_std=0.0,
                    C1_std=0.0,
                    C2_std=0.0,
                    C3_std=0.0,
                    C4_std=0.0,
                    marker_id=1,
                    valid=True,
                    reject_reason="",
                    note="",
                )
            )
            recorder.stop()

            with (out / "raw_timeseries.csv").open(encoding="utf-8-sig") as f:
                self.assertEqual(next(csv.reader(f)), RAW_FIELDS)
            with (out / "markers.csv").open(encoding="utf-8-sig") as f:
                self.assertEqual(next(csv.reader(f)), MARKER_FIELDS)
            with (out / "calibration_points.csv").open(encoding="utf-8-sig") as f:
                self.assertEqual(next(csv.reader(f)), CALIBRATION_FIELDS)
            self.assertFalse((out / "zero_drift_summary.csv").exists())
            with (out / "training_raw_timeseries.csv").open(encoding="utf-8-sig") as f:
                self.assertEqual(next(csv.reader(f)), RAW_FIELDS)
            with (out / "training_markers.csv").open(encoding="utf-8-sig") as f:
                self.assertEqual(next(csv.reader(f)), TRAINING_MARKER_FIELDS)
            with (out / "force_control_k.csv").open(encoding="utf-8-sig") as f:
                self.assertEqual(next(csv.reader(f)), FORCE_CONTROL_K_FIELDS)
            with (out / "force_control_log.csv").open(encoding="utf-8-sig") as f:
                self.assertEqual(next(csv.reader(f)), FORCE_CONTROL_LOG_FIELDS)

    def test_zero_drift_timeseries_uses_raw_schema_and_suffix(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = CsvRecorder(Path(tmp))
            recorder.start()
            first = recorder.start_zero_drift_timeseries()
            recorder.write_zero_drift_raw(CombinedSnapshot(timestamp="t", monotonic_s=1.0, source="mini45", fx=0.1))
            recorder.stop_zero_drift_timeseries()
            second = recorder.start_zero_drift_timeseries()
            recorder.stop()

            self.assertEqual(first.name, "zero_drift_timeseries_001.csv")
            self.assertEqual(second.name, "zero_drift_timeseries_002.csv")
            with first.open(encoding="utf-8-sig") as f:
                self.assertEqual(next(csv.reader(f)), RAW_FIELDS)

    def test_training_files_are_separate(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            recorder = CsvRecorder(out)
            recorder.start()
            recorder.write_training_raw(CombinedSnapshot(timestamp="t", monotonic_s=1.0, source="esp32", c0=1.0))
            recorder.write_training_marker(
                1,
                ExperimentMeta(experiment_id="train", cycle_id="training_001", axis="combined", target_fx=0.1),
                trajectory_type="fx_roundtrip",
                phase="moving",
                target_shear_n=0.1,
                target_angle_deg=0.0,
            )
            recorder.start_training_files()
            recorder.write_training_raw(CombinedSnapshot(timestamp="t2", monotonic_s=2.0, source="mini45", fx=1.0))
            recorder.stop()

            with (out / "training_raw_timeseries.csv").open(encoding="utf-8-sig") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0], RAW_FIELDS)
            self.assertEqual(len(rows), 3)
            with (out / "training_markers.csv").open(encoding="utf-8-sig") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0], TRAINING_MARKER_FIELDS)
            self.assertEqual(len(rows), 2)

    def test_force_control_files_are_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            recorder = CsvRecorder(out)
            recorder.start()
            recorder.write_force_control_k({"experiment_id": "e1", "valid": True, "K_Fx_X": 1.0})
            recorder.write_force_control_log({"experiment_id": "e1", "cycle_id": "c1", "delta_X_mm": 0.005})
            recorder.stop()

            with (out / "force_control_k.csv").open(encoding="utf-8-sig") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(rows[0]["experiment_id"], "e1")
            self.assertEqual(rows[0]["K_Fx_X"], "1.0")
            with (out / "force_control_log.csv").open(encoding="utf-8-sig") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(rows[0]["cycle_id"], "c1")


if __name__ == "__main__":
    unittest.main()
