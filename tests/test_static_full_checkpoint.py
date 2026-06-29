import csv
import tempfile
import unittest
from pathlib import Path

from app.static_full_checkpoint import (
    legacy_completed_point_count,
    load_checkpoint,
    load_last_valid_k_result,
    load_latest_precomp,
    save_checkpoint,
)


class StaticFullCheckpointTests(unittest.TestCase):
    def test_checkpoint_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            save_checkpoint(folder, {"status": "running", "targets": [{"point_index": 1}], "sequence_index": 7})
            restored = load_checkpoint(folder)
            self.assertEqual(restored["status"], "running")
            self.assertEqual(restored["sequence_index"], 7)
            self.assertEqual(restored["version"], 1)

    def test_legacy_progress_counts_timeout_markers(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            with (folder / "calibration_points.csv").open("w", newline="", encoding="utf-8-sig") as target:
                writer = csv.DictWriter(target, fieldnames=["marker_id"])
                writer.writeheader()
                writer.writerow({"marker_id": 1})
            with (folder / "markers.csv").open("w", newline="", encoding="utf-8-sig") as target:
                writer = csv.DictWriter(target, fieldnames=["marker_id", "branch"])
                writer.writeheader()
                writer.writerow({"marker_id": 1, "branch": "loading"})
                writer.writerow({"marker_id": 2, "branch": "unloading"})
            self.assertEqual(legacy_completed_point_count(folder), 2)

    def test_loads_latest_precomp_and_valid_k(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            bias_fields = {f"bias_{field}": index / 10 for index, field in enumerate(("fx", "fy", "fz", "mx", "my", "mz"))}
            with (folder / "mini45_precomp_summary_20260101_000000.csv").open("w", newline="", encoding="utf-8-sig") as target:
                writer = csv.DictWriter(target, fieldnames=["quality", *bias_fields])
                writer.writeheader()
                writer.writerow({"quality": "pass", **bias_fields})
            k_row = {
                "valid": "True",
                "debug": "False",
                "condition": "1",
                "noise_norm": "0.01",
                "delta_X_mm": "0.1",
                "delta_Y_mm": "0.1",
                "delta_Z_mm": "0.1",
                "singular_1": "1",
                "singular_2": "1",
                "singular_3": "1",
            }
            for force_axis in ("Fx", "Fy", "Fz"):
                for motor_axis in ("X", "Y", "Z"):
                    k_row[f"K_{force_axis}_{motor_axis}"] = "1" if force_axis[1].upper() == motor_axis else "0"
            for motor_axis in ("X", "Y", "Z"):
                for force_axis in ("Fx", "Fy", "Fz"):
                    k_row[f"before_{motor_axis}_{force_axis}"] = "0"
                    k_row[f"after_{motor_axis}_{force_axis}"] = "0.1"
            with (folder / "force_control_k.csv").open("w", newline="", encoding="utf-8-sig") as target:
                writer = csv.DictWriter(target, fieldnames=list(k_row))
                writer.writeheader()
                writer.writerow(k_row)

            precomp = load_latest_precomp(folder)
            self.assertIsNotNone(precomp)
            self.assertEqual(precomp[1], "pass")
            result = load_last_valid_k_result(folder)
            self.assertIsNotNone(result)
            self.assertTrue(result.valid)
            self.assertEqual(result.k[0], [1.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
