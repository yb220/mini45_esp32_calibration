import unittest

from app.force_frame import AxisFrameMap, ForceFrameMapping, transform_force_sample
from app.force_control import identify_k_matrix
from app.models import CombinedSnapshot, ForceSample


class ForceFrameMappingTests(unittest.TestCase):
    def test_transform_force_sample_maps_axis_and_sign(self):
        raw = ForceSample(
            timestamp="t",
            monotonic_s=1.0,
            fx=1.0,
            fy=2.0,
            fz=3.0,
            mx=0.1,
            my=0.2,
            mz=0.3,
            sequence=7,
            status=0,
        )
        mapping = ForceFrameMapping(
            sensor_fx=AxisFrameMap("Fz", -1),
            sensor_fy=AxisFrameMap("Fy", 1),
            sensor_fz=AxisFrameMap("Fx", 1),
        )

        sample = transform_force_sample(raw, mapping)

        self.assertEqual(sample.fx, -3.0)
        self.assertEqual(sample.fy, 2.0)
        self.assertEqual(sample.fz, 1.0)
        self.assertEqual(sample.mx, -0.3)
        self.assertEqual(sample.my, 0.2)
        self.assertEqual(sample.mz, 0.1)
        self.assertEqual(sample.sequence, 7)

    def test_duplicate_mini45_axes_are_rejected(self):
        mapping = ForceFrameMapping(
            sensor_fx=AxisFrameMap("Fx", 1),
            sensor_fy=AxisFrameMap("Fx", -1),
            sensor_fz=AxisFrameMap("Fz", 1),
        )

        with self.assertRaises(ValueError):
            mapping.validate()

    def test_identity_mapping_preserves_values(self):
        raw = ForceSample(
            timestamp="t",
            monotonic_s=1.0,
            fx=1.0,
            fy=2.0,
            fz=3.0,
            mx=4.0,
            my=5.0,
            mz=6.0,
        )

        sample = transform_force_sample(raw, ForceFrameMapping.identity())

        self.assertEqual((sample.fx, sample.fy, sample.fz), (1.0, 2.0, 3.0))
        self.assertEqual((sample.mx, sample.my, sample.mz), (4.0, 5.0, 6.0))

    def test_snapshot_keeps_sensor_and_raw_mini45_values(self):
        raw = ForceSample("t", 1.0, fx=1.0, fy=2.0, fz=3.0, mx=4.0, my=5.0, mz=6.0)
        mapped = ForceSample("t", 1.0, fx=-3.0, fy=2.0, fz=1.0, mx=-6.0, my=5.0, mz=4.0)

        snapshot = CombinedSnapshot.from_force(mapped, raw_sample=raw)

        self.assertEqual((snapshot.fx, snapshot.fy, snapshot.fz), (-3.0, 2.0, 1.0))
        self.assertEqual((snapshot.mini45_raw_fx, snapshot.mini45_raw_fy, snapshot.mini45_raw_fz), (1.0, 2.0, 3.0))

    def test_k_column_can_be_identified_from_sensor_frame_force(self):
        mapping = ForceFrameMapping(
            sensor_fx=AxisFrameMap("Fz", 1),
            sensor_fy=AxisFrameMap("Fy", 1),
            sensor_fz=AxisFrameMap("Fx", 1),
        )
        before_x = transform_force_sample(ForceSample("t", 1.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0), mapping)
        after_x = transform_force_sample(ForceSample("t", 1.0, 1.0, 2.0, 4.0, 0.0, 0.0, 0.0), mapping)

        result = identify_k_matrix(
            before_means={
                "X": [before_x.fx, before_x.fy, before_x.fz],
                "Y": [0.0, 0.0, 0.0],
                "Z": [0.0, 0.0, 0.0],
            },
            after_means={
                "X": [after_x.fx, after_x.fy, after_x.fz],
                "Y": [0.0, 1.0, 0.0],
                "Z": [0.0, 0.0, 1.0],
            },
            before_stds={axis: [0.0, 0.0, 0.0] for axis in ("X", "Y", "Z")},
            after_stds={axis: [0.0, 0.0, 0.0] for axis in ("X", "Y", "Z")},
            deltas_mm={"X": 0.5, "Y": 1.0, "Z": 1.0},
            condition_limit=1000.0,
        )

        self.assertAlmostEqual(result.k[0][0], 2.0)


if __name__ == "__main__":
    unittest.main()
