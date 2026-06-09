import unittest
import random

from app.calibration import (
    choose_control_axis,
    generate_fz_sequence,
    generate_shear_sequence,
    generate_three_axis_sequence,
    generate_training_trajectory,
    advance_ramped_force_target,
    parse_force_levels,
    training_target_reached,
    training_target_timed_out,
)
from app.models import CombinedSnapshot, ExperimentMeta, ForceSample, SafetySettings, StabilitySettings
from app.stability import evaluate_three_axis_stability


class CalibrationFlowTests(unittest.TestCase):
    def test_generate_fz_loading_unloading_sequence(self):
        points = generate_fz_sequence(max_force=2.0, step=1.0, cycles=1)
        self.assertEqual([p.target_fz for p in points], [0.0, 1.0, 2.0, 1.0, 0.0])
        self.assertEqual([p.branch for p in points], ["loading", "loading", "loading", "unloading", "unloading"])
        self.assertTrue(all(p.axis == "Fz" for p in points))

    def test_cycle_id_is_generated_from_sequence_cycle(self):
        point = generate_fz_sequence(max_force=1.0, step=1.0, cycles=2)[4]
        meta = point.to_meta(ExperimentMeta(experiment_id="sensor01_mount01", cycle_id="ignored"))
        self.assertEqual(meta.experiment_id, "sensor01_mount01")
        self.assertEqual(meta.cycle_id, "cycle_002")

    def test_generate_fx_positive_negative_sequence(self):
        points = generate_shear_sequence("Fx", max_force=1.2, step=0.6, target_fz=3.0, direction_mode="both", cycles=1)
        self.assertEqual([p.target_fx for p in points], [0.0, 0.6, 1.2, 0.6, 0.0, -0.0, -0.6, -1.2, -0.6, -0.0])
        self.assertEqual(points[1].direction, "positive")
        self.assertEqual(points[6].direction, "negative")
        self.assertTrue(all(p.target_fz == 3.0 for p in points))

    def test_generate_three_axis_sequence_groups_axes(self):
        points = generate_three_axis_sequence(
            fz_max_force=1.0,
            fz_step=1.0,
            shear_max_force=0.6,
            shear_step=0.6,
            target_fz=2.0,
            shear_direction_mode="positive",
            cycles=2,
        )
        self.assertEqual([p.axis for p in points[:6]], ["Fz"] * 6)
        self.assertEqual([p.axis for p in points[6:12]], ["Fx"] * 6)
        self.assertEqual([p.axis for p in points[12:]], ["Fy"] * 6)
        self.assertEqual([p.cycle_index for p in points[:6]], [1, 1, 1, 2, 2, 2])
        self.assertEqual([p.cycle_index for p in points[6:12]], [1, 1, 1, 2, 2, 2])
        self.assertTrue(all(p.target_fz == 2.0 for p in points if p.axis in {"Fx", "Fy"}))

    def test_choose_control_axis_uses_largest_normalized_error(self):
        force = ForceSample("t", 1.0, fx=0.2, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0)
        meta = ExperimentMeta(target_fx=0.3, target_fy=0.0, target_fz=1.0)
        settings = StabilitySettings(tolerance_fx=0.1, tolerance_fy=0.1, tolerance_fz=0.2)
        choice = choose_control_axis(force, meta, settings)
        self.assertEqual(choice.axis, "Fz")
        self.assertFalse(choice.all_in_window)

    def test_non_target_axis_error_is_not_safety_stop(self):
        samples = [
            CombinedSnapshot("t", float(i), "mini45", fx=3.6, fy=0.3, fz=0.0, mx=0.0, my=0.0, mz=0.0)
            for i in range(5)
        ]
        meta = ExperimentMeta(axis="Fx", target_fx=3.6, target_fy=0.0, target_fz=0.0)
        result = evaluate_three_axis_stability(samples, meta, StabilitySettings(tolerance_fy=0.05), SafetySettings())
        self.assertTrue(result.safe)
        self.assertFalse(result.stable)

    def test_parse_force_levels(self):
        self.assertEqual(parse_force_levels("7,3，5 3"), [3.0, 5.0, 7.0])

    def test_generate_training_fx_roundtrip(self):
        targets = generate_training_trajectory(
            fz_levels=[0.5],
            shear_max=0.3,
            trajectory_type="fx_roundtrip",
            target_step_n=0.3,
        )
        target_phase = [target for target in targets if target.phase == "target"]
        self.assertEqual([target.direction for target in target_phase[:4]], ["positive", "positive", "negative", "negative"])
        self.assertAlmostEqual(target_phase[0].target_fx, 0.3)
        self.assertAlmostEqual(target_phase[2].target_fx, -0.3)
        self.assertTrue(any(target.phase == "recovery" for target in targets))

    def test_generate_training_diagonal_roundtrip(self):
        targets = generate_training_trajectory(
            fz_levels=[1.0],
            shear_max=1.0,
            trajectory_type="diagonal_roundtrip",
            target_step_n=1.0,
        )
        angles = [target.target_angle_deg for target in targets if target.direction.startswith("angle_")]
        self.assertIn(45.0, angles)
        self.assertIn(315.0, angles)

    def test_generate_training_fy_roundtrip(self):
        targets = generate_training_trajectory(
            fz_levels=[0.5],
            shear_max=0.3,
            trajectory_type="fy_roundtrip",
            target_step_n=0.3,
        )
        target_phase = [target for target in targets if target.phase == "target"]
        self.assertAlmostEqual(target_phase[0].target_fy, 0.3)
        self.assertAlmostEqual(target_phase[2].target_fy, -0.3)

    def test_training_random_targets_are_inside_shear_disk_and_not_fixed(self):
        first = generate_training_trajectory([1.0], 1.0, "random_perturb", random_points=8)
        second = generate_training_trajectory([1.0], 1.0, "random_perturb", random_points=8)
        first_random = [target for target in first if target.direction.startswith("random_")]
        second_random = [target for target in second if target.direction.startswith("random_")]
        self.assertEqual(len(first_random), 8)
        self.assertTrue(all(target.target_shear_n <= 1.0 for target in first_random))
        self.assertNotEqual(
            [(target.target_fx, target.target_fy) for target in first_random],
            [(target.target_fx, target.target_fy) for target in second_random],
        )

    def test_training_random_targets_can_be_replayed_with_same_seed(self):
        first = generate_training_trajectory([1.0, 2.0], 1.0, "random_perturb", random_points=8, rng=random.Random(123))
        second = generate_training_trajectory([1.0, 2.0], 1.0, "random_perturb", random_points=8, rng=random.Random(123))
        self.assertEqual(
            [(target.target_fx, target.target_fy, target.target_fz) for target in first],
            [(target.target_fx, target.target_fy, target.target_fz) for target in second],
        )

    def test_training_circular_shear_is_removed(self):
        with self.assertRaises(ValueError):
            generate_training_trajectory([1.0], 1.0, "circular_shear")

    def test_training_target_reached_uses_sensor_frame_force(self):
        target = next(target for target in generate_training_trajectory([0.5], 0.3, "fx_roundtrip", target_step_n=0.3) if target.phase == "target")
        sensor_force = ForceSample("t", 1.0, fx=0.29, fy=0.01, fz=0.51, mx=0.0, my=0.0, mz=0.0)
        self.assertTrue(training_target_reached(sensor_force, target, 0.05))

    def test_training_target_timeout(self):
        self.assertFalse(training_target_timed_out(59.9, 60.0))
        self.assertTrue(training_target_timed_out(60.0, 60.0))

    def test_training_ramp_pauses_when_measured_force_lags(self):
        paused = advance_ramped_force_target((1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.2, 1.0)
        self.assertEqual(paused, (1.0, 0.0, 0.0))
        moved = advance_ramped_force_target((1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.9, 0.0, 0.0), 0.2, 1.0)
        self.assertEqual(moved, (1.2, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
