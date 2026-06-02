import unittest

from app.force_control import (
    DecoupledControlSettings,
    DecoupledControlState,
    auto_damping_eta,
    compute_decoupled_command,
    identify_k_matrix,
)
from app.models import SafetySettings


class ForceControlTests(unittest.TestCase):
    def test_identify_k_matrix_uses_force_rows_and_motor_columns(self):
        before = {axis: [0.0, 0.0, 0.0] for axis in ("X", "Y", "Z")}
        after = {
            "X": [0.1, 0.2, 1.0],
            "Y": [0.0, 0.5, 0.1],
            "Z": [0.8, 0.1, 0.0],
        }
        std = {axis: [0.0, 0.0, 0.0] for axis in ("X", "Y", "Z")}
        result = identify_k_matrix(before, after, std, std, {"X": 0.1, "Y": 0.1, "Z": 0.1})
        self.assertTrue(result.valid)
        self.assertEqual([row[0] for row in result.k], [1.0, 2.0, 10.0])
        self.assertEqual([row[1] for row in result.k], [0.0, 5.0, 1.0])
        self.assertEqual([row[2] for row in result.k], [8.0, 1.0, 0.0])

    def test_low_force_delta_is_invalid(self):
        before = {axis: [0.0, 0.0, 0.0] for axis in ("X", "Y", "Z")}
        after = {axis: [0.001, 0.0, 0.0] for axis in ("X", "Y", "Z")}
        std = {axis: [0.0, 0.0, 0.0] for axis in ("X", "Y", "Z")}
        result = identify_k_matrix(before, after, std, std, {"X": 0.1, "Y": 0.1, "Z": 0.1})
        self.assertFalse(result.valid)
        self.assertIn("力变化过小", result.reject_reason)

    def test_auto_damping_increases_for_ill_conditioned_matrix(self):
        good = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        bad = [[10.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.001]]
        self.assertGreater(auto_damping_eta(bad), auto_damping_eta(good))

    def test_decoupled_command_outputs_motor_axis_deltas(self):
        k = [
            [0.0, 0.0, 8.0],
            [0.0, 5.0, 0.0],
            [10.0, 0.0, 0.0],
        ]
        state = DecoupledControlState()
        command = compute_decoupled_command(
            k=k,
            target_force=[0.8, 0.5, 1.0],
            current_force=[0.0, 0.0, 0.0],
            state=state,
            settings=DecoupledControlSettings(max_step_mm=0.1, style="standard"),
            safety=SafetySettings(),
        )
        self.assertGreater(command.delta_mm["X"], 0.0)
        self.assertGreater(command.delta_mm["Y"], 0.0)
        self.assertGreater(command.delta_mm["Z"], 0.0)
        self.assertAlmostEqual(command.delta_mm["X"], round(command.delta_mm["X"] / 0.005) * 0.005, places=9)

    def test_trust_scale_shrinks_when_error_increases(self):
        k = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        state = DecoupledControlState(previous_error=[0.1, 0.0, 0.0], trust_scale=1.0)
        command = compute_decoupled_command(
            k=k,
            target_force=[1.0, 0.0, 0.0],
            current_force=[0.0, 0.0, 0.0],
            state=state,
            settings=DecoupledControlSettings(max_step_mm=0.1, style="standard"),
            safety=SafetySettings(),
        )
        self.assertLess(command.trust_scale, 1.0)

    def test_trust_scale_grows_when_progress_is_too_slow(self):
        k = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        state = DecoupledControlState(previous_error=[1.0, 0.0, 0.0], trust_scale=0.5)
        command = compute_decoupled_command(
            k=k,
            target_force=[0.98, 0.0, 0.0],
            current_force=[0.0, 0.0, 0.0],
            state=state,
            settings=DecoupledControlSettings(max_step_mm=1.0, style="standard"),
            safety=SafetySettings(),
        )
        self.assertGreater(command.trust_scale, 0.5)

    def test_min_effective_step_avoids_tiny_far_from_target_moves(self):
        k = [[50.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
        state = DecoupledControlState()
        command = compute_decoupled_command(
            k=k,
            target_force=[1.0, 0.0, 0.0],
            current_force=[0.0, 0.0, 0.0],
            state=state,
            settings=DecoupledControlSettings(
                max_step_mm=0.1,
                style="standard",
                min_effective_step_mm=0.02,
                coarse_error_n=0.2,
            ),
            safety=SafetySettings(),
        )
        self.assertGreaterEqual(abs(command.delta_mm["X"]), 0.02)


if __name__ == "__main__":
    unittest.main()
