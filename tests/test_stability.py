import unittest

from app.models import CombinedSnapshot, ExperimentMeta, SafetySettings, StabilitySettings
from app.stability import build_calibration_point, evaluate_stability, evaluate_three_axis_stability


def force_sample(t, fz=1.0, fx=0.0, fy=0.0):
    return CombinedSnapshot(timestamp=f"t{t}", monotonic_s=float(t), source="mini45", fx=fx, fy=fy, fz=fz, mx=0.0, my=0.0, mz=0.0)


def cap_sample(t, c0=10.0):
    return CombinedSnapshot(timestamp=f"t{t}", monotonic_s=float(t), source="esp32", c0=c0, c1=1.0, c2=2.0, c3=3.0, c4=4.0)


class StabilityTests(unittest.TestCase):
    def test_stable_window_valid(self):
        samples = [force_sample(i, fz=1.0 + 0.001 * i) for i in range(10)] + [cap_sample(i, c0=10.0 + 0.001 * i) for i in range(10)]
        meta = ExperimentMeta(axis="Fz", target_fz=1.0)
        settings = StabilitySettings(tolerance_fz=0.05)
        result = evaluate_stability(samples, meta, settings, SafetySettings())
        self.assertTrue(result.in_window)
        self.assertTrue(result.stable)

    def test_capacitance_single_spike_does_not_reject_stable_body(self):
        samples = [force_sample(i, fz=1.0) for i in range(100)]
        samples += [cap_sample(i, c0=10.0) for i in range(99)]
        samples.append(cap_sample(99, c0=10.2))
        result = evaluate_stability(samples, ExperimentMeta(axis="Fz", target_fz=1.0), StabilitySettings(), SafetySettings())
        self.assertTrue(result.stable)

    def test_capacitance_p95p5_rejects_wide_body(self):
        samples = [force_sample(i, fz=1.0) for i in range(100)]
        samples += [cap_sample(i, c0=9.965) for i in range(50)]
        samples += [cap_sample(i + 50, c0=10.035) for i in range(50)]
        result = evaluate_stability(samples, ExperimentMeta(axis="Fz", target_fz=1.0), StabilitySettings(), SafetySettings())
        self.assertFalse(result.stable)
        self.assertIn("c0 capacitance p95-p5 too high", result.reject_reason)

    def test_capacitance_std_rejects_noisy_body(self):
        samples = [force_sample(i, fz=1.0) for i in range(100)]
        samples += [cap_sample(i, c0=9.975) for i in range(50)]
        samples += [cap_sample(i + 50, c0=10.025) for i in range(50)]
        result = evaluate_stability(samples, ExperimentMeta(axis="Fz", target_fz=1.0), StabilitySettings(), SafetySettings())
        self.assertFalse(result.stable)
        self.assertIn("c0 capacitance std too high", result.reject_reason)

    def test_capacitance_rules_match_three_axis_stability(self):
        samples = [force_sample(i, fz=1.0) for i in range(100)]
        samples += [cap_sample(i, c0=9.975) for i in range(50)]
        samples += [cap_sample(i + 50, c0=10.025) for i in range(50)]
        meta = ExperimentMeta(axis="Fz", target_fz=1.0)
        single = evaluate_stability(samples, meta, StabilitySettings(), SafetySettings())
        three_axis = evaluate_three_axis_stability(samples, meta, StabilitySettings(), SafetySettings())
        self.assertFalse(single.stable)
        self.assertFalse(three_axis.stable)
        self.assertIn("c0 capacitance std too high", three_axis.reject_reason)

    def test_target_outside_window_invalid(self):
        samples = [force_sample(i, fz=1.3) for i in range(10)]
        meta = ExperimentMeta(axis="Fz", target_fz=1.0)
        result = evaluate_stability(samples, meta, StabilitySettings(tolerance_fz=0.05), SafetySettings())
        self.assertFalse(result.in_window)
        self.assertFalse(result.stable)

    def test_build_calibration_point(self):
        samples = [force_sample(i, fz=1.0) for i in range(3)] + [cap_sample(i, c0=10.0) for i in range(3)]
        meta = ExperimentMeta(experiment_id="e1", cycle_id="c1", axis="Fz", target_fz=1.0)
        point = build_calibration_point(samples, meta, 1, True, "")
        self.assertIsNotNone(point)
        self.assertEqual(point.experiment_id, "e1")
        self.assertEqual(point.marker_id, 1)
        self.assertAlmostEqual(point.Fz_mean, 1.0)
        self.assertAlmostEqual(point.preload_N, 1.0)
        self.assertAlmostEqual(point.C0_mean, 10.0)
        self.assertAlmostEqual(point.C0_trimmed_mean, 10.0)
        self.assertEqual(point.cap_sample_count, 3)
        self.assertEqual(point.force_sample_count, 3)

    def test_trimmed_mean_rejects_p01_p99_extremes(self):
        samples = [force_sample(i, fz=value) for i, value in enumerate([1.0] * 98 + [-20.0, 20.0])]
        samples += [cap_sample(i, c0=value) for i, value in enumerate([10.0] * 98 + [-50.0, 50.0])]
        point = build_calibration_point(samples, ExperimentMeta(axis="Fz"), 1, True, "")
        self.assertAlmostEqual(point.Fz_trimmed_mean, 1.0)
        self.assertAlmostEqual(point.C0_trimmed_mean, 10.0)


if __name__ == "__main__":
    unittest.main()
