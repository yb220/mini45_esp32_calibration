import unittest

from app.arduino_motion import (
    AUTO_MIN_PULSES,
    DEFAULT_FORCE_TO_MOTOR,
    DEFAULT_FORCE_TO_MOTOR_SIGN,
    MM_PER_PULSE,
    adaptive_force_step_mm,
    mapped_motor_delta,
    mm_to_pulses,
    parse_axis_position,
    parse_motion_line,
    quantize_mm_to_pulses,
)


class MotionProtocolTests(unittest.TestCase):
    def test_parse_ok(self):
        item = parse_motion_line("OK MODE PC")
        self.assertIsNotNone(item)
        self.assertEqual(item.kind, "OK")
        self.assertEqual(item.level, "info")
        self.assertEqual(item.message, "MODE PC")

    def test_parse_position(self):
        item = parse_motion_line("POS X=200 Y=0 Z=-10 XMM=1.0000 YMM=0.0000 ZMM=-0.0500")
        self.assertIsNotNone(item)
        self.assertEqual(item.kind, "POS")
        self.assertEqual(item.values["X"], "200")
        self.assertAlmostEqual(parse_axis_position(item.values, "X"), 1.0)
        self.assertAlmostEqual(parse_axis_position(item.values, "Z"), -0.05)

    def test_default_force_to_motor_mapping(self):
        motor_axis, delta = mapped_motor_delta("Fz", 1.0, 0.005, DEFAULT_FORCE_TO_MOTOR, DEFAULT_FORCE_TO_MOTOR_SIGN)
        self.assertEqual(motor_axis, "X")
        self.assertAlmostEqual(delta, 0.005)

        motor_axis, delta = mapped_motor_delta("Fx", -1.0, 0.005, DEFAULT_FORCE_TO_MOTOR, DEFAULT_FORCE_TO_MOTOR_SIGN)
        self.assertEqual(motor_axis, "Z")
        self.assertAlmostEqual(delta, -0.005)

        motor_axis, delta = mapped_motor_delta("Fy", 1.0, 0.005, DEFAULT_FORCE_TO_MOTOR, DEFAULT_FORCE_TO_MOTOR_SIGN)
        self.assertEqual(motor_axis, "Y")
        self.assertAlmostEqual(delta, 0.005)

    def test_sign_can_be_reversed(self):
        signs = dict(DEFAULT_FORCE_TO_MOTOR_SIGN)
        signs["Fz"] = -1
        motor_axis, delta = mapped_motor_delta("Fz", 1.0, 0.005, DEFAULT_FORCE_TO_MOTOR, signs)
        self.assertEqual(motor_axis, "X")
        self.assertAlmostEqual(delta, -0.005)

    def test_quantize_mm_to_motor_pulses(self):
        self.assertAlmostEqual(MM_PER_PULSE, 0.005)
        self.assertEqual(mm_to_pulses(0.10), 20)
        self.assertAlmostEqual(quantize_mm_to_pulses(0.001), 0.005)
        self.assertAlmostEqual(quantize_mm_to_pulses(-0.001, min_pulses=4), -0.020)

    def test_adaptive_force_step(self):
        self.assertEqual(adaptive_force_step_mm(0.05, tolerance=0.10), 0.0)
        self.assertAlmostEqual(adaptive_force_step_mm(0.20, tolerance=0.10), AUTO_MIN_PULSES * MM_PER_PULSE)
        self.assertAlmostEqual(adaptive_force_step_mm(5.0, tolerance=0.10, max_step_mm=0.10), 0.10)


if __name__ == "__main__":
    unittest.main()
