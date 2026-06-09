import struct
import unittest

from app.esp32_serial import Esp32Log, Esp32ProfileStatus, parse_cap_line
from app.mini45_netft import build_rdt_command, parse_rdt_packet
from app.models import CapSample


class ParserTests(unittest.TestCase):
    def test_parse_legacy_data0(self):
        item = parse_cap_line("DATA0,1,2,3,4,5", monotonic_s=10.0)
        self.assertIsInstance(item, CapSample)
        self.assertEqual(item.c0, 1.0)
        self.assertEqual(item.c4, 5.0)
        self.assertIsNone(item.sequence)

    def test_parse_stream_cap(self):
        item = parse_cap_line("CAP,1234,7,1.1,2.2,3.3,4.4,5.5", monotonic_s=10.0)
        self.assertIsInstance(item, CapSample)
        self.assertEqual(item.esp_ms, 1234)
        self.assertEqual(item.sequence, 7)
        self.assertAlmostEqual(item.c2, 3.3)

    def test_parse_profiled_stream_cap(self):
        item = parse_cap_line(
            "CAP,1234,7,1.1,2.2,3.3,4.4,5.5,STATIC_PRECISION,255,32,2.262325",
            monotonic_s=10.0,
        )
        self.assertIsInstance(item, CapSample)
        self.assertEqual(item.cap_profile, "STATIC_PRECISION")
        self.assertEqual(item.mc1081_cnt, 255)
        self.assertEqual(item.mc1081_cavg, 32)

    def test_parse_profile_status(self):
        item = parse_cap_line("L:PROFILE,TRAINING_BALANCED,cnt=191,cavg=8,nominal_hz=11.363636")
        self.assertIsInstance(item, Esp32ProfileStatus)
        self.assertEqual(item.name, "TRAINING_BALANCED")
        self.assertEqual(item.cnt, 191)

    def test_parse_log_and_error(self):
        self.assertIsInstance(parse_cap_line("L:ok"), Esp32Log)
        self.assertIsInstance(parse_cap_line("E:MEASURE,fail"), Esp32Log)

    def test_build_rdt_command(self):
        self.assertEqual(build_rdt_command(0x0002, 0), struct.pack("!HHI", 0x1234, 0x0002, 0))

    def test_parse_rdt_packet_36_bytes(self):
        packet = struct.pack("!IIIiiiiii", 1, 2, 0, 1000000, -2000000, 3000000, 1000, -2000, 3000)
        samples = parse_rdt_packet(packet, monotonic_s=1.0, torque_counts_per_unit=1000.0)
        self.assertEqual(len(samples), 1)
        sample = samples[0]
        self.assertEqual(sample.sequence, 1)
        self.assertAlmostEqual(sample.fx, 1.0)
        self.assertAlmostEqual(sample.fy, -2.0)
        self.assertAlmostEqual(sample.fz, 3.0)
        self.assertAlmostEqual(sample.mx, 1.0)
        self.assertAlmostEqual(sample.my, -2.0)
        self.assertAlmostEqual(sample.mz, 3.0)


if __name__ == "__main__":
    unittest.main()
