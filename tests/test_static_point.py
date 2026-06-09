import unittest

from app.models import CapSample
from app.static_point import StaticPointCollector, collection_tolerances


def cap(seq: int, profile: str = "STATIC_PRECISION") -> CapSample:
    return CapSample(
        timestamp=f"t{seq}",
        monotonic_s=float(seq),
        c0=1.0,
        c1=2.0,
        c2=3.0,
        c3=4.0,
        c4=5.0,
        sequence=seq,
        cap_profile=profile,
    )


class StaticPointCollectorTests(unittest.TestCase):
    def test_requires_stable_hold_and_45_unique_samples(self):
        collector = StaticPointCollector()
        collector.begin(1.0)
        collector.update_force_state(5.9, in_window=True, stable=True)
        self.assertFalse(collector.add_cap_sample(cap(1)))
        collector.update_force_state(11.0, in_window=True, stable=True)
        for sequence in range(45):
            self.assertTrue(collector.add_cap_sample(cap(sequence)))
        self.assertTrue(collector.complete)
        self.assertEqual(len(collector.selected_cap_samples()), 45)

    def test_brief_excursion_pauses_and_retains_samples(self):
        collector = StaticPointCollector(stable_hold_s=0.0)
        collector.begin(1.0)
        collector.update_force_state(1.0, in_window=True, stable=True)
        collector.add_cap_sample(cap(1))
        event = collector.update_force_state(
            2.0,
            in_window=False,
            stable=False,
            collection_in_window=False,
            collection_stable=False,
        )
        self.assertEqual(event, "collection_paused")
        self.assertEqual(len(collector.cap_samples), 1)
        self.assertFalse(collector.add_cap_sample(cap(2)))
        event = collector.update_force_state(
            4.0,
            in_window=False,
            stable=False,
            collection_in_window=True,
            collection_stable=True,
        )
        self.assertEqual(event, "collection_resumed")
        self.assertEqual(len(collector.cap_samples), 1)

    def test_prolonged_excursion_clears_samples(self):
        collector = StaticPointCollector(stable_hold_s=0.0, out_of_window_grace_s=5.0)
        collector.begin(1.0)
        collector.update_force_state(1.0, in_window=True, stable=True)
        collector.add_cap_sample(cap(1))
        collector.update_force_state(2.0, in_window=False, stable=False, collection_in_window=False, collection_stable=False)
        event = collector.update_force_state(7.1, in_window=False, stable=False, collection_in_window=False, collection_stable=False)
        self.assertEqual(event, "collection_reset")
        self.assertEqual(len(collector.cap_samples), 0)
        self.assertTrue(collector.timed_out(121.0))

    def test_brief_excursion_after_44_samples_can_finish_without_restart(self):
        collector = StaticPointCollector(stable_hold_s=0.0, out_of_window_grace_s=5.0)
        collector.begin(1.0)
        collector.update_force_state(1.0, in_window=True, stable=True)
        for sequence in range(44):
            self.assertTrue(collector.add_cap_sample(cap(sequence)))
        collector.update_force_state(2.0, in_window=False, stable=False, collection_in_window=False, collection_stable=False)
        self.assertEqual(len(collector.cap_samples), 44)
        event = collector.update_force_state(8.0, in_window=False, stable=False, collection_in_window=False, collection_stable=False)
        self.assertEqual(event, "collection_preserved")
        self.assertEqual(len(collector.cap_samples), 44)
        collector.update_force_state(9.0, in_window=False, stable=False, collection_in_window=True, collection_stable=True)
        self.assertTrue(collector.add_cap_sample(cap(44)))
        self.assertTrue(collector.complete)

    def test_rejects_wrong_profile_and_duplicate_sequence(self):
        collector = StaticPointCollector(stable_hold_s=0.0)
        collector.begin(1.0)
        collector.update_force_state(1.0, in_window=True, stable=True)
        self.assertFalse(collector.add_cap_sample(cap(1, "TRAINING_FAST")))
        self.assertTrue(collector.add_cap_sample(cap(1)))
        self.assertFalse(collector.add_cap_sample(cap(1)))

    def test_rejects_sample_without_sequence(self):
        collector = StaticPointCollector(stable_hold_s=0.0)
        collector.begin(1.0)
        collector.update_force_state(1.0, in_window=True, stable=True)
        sample = cap(1)
        sample.sequence = None
        self.assertFalse(collector.add_cap_sample(sample))

    def test_timeout_allows_two_retries_then_fails(self):
        collector = StaticPointCollector(timeout_s=10.0, max_retries=2)
        collector.begin(1.0)
        self.assertTrue(collector.timed_out(11.0))
        self.assertTrue(collector.retry(11.0))
        self.assertTrue(collector.retry(21.0))
        self.assertFalse(collector.retry(31.0))

    def test_collection_tolerances_are_scaled_by_full_range(self):
        fx, fy, fz = collection_tolerances()
        self.assertAlmostEqual(fx, 0.06)
        self.assertAlmostEqual(fy, 0.06)
        self.assertAlmostEqual(fz, 0.10)
        self.assertEqual(StaticPointCollector().preserve_threshold, 36)


if __name__ == "__main__":
    unittest.main()
