import hashlib
import json
from pathlib import Path
import unittest

from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.machines import Fault, Station
from orbital.maintenance_observation import RecoveryEvidence
from orbital.proximity import Unavailable
from orbital.telemetry_observation import canonical

ROOT = Path(__file__).resolve().parents[1]


class MaintenanceObservationTests(unittest.TestCase):
    def setUp(self):
        self.registry = MachineRegistry.load(ROOT)
        self.model = MachineTelemetry(self.registry)
        self.evidence = RecoveryEvidence(self.registry)
        self.stamp = 500_000_000
        self.wall = .5
        self.packets = []

    def sample(self, *, deliver=True, wall_step=.5):
        self.stamp += 500_000_000
        self.wall += wall_step
        packet = self.model.sample(self.stamp)
        if deliver:
            self.packets.append(packet)
            self.assertTrue(self.evidence.observe(json.dumps(packet), self.wall, self.stamp))
        return packet

    def measure(self):
        return self.evidence.measure(self.wall, self.stamp)

    def complete_window(self):
        for _ in range(22):
            self.sample()
        return self.measure()

    def test_requires_ten_seconds_after_fresh_pair_and_binds_all_samples(self):
        for _ in range(21):
            self.sample()
        before = self.measure()
        self.assertFalse(before['verified'])
        self.assertEqual(before['window']['duration_s'], 9.5)
        self.sample()
        after = self.measure()
        self.assertTrue(after['verified'])
        self.assertEqual(after['window']['duration_s'], 10.)
        self.assertEqual(after['window']['samples'], 21)
        expected = hashlib.sha256(b''.join(canonical(p).encode()+b'\n' for p in self.packets[1:]))
        self.assertEqual(after['window']['samples_sha256'], expected.hexdigest())
        after['window']['first']['stamp_ns'] = 0
        self.assertNotEqual(self.measure()['window']['first']['stamp_ns'], 0)

    def test_polls_and_advancing_clock_do_not_invent_new_measurements(self):
        for _ in range(21):
            self.sample()
        before = self.measure()
        after = self.evidence.measure(self.wall+.75, self.stamp+750_000_000)
        self.assertFalse(after['verified'])
        self.assertEqual(before['window'], after['window'])

    def test_any_machine_power_loss_discards_prior_healthy_window(self):
        self.assertTrue(self.complete_window()['verified'])
        self.model.station.machines['cooler'].powered = False
        self.sample()
        result = self.measure()
        self.assertFalse(result['verified'])
        self.assertEqual(result['reason'], 'unhealthy_measurement')
        self.assertIsNone(result['window'])
        self.model.station.machines['cooler'].powered = True
        for _ in range(20):
            self.sample()
        self.assertFalse(self.measure()['verified'])
        self.sample()
        self.assertTrue(self.measure()['verified'])

    def test_isolation_improves_voltage_but_never_verifies_recovery(self):
        self.model = MachineTelemetry(self.registry, Fault.LATCHED)
        self.sample()
        s = self.model.station
        s.intervene('off', 'isolate', 'pump', distance_m=.5, speed_mps=0, observed_at_s=s.time_s)
        result = self.complete_window()
        self.assertGreaterEqual(self.packets[-1]['voltage_v'], 27)
        self.assertFalse(result['verified'])
        self.assertIsNone(result['window'])

    def test_successful_jam_reset_and_restore_do_not_verify_recovery(self):
        self.model = MachineTelemetry(self.registry, Fault.JAM)
        self.sample()
        s = self.model.station
        for op in ('isolate', 'reset', 'restore'):
            receipt = s.intervene(op, op, 'pump', distance_m=.5, speed_mps=0, observed_at_s=s.time_s)
            self.assertEqual(receipt['status'], 'succeeded')
        self.assertFalse(self.complete_window()['verified'])

    def test_missing_sequence_requires_a_new_complete_window(self):
        self.assertTrue(self.complete_window()['verified'])
        self.sample(deliver=False)
        self.sample()
        with self.assertRaisesRegex(Unavailable, 'awaiting_fresh_telemetry_pair'):
            self.measure()
        for _ in range(20):
            self.sample()
        self.assertFalse(self.measure()['verified'])
        self.sample()
        result = self.measure()
        self.assertTrue(result['verified'])
        self.assertEqual(result['freshness']['missing_sequences'], 1)

    def test_receipt_expiry_cannot_bridge_healthy_windows(self):
        self.assertTrue(self.complete_window()['verified'])
        with self.assertRaises(Unavailable):
            self.evidence.measure(self.wall+1., self.stamp)
        self.wall += 1.
        self.sample()
        with self.assertRaises(Unavailable):
            self.measure()
        for _ in range(20):
            self.sample()
        self.assertFalse(self.measure()['verified'])
        self.sample()
        self.assertTrue(self.measure()['verified'])

    def test_simulation_expiry_without_wall_expiry_also_resets_window(self):
        self.assertTrue(self.complete_window()['verified'])
        with self.assertRaises(Unavailable):
            self.evidence.measure(self.wall, self.stamp+1_000_000_000)
        self.sample(deliver=False)
        self.sample()
        with self.assertRaises(Unavailable):
            self.measure()
        self.sample()
        self.assertEqual(self.measure()['window']['duration_s'], 0.)

    def test_source_run_change_is_sticky_and_cannot_reuse_old_window(self):
        self.assertTrue(self.complete_window()['verified'])
        self.model.reset()
        self.stamp += 500_000_000
        self.wall += .5
        packet = self.model.sample(self.stamp)
        self.assertFalse(self.evidence.observe(json.dumps(packet), self.wall, self.stamp))
        with self.assertRaisesRegex(Unavailable, 'telemetry_run_changed'):
            self.measure()
        self.assertFalse(self.evidence.observe(json.dumps(packet), self.wall, self.stamp))

    def test_invalid_or_replayed_public_sample_revokes_verified_result(self):
        for mode in ('extra_hidden_field', 'replay'):
            with self.subTest(mode=mode):
                self.setUp()
                self.assertTrue(self.complete_window()['verified'])
                packet = json.loads(json.dumps(self.packets[-1]))
                if mode == 'extra_hidden_field':
                    packet['fault'] = 'none'
                self.assertFalse(self.evidence.observe(json.dumps(packet), self.wall, self.stamp))
                with self.assertRaises(Unavailable):
                    self.measure()

    def test_isolated_measurements_cannot_confirm_reset_application(self):
        stations = [Station(Fault.LATCHED), Station(Fault.LATCHED), Station(Fault.JAM)]
        for station in stations:
            station.intervene('off', 'isolate', 'pump', distance_m=.5, speed_mps=0, observed_at_s=0)
        for station in stations[1:]:
            station.intervene('reset', 'reset', 'pump', distance_m=.5, speed_mps=0, observed_at_s=0)
        # These internal assertions are evaluator-only controls. The observations
        # remain identical despite no reset, a cleared latch and a persistent jam.
        self.assertEqual([s.machines['pump'].reset_count for s in stations], [0, 1, 1])
        self.assertEqual([s.machines['pump'].fault for s in stations], [Fault.LATCHED, Fault.NONE, Fault.JAM])
        for _ in range(6):
            self.assertEqual(stations[0].observe(), stations[1].observe())
            self.assertEqual(stations[1].observe(), stations[2].observe())
            for station in stations:
                station.tick(.5)


if __name__ == '__main__':
    unittest.main()
