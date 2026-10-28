import copy
import json
import uuid

from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.machines import Fault
from orbital.station_control import EffectRejected
from orbital.telemetry_observation import decode
from test_machine_registry import RegistryFixture


class StationControlTests(RegistryFixture):
    def setUp(self):
        super().setUp()
        self.registry = MachineRegistry.load(self.root)
        self.model = MachineTelemetry(self.registry, Fault.LATCHED)
        self.first = self.model.sample(1_000_000_000)
        self.context = {'owner_run_id': uuid.uuid4().hex, 'mission_id': uuid.uuid4().hex,
                        'action_id': uuid.uuid4().hex, 'request_sha256': 'a'*64, 'machine_id': 'pump'}

    def isolate(self, **kwargs):
        values = dict(context=self.context, expected_run=self.first['run_id'], expected_revision=0,
                      admission=lambda:{'stamp_ns': 1_200_000_000, 'distance_m': .6, 'speed_mps': .001})
        values.update(kwargs)
        return self.model.isolate(**values)

    def test_effect_changes_the_published_station_and_is_bound_to_new_samples(self):
        effect = self.isolate()
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertTrue(self.model.station.machines['cooler'].powered)
        self.assertTrue(self.model.station.machines['power_unit'].powered)
        self.assertIsNone(self.model.sample(1_200_000_000))
        after = decode(json.dumps(self.model.sample(1_500_000_000)), self.registry)
        self.assertFalse(after['machines']['pump']['powered'])
        self.assertEqual(after['machines']['pump']['current_a'], 0.)
        self.assertEqual(after['control'], {'revision': 1, 'last_applied': effect})
        self.assertGreater(after['stamp_ns'], effect['applied_stamp_ns'])
        self.assertEqual(after['run_id'], self.first['run_id'])
        self.assertEqual(after['sequence'], self.first['sequence']+1)
        self.assertAlmostEqual(after['time_s'], .5)

    def test_exact_replay_never_calls_admission_or_actuator_again(self):
        effect = self.isolate()
        self.model.station.tick(.1)
        before = self.model.station.time_s
        def forbidden(): raise AssertionError('replay repeated an effect')
        self.assertEqual(effect, self.isolate(admission=forbidden))
        self.assertEqual(self.model.station.time_s, before)
        self.assertEqual(self.model.control_revision, 1)
        self.assertEqual(len(self.model.station._completed), 1)
        with self.assertRaises(EffectRejected):
            self.isolate(context={**self.context, 'machine_id': 'cooler'})
        with self.assertRaises(EffectRejected):
            self.isolate(expected_revision=1)

    def test_invalid_late_admission_never_changes_power(self):
        def rejected(): raise ValueError('controlled late pose loss')
        with self.assertRaisesRegex(EffectRejected, 'late pose loss'):
            self.isolate(admission=rejected)
        for values in ({'distance_m': 1.}, {'speed_mps': .02}, {'speed_mps': True}, {'stamp_ns': 0}):
            with self.subTest(values=values), self.assertRaises(EffectRejected):
                self.isolate(admission=lambda:{'stamp_ns': 1_200_000_000, 'distance_m': .6, 'speed_mps': 0., **values})
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertEqual(self.model.control_revision, 0)
        self.assertEqual(self.model.effects, {})

    def test_clock_reset_during_admission_invalidates_the_old_run(self):
        with self.assertRaisesRegex(EffectRejected, 'station_run_changed'):
            self.isolate(admission=lambda:{'stamp_ns': 500_000_000, 'distance_m': .6, 'speed_mps': 0.})
        self.assertNotEqual(self.model.run_id, self.first['run_id'])
        self.assertTrue(self.model.station.machines['pump'].powered)
        with self.assertRaises(EffectRejected):
            self.isolate()

    def test_already_isolated_or_changed_revision_refuses_new_effect(self):
        self.isolate()
        context = {**self.context, 'action_id': uuid.uuid4().hex}
        with self.assertRaises(EffectRejected): self.isolate(context=context)
        with self.assertRaisesRegex(EffectRejected, 'already_isolated'):
            self.isolate(context=context, expected_revision=1)
        self.assertEqual(self.model.control_revision, 1)

    def test_returned_effect_and_public_packet_cannot_mutate_owner_history(self):
        effect = self.isolate()
        expected = copy.deepcopy(effect)
        effect['machine_id'] = 'cooler'
        packet = self.model.sample(1_500_000_000)
        packet['control']['last_applied']['machine_id'] = 'cooler'
        self.assertEqual(self.isolate(), expected)
        self.assertEqual(self.model.sample(2_000_000_000)['control']['last_applied'], expected)

    def test_schema_one_remains_read_only_and_schema_two_control_is_strict(self):
        legacy = copy.deepcopy(self.first)
        legacy.pop('control')
        legacy['schema_version'] = 1
        self.assertEqual(decode(json.dumps(legacy), self.registry), legacy)
        self.isolate()
        packet = self.model.sample(1_500_000_000)
        variants = []
        for key, value in (('revision', True), ('revision', 2), ('last_applied', None)):
            bad = copy.deepcopy(packet); bad['control'][key] = value; variants.append(bad)
        for key, value in (('applied_stamp_ns', 2_000_000_000), ('operation', 'repair'), ('machine_id', 'unknown')):
            bad = copy.deepcopy(packet); bad['control']['last_applied'][key] = value; variants.append(bad)
        bad = copy.deepcopy(packet); bad['control']['last_applied']['fault'] = 'none'; variants.append(bad)
        for bad in variants:
            with self.subTest(control=bad['control']), self.assertRaises(ValueError):
                decode(json.dumps(bad), self.registry)
