import json

from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.machines import Fault, Station
from test_machine_registry import RegistryFixture


class MachineTelemetryTests(RegistryFixture):
    def model(self, fault=Fault.NONE):
        return MachineTelemetry(MachineRegistry.load(self.root), fault)

    def test_preserves_causal_observations_with_absolute_source_time(self):
        model = self.model(Fault.LATCHED)
        reference = Station(Fault.LATCHED)
        first = model.sample(50*10**9)
        self.assertEqual(first['stamp_ns'], 50*10**9)
        self.assertEqual(first['time_s'], 0.)
        later = model.sample(51*10**9+250000000)
        reference.tick(1.)
        reference.tick(.25)
        for field, value in reference.observe().items():
            self.assertEqual(later[field], value)
        self.assertEqual(later['sequence'], 2)
        self.assertEqual(first['run_id'], later['run_id'])
        self.assertEqual(later['units']['current_a'], 'A')

    def test_scenario_never_changes_registry_or_exposes_hidden_model_fields(self):
        registries = []
        for fault in Fault:
            value = self.model(fault).sample(10**9)
            registries.append(value['registry'])
            self.assertEqual(set(value), {'schema_version','source','registry','run_id','sequence','stamp_ns','units','time_s','voltage_v','machines','control'})
            self.assertEqual(value['schema_version'], 2)
            self.assertEqual(value['control'], {'revision': 0, 'last_applied': None})
            for machine in value['machines'].values():
                self.assertEqual(set(machine), {'powered','temperature_c','current_a','vibration_au'})
            text = json.dumps(value)
            for forbidden in ('fault','scenario','latched_controller','mechanical_jam','reset_count','recovered'):
                self.assertNotIn(forbidden, text)
        self.assertEqual(registries[0], registries[1])
        self.assertEqual(registries[1], registries[2])

    def test_zero_and_repeated_clock_do_not_manufacture_fresh_samples(self):
        model = self.model()
        self.assertIsNone(model.sample(0))
        value = model.sample(10**9)
        self.assertIsNone(model.sample(10**9))
        later = model.sample(2*10**9)
        self.assertEqual((value['sequence'], later['sequence']), (1,2))
        self.assertEqual(later['time_s'], 2.)

    def test_clock_reset_rotates_run_and_preserves_explicit_model_reset_semantics(self):
        model = self.model(Fault.LATCHED)
        before = model.sample(5*10**9)
        model.station.intervene('a','isolate','pump',distance_m=.5,speed_mps=0.,observed_at_s=0.)
        self.assertIsNone(model.sample(0))
        after = model.sample(10**9)
        self.assertNotEqual(before['run_id'], after['run_id'])
        self.assertEqual(after['sequence'], 1)
        self.assertTrue(after['machines']['pump']['powered'])
        self.assertLess(after['voltage_v'], 27)

    def test_forward_gap_boundary_is_explicit_and_rotates_only_above_ten_seconds(self):
        model = self.model()
        first = model.sample(10**9)
        boundary = model.sample(11*10**9)
        self.assertEqual(first['run_id'], boundary['run_id'])
        self.assertEqual(boundary['time_s'], 10.)
        after = model.sample(21*10**9+1)
        self.assertNotEqual(after['run_id'], boundary['run_id'])
        self.assertEqual(after['time_s'], 0.)
        self.assertEqual(after['sequence'], 1)

    def test_invalid_clock_and_changed_source_do_not_advance_model(self):
        model = self.model()
        before = model.sample(10**9)
        for stamp in (-1, True, 1., 2**63):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                model.sample(stamp)
        self.world_path.write_bytes(self.world_path.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError,'source changed'):
            model.sample(2*10**9)
        self.assertEqual((model.previous,model.sequence,model.station.time_s), (10**9,1,0.))
        self.assertEqual(model.run_id,before['run_id'])

    def test_station_identity_drift_cannot_relabel_a_machine(self):
        for kind in ('missing','alias'):
            model = self.model()
            if kind == 'missing':
                del model.station.machines['pump']
            else:
                model.station.machines['pump'].name = 'cooler'
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError,'identities'):
                model.sample(10**9)

    def test_caller_cannot_mutate_subsequent_observations_or_units(self):
        model = self.model()
        first = model.sample(10**9)
        first['units']['current_a'] = 'wrong'
        first['machines']['pump']['powered'] = False
        first['registry']['machines']['pump']['control_xy_m'][0] = 0
        later = model.sample(2*10**9)
        self.assertEqual(later['units']['current_a'], 'A')
        self.assertTrue(later['machines']['pump']['powered'])
        self.assertEqual(later['registry']['machines']['pump']['control_xy_m'], [2.6,1.8])
