import copy
import json
from unittest.mock import patch

from orbital.inspection_protocol import canonical
from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.machines import Fault
from orbital.planner_observation import PlannerObservation
from orbital.planner_protocol import PlannerError
from planner_fixture import RUN, MISSION, call
from test_machine_registry import RegistryFixture


class PlannerObservationTests(RegistryFixture):
    def setUp(self):
        super().setUp()
        self.registry = MachineRegistry.load(self.root)
        self.wall, self.sim = 10., 10**9
        self.model = MachineTelemetry(self.registry)
        self.boundary = PlannerObservation(self.registry, run_id=RUN, mission_id=MISSION,
            now=lambda: self.wall, sim_ns=lambda: self.sim)

    def pair(self):
        for wall, sim in ((10., 10**9), (10.5, 1500000000)):
            self.wall, self.sim = wall, sim
            self.assertTrue(self.boundary.observe(canonical(self.model.sample(sim))))
        return self.boundary.view()

    def bound_call(self, observation, tool='inspect_machine'):
        value = call(tool); value['observation_id'] = observation['observation_id']
        return canonical(value)

    def test_actual_public_model_measurements_and_geometry_survive_projection(self):
        view = self.pair()
        public = self.model.station.observe()
        for key in ('time_s', 'voltage_v', 'machines'):
            self.assertEqual(view['station'][key], public[key])
        self.assertEqual(view['machines']['pump']['inspection_xy_yaw'], [2., 1.8, 0.])
        self.assertEqual(view['registry_identity'], self.registry.identity)
        self.assertEqual(view['station']['run_id'], self.model.run_id)
        self.assertEqual(view['run_id'], RUN)
        self.assertNotEqual(view['station']['run_id'], view['run_id'])
        self.assertEqual(self.boundary.validate(self.bound_call(view))['tool'], 'inspect_machine')

    def test_private_model_state_and_repository_paths_never_enter_public_views(self):
        for fault in Fault:
            self.setUp()
            self.model = MachineTelemetry(self.registry, fault)
            text = canonical(self.pair())
            for secret in ('fault', 'scenario', 'seed', 'reset_count', 'recovered', 'configuration',
                           'mechanical_jam', 'latched_controller', str(self.root), 'worlds/', 'source_sha256'):
                with self.subTest(fault=fault, secret=secret): self.assertNotIn(secret, text)

    def test_unexposed_forged_and_previous_sample_ids_do_not_pass(self):
        with self.assertRaisesRegex(PlannerError, '^stale_observation$'):
            self.boundary.validate(canonical(call()))
        view = self.pair()
        with self.assertRaises(PlannerError): self.boundary.validate(canonical(call()))
        self.wall, self.sim = 11., 2*10**9
        self.assertTrue(self.boundary.observe(canonical(self.model.sample(self.sim))))
        with self.assertRaises(PlannerError): self.boundary.validate(self.bound_call(view))
        current = self.boundary.view()
        self.assertNotEqual(current['observation_id'], view['observation_id'])
        self.boundary.validate(self.bound_call(current))

    def test_both_clocks_expire_the_observation_without_renewing_it_by_reading(self):
        for wall, sim in ((11.5, 1500000000), (10.5, 2500000000)):
            self.setUp(); view = self.pair()
            self.wall, self.sim = wall, sim
            with self.subTest(wall=wall, sim=sim), self.assertRaises(PlannerError):
                self.boundary.validate(self.bound_call(view))

    def test_slow_source_verification_cannot_make_expired_observations_current(self):
        view = self.pair()
        original = MachineRegistry.verify_current
        def slow(registry):
            original(registry)
            self.wall, self.sim = 12., 3*10**9
        with patch.object(MachineRegistry, 'verify_current', slow), self.assertRaises(PlannerError):
            self.boundary.validate(self.bound_call(view))

    def test_changed_source_is_sticky_even_if_original_bytes_are_restored(self):
        view = self.pair(); source = self.config_path.read_bytes()
        self.config_path.write_bytes(source+b'\n')
        with self.assertRaisesRegex(PlannerError, '^unavailable$'):
            self.boundary.validate(self.bound_call(view))
        self.config_path.write_bytes(source)
        with self.assertRaisesRegex(PlannerError, '^unavailable$'): self.boundary.view()

    def test_hidden_fields_in_incoming_packets_poison_the_stream(self):
        for location in ('packet', 'machine', 'registry'):
            self.setUp(); self.pair()
            self.wall, self.sim = 11., 2*10**9
            packet = self.model.sample(self.sim)
            target = {'packet': packet, 'machine': packet['machines']['pump'], 'registry': packet['registry']}[location]
            target['private-canary'] = 'hidden'
            self.assertFalse(self.boundary.observe(canonical(packet)))
            with self.assertRaisesRegex(PlannerError, '^stale_observation$'): self.boundary.view()

    def test_actual_causal_isolation_invalidates_the_previous_public_reference(self):
        view = self.pair()
        context = {'owner_run_id': '1'*32, 'mission_id': MISSION, 'action_id': '2'*32,
                   'request_sha256': '3'*64, 'machine_id': 'pump'}
        self.model.isolate(context, self.model.run_id, 0,
            admission=lambda: {'stamp_ns': 1600000000, 'distance_m': .3, 'speed_mps': 0.})
        self.wall, self.sim = 10.7, 1700000000
        self.assertTrue(self.boundary.observe(canonical(self.model.sample(self.sim))))
        with self.assertRaises(PlannerError): self.boundary.validate(self.bound_call(view))
        current = self.boundary.view()
        self.assertFalse(current['station']['machines']['pump']['powered'])
        self.assertEqual(current['station']['control']['revision'], 1)
        self.assertEqual(current['station']['control']['last_applied']['action_id'], context['action_id'])

    def test_returned_view_mutation_cannot_change_retained_source_or_next_view(self):
        view = self.pair(); before = copy.deepcopy(view)
        view['station']['machines']['pump']['current_a'] = -123
        view['machines']['pump']['inspection_xy_yaw'][0] = 999
        view['station']['control']['revision'] = 100
        self.assertEqual(self.boundary.view(), before)

    def test_invalid_run_or_mission_is_rejected_before_public_source_io(self):
        view = self.pair(); value = json.loads(self.bound_call(view)); value['mission_id'] = 'f'*32
        with patch.object(MachineRegistry, 'verify_current', side_effect=AssertionError('unexpected IO')):
            with self.assertRaisesRegex(PlannerError, '^wrong_run$'):
                self.boundary.validate(canonical(value))

    def test_stop_and_outcome_claim_remain_parseable_when_sources_are_unavailable(self):
        self.pair(); self.config_path.write_text('private-canary')
        with self.assertRaises(PlannerError): self.boundary.view()
        for tool in ('stop', 'report_outcome'):
            self.assertEqual(self.boundary.validate(canonical(call(tool)))['tool'], tool)
        self.assertEqual(self.model.control_revision, 0)
        self.assertFalse(self.model.effects)

    def test_validation_is_not_replay_safe_dispatch_and_has_no_effects(self):
        view = self.pair(); text = self.bound_call(view, 'isolate_power')
        first = self.boundary.validate(text)
        self.assertEqual(self.boundary.validate(text), first)
        self.assertEqual(self.model.control_revision, 0)
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertFalse(self.model.effects)

    def test_single_sample_and_replay_cannot_create_a_view(self):
        packet = canonical(self.model.sample(self.sim))
        self.assertTrue(self.boundary.observe(packet))
        with self.assertRaises(PlannerError): self.boundary.view()
        self.assertFalse(self.boundary.observe(packet))
        with self.assertRaises(PlannerError): self.boundary.view()

    def test_clock_regression_and_station_restart_require_a_new_boundary(self):
        for mode in ('wall', 'simulation', 'restart'):
            self.setUp(); self.pair()
            if mode == 'wall': self.wall = 10.4
            elif mode == 'simulation': self.sim = 1400000000
            else:
                self.model = MachineTelemetry(self.registry)
                self.wall, self.sim = 11., 2*10**9
                self.assertFalse(self.boundary.observe(canonical(self.model.sample(self.sim))))
            with self.subTest(mode=mode), self.assertRaises(PlannerError): self.boundary.view()
            self.wall, self.sim = 11.5, 2500000000
            self.assertFalse(self.boundary.observe(canonical(self.model.sample(self.sim))))

    def test_legacy_telemetry_without_control_revision_is_not_a_planner_observation(self):
        for wall, sim in ((10., 10**9), (10.5, 1500000000)):
            self.wall, self.sim = wall, sim
            packet = self.model.sample(sim); packet['schema_version'] = 1; del packet['control']
            self.assertTrue(self.boundary.observe(canonical(packet)))
        with self.assertRaisesRegex(PlannerError, '^unavailable$'): self.boundary.view()
