"""Versioned private initial conditions and actual station publication boundaries."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
from unittest.mock import patch

from orbital import scenario_spec as spec, scenario_input as storage
from orbital.inspection_protocol import canonical
from orbital.inspection_worker import OwnerLock
from orbital.machine_registry import MachineRegistry
from orbital.maintenance_journal import Journal, prepare_configured_scenario, prepare_scenario, read_history
from orbital.machine_telemetry import MachineTelemetry
from orbital.scenario_input import PreparedInput
from orbital.telemetry_observation import decode
from test_machine_registry import RegistryFixture, ROOT
from test_machine_telemetry_adapter import AdapterHarness


class ScenarioInputTests(RegistryFixture):
    def prepared(self, profile='recoverable', seed=0):
        assignment = prepare_configured_scenario(self.root, profile, seed)
        lock = OwnerLock(self.root, service='station-service'); self.addCleanup(lock.close)
        journal = Journal(lock); self.addCleanup(journal.close)
        value = PreparedInput(journal, MachineRegistry.load(self.root)); self.addCleanup(value.close)
        return assignment, lock, journal, value

    def test_same_seed_reproduces_conditions_with_unrelated_new_trial_identity(self):
        registry = MachineRegistry.load(self.root)
        for profile in spec.PROFILES:
            first, second = [spec.create(profile, 19, registry.identity) for _ in range(2)]
            self.assertEqual(first['initial_state'], second['initial_state'])
            self.assertEqual(first['navigation'], second['navigation'])
            self.assertNotEqual(first['trial_id'], second['trial_id'])
            self.assertEqual(spec.validate(canonical(first), registry.identity), first)
        varied = [spec.create('recoverable', seed, registry.identity) for seed in (0, 1, 2)]
        targets = [next(n for n, m in v['initial_state'].items() if m['fault'] != 'none') for v in varied]
        self.assertEqual(targets, ['pump', 'cooler', 'power_unit'])

    def test_irrelevant_temperatures_are_independent_of_profile(self):
        registry = MachineRegistry.load(self.root)
        states = [spec.create(profile, 51, registry.identity)['initial_state'] for profile in spec.PROFILES]
        temperatures = [{n:m['temperature_c'] for n, m in v.items()} for v in states]
        self.assertTrue(all(v == temperatures[0] for v in temperatures))

    def test_public_assignment_is_an_allowlist_without_seed_truth_or_private_digest(self):
        registry = MachineRegistry.load(self.root)
        values = [spec.assignment(spec.create(profile, 31, registry.identity, trial_id='a'*32)) for profile in spec.PROFILES]
        self.assertTrue(all(v == values[0] for v in values))
        self.assertEqual(set(values[0]), {'schema_version', 'kind', 'trial_id', 'registry_identity', 'task'})
        private = spec.create('persistent', 31, registry.identity)
        private['future_secret'] = 'hidden-canary'
        self.assertNotIn('hidden-canary', canonical(spec.assignment(private)))

    def test_invalid_or_coherently_changed_specifications_are_refused(self):
        registry = MachineRegistry.load(self.root)
        base = spec.create('recoverable', 0, registry.identity)
        variants = []
        for field, value in (('schema_version', True), ('seed', True), ('seed', -1), ('seed', 2**32),
                ('profile', 'unknown'), ('generator', 'future'), ('source_revision', 'f'*64),
                ('registry_identity', 'e'*64), ('trial_id', '../input')):
            variants.append({**base, field:value})
        changed = copy.deepcopy(base); changed['initial_state']['pump']['fault'] = 'none'; variants.append(changed)
        changed = copy.deepcopy(base); changed['initial_state']['pump']['powered'] = 1; variants.append(changed)
        variants.append({**base, 'extra':'hidden'})
        for value in variants:
            with self.subTest(value=value), self.assertRaises(ValueError):spec.validate(canonical(value), registry.identity)
        for text in (canonical(base).replace('"schema_version":1', '"schema_version":1,"schema_version":1'),
                     canonical(base).replace('"seed":0', '"seed":NaN'), ' '* (spec.MAX_BYTES+1)):
            with self.assertRaises(ValueError):spec.validate(text, registry.identity)

    def test_real_model_uses_private_conditions_and_preserves_them_on_prototype_reset(self):
        assignment, lock, journal, prepared = self.prepared('recoverable', 1)
        model = prepared.model()
        first = model.sample(10**9)
        self.assertGreater(first['machines']['cooler']['current_a'], first['machines']['pump']['current_a'])
        self.assertEqual(model.station.machines['cooler'].fault.value, 'latched_controller')
        # This unclaimed model-only reset is not permitted once a durable owner attaches.
        model.sample(0)
        self.assertEqual(first['machines'], model.station.observe()['machines'])
        later = model.sample(10**9)
        self.assertNotEqual(first['run_id'], later['run_id'])
        self.assertEqual(later['time_s'], 1.)
        journal.claim(uuid.uuid4().hex, model.run_id, model.registry.identity, model.configuration_sha256)
        model.journal = journal
        with self.assertRaisesRegex(ValueError, 'requires_new_scenario'):model.sample(0)
        status = read_history(self.root).status()
        self.assertNotIn('configuration_sha256', canonical(status))
        self.assertNotIn('profile', canonical(status))
        self.assertEqual(status['current_scenario']['scenario_id'], assignment['trial_id'])

    def test_configured_generation_rejects_unbound_model_and_station_restart(self):
        assignment, lock, journal, prepared = self.prepared()
        model = MachineTelemetry(prepared.registry)
        with self.assertRaisesRegex(ValueError, 'configuration_mismatch'):
            journal.claim(uuid.uuid4().hex, model.run_id, model.registry.identity)
        model = prepared.model()
        journal.claim(uuid.uuid4().hex, model.run_id, model.registry.identity, prepared.digest)
        prepared.close(); journal.close(); lock.close()
        before = (self.root/'runtime/station-service/events.jsonl').read_bytes()
        harness = AdapterHarness()
        with self.assertRaisesRegex(ValueError, 'scenario_already_claimed'):harness.run(self.root, prepare=False)
        self.assertEqual(harness.messages['/station/telemetry'], [])
        self.assertEqual((self.root/'runtime/station-service/events.jsonl').read_bytes(), before)

    def test_legacy_history_is_readable_but_cannot_launch_live_station(self):
        prepare_scenario(self.root)
        before = read_history(self.root).status()
        harness = AdapterHarness()
        with self.assertRaisesRegex(ValueError, 'private_scenario_not_prepared'):harness.run(self.root, prepare=False)
        self.assertEqual(harness.messages['/station/telemetry'], [])
        self.assertEqual(read_history(self.root).status(), before)

    def test_actual_adapter_profiles_publish_only_public_fields_and_ignore_ros_fault_override(self):
        for profile, seed in (('healthy', 0), ('recoverable', 1), ('persistent', 2), ('ambiguous', 0)):
            with self.subTest(profile=profile):
                prepare_configured_scenario(self.root, profile, seed)
                harness = AdapterHarness(scenario='mechanical_jam' if profile == 'healthy' else 'none')
                harness.run(self.root, prepare=False)
                packets = [decode(text, MachineRegistry.load(self.root)) for text in harness.messages['/station/telemetry']]
                self.assertEqual(len(packets), 2)
                self.assertNotIn('scenario', harness.declarations)
                self.assertNotIn('scenario', harness.parameter_reads)
                raw = canonical(harness.messages)
                for key in ('latched_controller', 'mechanical_jam', 'initial_state', 'configuration_sha256', 'source_revision', '"seed"'):
                    self.assertNotIn(key, raw)
                measured = packets[0]
                high = [n for n, m in measured['machines'].items() if m['current_a']*measured['voltage_v'] > 110.]
                expected = [] if profile == 'healthy' else ['cooler'] if profile == 'recoverable' else ['power_unit'] if profile == 'persistent' else ['pump', 'cooler']
                self.assertEqual(sorted(high), sorted(expected))
                self.assertTrue(harness.destroyed and harness.shutdown)

    def test_blocked_descriptor_cannot_start_an_unblocked_world_or_fake_a_result(self):
        descriptor = spec.create('blocked', 1, MachineRegistry.load(self.root).identity)
        self.assertEqual(descriptor['navigation'], {'mode':'blocked_scene_required', 'waypoint_id':'cooler'})
        with self.assertRaisesRegex(ValueError, 'compiled_navigation_fixture'):
            prepare_configured_scenario(self.root, 'blocked', 1)
        self.assertFalse((self.root/'runtime/station-service').exists())

    def test_file_change_is_sticky_and_prevents_new_measurements(self):
        assignment, lock, journal, prepared = self.prepared()
        model = prepared.model(); model.sample(10**9)
        path = self.root/'runtime/station-service/scenario-inputs'/prepared.name
        before = path.read_bytes(); path.write_bytes(before+b' ')
        with self.assertRaisesRegex(ValueError, 'changed'):model.sample(2*10**9)
        path.write_bytes(before)
        with self.assertRaisesRegex(ValueError, 'unavailable'):model.sample(2*10**9)
        self.assertEqual(model.sequence, 1)

    def test_private_file_link_mode_and_generation_mismatches_are_refused(self):
        assignment, lock, journal, prepared = self.prepared()
        path = self.root/'runtime/station-service/scenario-inputs'/prepared.name
        prepared.close(); raw = path.read_bytes()
        original = path.with_suffix('.retained'); path.rename(original)
        for kind in ('symlink', 'hardlink', 'mode', 'bytes'):
            if kind == 'symlink':path.symlink_to(original.name)
            elif kind == 'hardlink':os.link(original, path)
            else:path.write_bytes(raw if kind == 'mode' else raw.replace(b'recoverable', b'persistent')); path.chmod(0o644 if kind == 'mode' else 0o600)
            try:
                with self.subTest(kind=kind), self.assertRaises((ValueError, OSError)):PreparedInput(journal, prepared.registry)
            finally:path.unlink()
        self.assertEqual(original.read_bytes(), raw)

    def test_short_write_and_failed_sync_never_prepare_a_generation(self):
        actual_write, actual_sync = os.write, os.fsync
        for mode in ('short', 'sync'):
            def write(fd, raw):return actual_write(fd, raw[:len(raw)//2]) if mode == 'short' and b'private_scenario_input' in raw else actual_write(fd, raw)
            def sync(fd):
                if mode == 'sync' and os.fstat(fd).st_size > 0:raise OSError('controlled input sync failure')
                return actual_sync(fd)
            with self.subTest(mode=mode), patch.object(storage.os, 'write', write), patch.object(storage.os, 'fsync', sync):
                with self.assertRaises(OSError):prepare_configured_scenario(self.root, 'recoverable', 0)
            self.assertEqual(read_history(self.root).status()['scenario_count'], 0)
        self.assertEqual(len(list((self.root/'runtime/station-service/scenario-inputs').glob('*.json'))), 2)

    def test_source_revision_change_refuses_before_sampling(self):
        assignment, lock, journal, prepared = self.prepared()
        model = prepared.model()
        with patch.object(spec, 'source_revision', return_value='f'*64):
            with self.assertRaisesRegex(ValueError, 'source_mismatch'):model.sample(10**9)
        self.assertEqual(model.sequence, 0)

    def test_concurrent_file_change_during_read_is_refused(self):
        assignment, lock, journal, prepared = self.prepared()
        path = self.root/'runtime/station-service/scenario-inputs'/prepared.name
        prepared.close()
        actual = os.pread
        def changed(fd, size, offset):
            raw = actual(fd, size, offset)
            if b'private_scenario_input' in raw:
                with path.open('ab') as stream:stream.write(b' ')
            return raw
        with patch.object(storage.os, 'pread', changed):
            with self.assertRaisesRegex(ValueError, 'changed'):PreparedInput(journal, prepared.registry)

    def test_coherent_replacement_cannot_change_the_journal_bound_initial_conditions(self):
        assignment, lock, journal, prepared = self.prepared('healthy', 0)
        path = self.root/'runtime/station-service/scenario-inputs'/prepared.name
        prepared.close()
        replacement = spec.create('persistent', 2, prepared.registry.identity, trial_id=prepared.scenario_id)
        self.assertEqual(spec.validate(canonical(replacement), prepared.registry.identity), replacement)
        path.write_text(canonical(replacement)+'\n')
        with self.assertRaisesRegex(ValueError, 'scenario_input_changed'):
            PreparedInput(journal, prepared.registry)
        self.assertIsNone(journal.history.scenarios[journal.history.current]['claim'])

    def test_replaced_private_directory_cannot_renew_an_open_input(self):
        assignment, lock, journal, prepared = self.prepared()
        parent = self.root/'runtime/station-service/scenario-inputs'
        raw = (parent/prepared.name).read_bytes()
        parent.rename(parent.with_name('retained-inputs'))
        parent.mkdir(mode=0o700)
        (parent/prepared.name).write_bytes(raw); (parent/prepared.name).chmod(0o600)
        with self.assertRaisesRegex(ValueError, 'directory_changed'):prepared.verify()

    def test_private_input_capacity_refuses_before_new_history_record(self):
        prepare_configured_scenario(self.root, 'recoverable', 0)
        before = (self.root/'runtime/station-service/events.jsonl').read_bytes()
        with patch.object(storage, 'MAX_INPUTS', 1):
            with self.assertRaisesRegex(ValueError, 'capacity'):prepare_configured_scenario(self.root, 'healthy', 1)
        self.assertEqual((self.root/'runtime/station-service/events.jsonl').read_bytes(), before)

    def test_operator_cli_never_prints_private_initial_conditions(self):
        script = self.root/'scripts/maintenance-journal.py'; script.parent.mkdir(exist_ok=True)
        script.write_bytes((ROOT/'scripts/maintenance-journal.py').read_bytes())
        env = {**os.environ, 'PYTHONPATH':str(ROOT), 'PYTHONDONTWRITEBYTECODE':'1'}
        for profile in ('healthy', 'recoverable', 'persistent', 'ambiguous'):
            result = subprocess.run([sys.executable, '-B', str(script), 'prepare', '--new-world', '--profile', profile, '--seed', '1234567'],
                                    env=env, capture_output=True, text=True, timeout=5.)
            self.assertEqual(result.returncode, 0, result.stderr)
            value = json.loads(result.stdout)
            self.assertEqual(set(value), {'schema_version', 'kind', 'trial_id', 'registry_identity', 'task', 'new_world_required'})
            self.assertNotIn('1234567', result.stdout)
        before = (self.root/'runtime/station-service/events.jsonl').read_bytes()
        for args in (['--seed', '-1'], ['--profile', 'blocked'], ['--seed', '4294967296']):
            result = subprocess.run([sys.executable, '-B', str(script), 'prepare', '--new-world', *args],
                                    env=env, capture_output=True, text=True, timeout=5.)
            self.assertNotEqual(result.returncode, 0)
        self.assertEqual((self.root/'runtime/station-service/events.jsonl').read_bytes(), before)
