"""Run the shipped launch preflight without starting ROS, Gazebo or systemd."""
import json
import os
import subprocess
import uuid
from unittest.mock import patch

from orbital.maintenance_journal import prepare_configured_scenario, read_history
from orbital.scenario_input import check_prepared
from test_machine_registry import RegistryFixture, ROOT
from test_machine_telemetry_adapter import AdapterHarness


class ScenarioLaunchTests(RegistryFixture):
    def setUp(self):
        super().setUp()
        scripts = self.root/'scripts'; scripts.mkdir()
        for name in ('start-demo.sh', 'maintenance-journal.py'):
            (scripts/name).write_bytes((ROOT/'scripts'/name).read_bytes())
        self.bin_dir = self.root/'test-bin'; self.bin_dir.mkdir()
        cutoff = self.bin_dir/'mkdir'
        cutoff.write_text('''#!/usr/bin/env bash
printf '%s' "$ORBITAL_TRIAL_ID" > "$SCENARIO_TEST_RECEIPT"
exit 77
''')
        cutoff.chmod(0o755)
        self.receipt = self.root/'launch-receipt'
        self.env = {**os.environ, 'PATH':str(self.bin_dir)+os.pathsep+os.environ['PATH'],
                    'PYTHONPATH':str(ROOT), 'PYTHONDONTWRITEBYTECODE':'1',
                    'SCENARIO_TEST_RECEIPT':str(self.receipt)}

    def launch(self, *args):
        return subprocess.run(['bash', str(self.root/'scripts/start-demo.sh'), *args],
                              env=self.env, capture_output=True, text=True, timeout=5.)

    def test_prepared_nondefault_input_survives_actual_shell_preflight(self):
        assignment = prepare_configured_scenario(self.root, 'persistent', 1234567)
        before = (self.root/'runtime/station-service/events.jsonl').read_bytes()
        result = self.launch('--trial-id', assignment['trial_id'])
        self.assertEqual(result.returncode, 77, result.stderr)  # Cut before any component/cache setup.
        self.assertEqual(self.receipt.read_text(), assignment['trial_id'])
        self.assertEqual(json.loads(result.stdout), check_prepared(self.root, assignment['trial_id']))
        for secret in ('persistent', '1234567', 'mechanical_jam', 'configuration_sha256'):
            self.assertNotIn(secret, result.stdout+result.stderr)
        self.assertEqual((self.root/'runtime/station-service/events.jsonl').read_bytes(), before)
        harness = AdapterHarness()
        with patch.dict(os.environ, {'ORBITAL_TRIAL_ID':assignment['trial_id']}):
            harness.run(self.root, prepare=False)
        value = json.loads(harness.messages['/station/telemetry'][0])
        self.assertGreater(value['machines']['cooler']['vibration_au'], 1.)
        self.assertEqual(read_history(self.root).current, assignment['trial_id'])

    def test_default_launch_prepares_exactly_one_configured_generation(self):
        result = self.launch()
        self.assertEqual(result.returncode, 77, result.stderr)
        history = read_history(self.root)
        self.assertEqual(len(history.scenarios), 1)
        self.assertEqual(self.receipt.read_text(), history.current)
        self.assertIsNone(history.scenarios[history.current]['claim'])
        self.assertIn('configuration_sha256', history.scenarios[history.current])

    def test_stale_trial_and_invalid_arguments_cannot_start_components_or_reprepare(self):
        old = prepare_configured_scenario(self.root, 'healthy', 0)
        prepare_configured_scenario(self.root, 'persistent', 2)
        before = (self.root/'runtime/station-service/events.jsonl').read_bytes()
        for args in (('--trial-id', old['trial_id']), ('--trial-id',), ('--profile', 'healthy'),
                     ('--trial-id', '../input'), ('--trial-id', uuid.uuid4().hex)):
            with self.subTest(args=args):
                result = self.launch(*args)
                self.assertNotEqual(result.returncode, 77)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.receipt.exists())
                self.assertEqual((self.root/'runtime/station-service/events.jsonl').read_bytes(), before)

    def test_preflight_is_rechecked_against_station_start_generation(self):
        first = prepare_configured_scenario(self.root, 'healthy', 0)
        check_prepared(self.root, first['trial_id'])
        current = prepare_configured_scenario(self.root, 'persistent', 2)
        harness = AdapterHarness()
        with patch.dict(os.environ, {'ORBITAL_TRIAL_ID':first['trial_id']}):
            with self.assertRaisesRegex(ValueError, 'launch_identity_mismatch'):
                harness.run(self.root, prepare=False)
        self.assertEqual(harness.messages['/station/telemetry'], [])
        self.assertIsNone(read_history(self.root).scenarios[current['trial_id']]['claim'])
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_claimed_input_cannot_be_reused_for_a_new_launch(self):
        value = prepare_configured_scenario(self.root, 'healthy', 0)
        AdapterHarness().run(self.root, prepare=False)
        before = (self.root/'runtime/station-service/events.jsonl').read_bytes()
        result = self.launch('--trial-id', value['trial_id'])
        self.assertEqual(result.returncode, 2)
        self.assertIn('scenario_already_claimed', result.stderr)
        self.assertFalse(self.receipt.exists())
        self.assertEqual((self.root/'runtime/station-service/events.jsonl').read_bytes(), before)

    def test_changed_private_input_refuses_before_any_component_start(self):
        value = prepare_configured_scenario(self.root, 'healthy', 0)
        path = self.root/'runtime/station-service/scenario-inputs'/(value['trial_id']+'.json')
        path.write_bytes(path.read_bytes()+b' ')
        result = self.launch('--trial-id', value['trial_id'])
        self.assertEqual(result.returncode, 2)
        self.assertFalse(self.receipt.exists())
        self.assertIsNone(read_history(self.root).scenarios[value['trial_id']]['claim'])
