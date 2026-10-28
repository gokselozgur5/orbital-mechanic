import importlib.util
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

from proximity_fixture import Harness
from orbital import ros_proximity
import test_map_binding as map_fixture
from test_proximity import health
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]


class ProximityAdapterTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)

    def test_actual_adapter_uses_real_map_binding_and_time_specific_tf(self):
        harness = Harness(self)
        result = harness.run()
        self.assertTrue(result['valid'] and result['within_distance'])
        self.assertAlmostEqual(result['distance_m'],.6)
        self.assertEqual(result['map_binding']['config_sha256'],harness.config_hash)
        self.assertIn('/map_server/map',[path for path,names in harness.reads])
        self.assertIn('/orbital_navigation_supervisor/get_parameters',[path for path,names in harness.reads])
        self.assertLess(result['map_observed_age_s'],.5)
        self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)
        self.assertEqual(harness.clients,[])

    def test_wrong_actual_map_cells_or_consumer_parameters_refuse_measurement(self):
        for kind in ('wrong_map','wrong_params'):
            harness = Harness(self)
            setattr(harness,kind,True)
            with self.subTest(kind=kind),self.assertRaises(ValueError): harness.run()
            self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)
            self.assertEqual(harness.clients,[])

    def test_missing_or_wrong_time_tf_and_unhealthy_or_paused_data_never_produce_distance(self):
        for kind in ('no_tf','wrong_tf','unhealthy','paused','drop_health'):
            harness = Harness(self)
            setattr(harness,kind,True)
            with self.subTest(kind=kind),self.assertRaises(ValueError): harness.run()
            self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_source_change_during_measurement_is_rejected(self):
        harness = Harness(self)
        path = self.root/'worlds/maintenance.sdf'
        harness.lookup_hook = lambda:path.write_bytes(path.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError,'source changed'): harness.run()

    def test_publisher_generation_change_during_lookup_is_rejected(self):
        harness = Harness(self)
        harness.lookup_hook = lambda:setattr(harness,'changed_gid',True)
        with self.assertRaisesRegex(ValueError,'publisher_generation_changed'): harness.run()

    def test_expired_map_observation_is_not_rescued_by_fresh_health(self):
        harness = Harness(self)
        def late():
            harness.wall += .5
            harness.sim += 10000000
            harness.sequence += 1
            harness.callbacks['/navigation/health'](NS(data=health(harness.sequence,harness.sim/1e9)))
        harness.lookup_hook = late
        with self.assertRaisesRegex(ValueError,'map_binding_observation_expired'): harness.run()

    def test_clock_mode_change_during_lookup_is_refused(self):
        harness = Harness(self)
        harness.lookup_hook = lambda:setattr(harness,'clock_mode_changed',True)
        with self.assertRaisesRegex(ValueError,'clock_mode'): harness.run()

    def test_health_epoch_change_during_lookup_invalidates_measurement(self):
        harness = Harness(self)
        def changed():
            harness.sequence += 1
            harness.callbacks['/navigation/health'](NS(data=health(harness.sequence,harness.sim/1e9,epoch=1)))
        harness.lookup_hook = changed
        with self.assertRaisesRegex(ValueError,'generation_changed'): harness.run()

    def test_expired_overall_deadline_cannot_accept_a_late_transform(self):
        harness = Harness(self)
        def late(): harness.wall += 3.
        harness.lookup_hook = late
        with self.assertRaises(ValueError): harness.run()

    def test_wall_clock_mode_change_is_refused(self):
        harness = Harness(self)
        harness.clock_mode_changed = True
        with self.assertRaisesRegex(ValueError,'clock_mode'): harness.run()

    def test_result_identity_work_cannot_bypass_pose_validity_checks(self):
        original=ros_proximity.uuid.uuid4
        path=self.root/'worlds/maintenance.sdf';source=path.read_bytes()
        for settled in (False,True):
            for kind in ('gid','mode','age','deadline','source'):
                harness=Harness(self)
                def identity():
                    if harness.lookups:
                        if kind=='gid':harness.changed_gid=True
                        elif kind=='mode':harness.clock_mode_changed=True
                        elif kind=='age':harness.wall+=.51
                        elif kind=='deadline':harness.wall+=3.
                        else:path.write_bytes(source+b'\n')
                    return original()
                try:
                    with patch.object(ros_proximity.uuid,'uuid4',identity), \
                            self.subTest(kind=kind,settled=settled),self.assertRaises(ValueError):
                        harness.run(require_settled=settled)
                    self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)
                finally:path.write_bytes(source)


class ProximityCliTests(unittest.TestCase):
    def command(self,*args,local=True):
        env = {**os.environ,'ROS_DOMAIN_ID':'42','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
        if not local: env['ROS_DOMAIN_ID'] = '77'
        return subprocess.run([sys.executable,str(ROOT/'scripts/machine-proximity.py'),*args],env=env,
                              capture_output=True,text=True,timeout=5)

    def test_help_and_invalid_input_do_not_need_ros(self):
        self.assertEqual(self.command('--help').returncode,0)
        for args in (('unknown',),('pump','--distance','0'),('pump','--pose','0,0'),
                     ('pump','--timeout','nan'),('pump','--timeout','11')):
            result = self.command(*args)
            self.assertEqual(result.returncode,2,result.stderr)
            self.assertNotIn('ModuleNotFoundError',result.stderr)

    def test_other_project_domain_is_refused_before_ros(self):
        result = self.command('pump',local=False)
        self.assertEqual(result.returncode,2)
        self.assertIn('localhost',result.stderr)

    def test_missing_ros_returns_invalid_json_without_a_fabricated_distance(self):
        spec = importlib.util.spec_from_file_location('proximity_cli',ROOT/'scripts/machine-proximity.py')
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        stream = io.StringIO()
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'42','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}), \
                patch.object(cli,'run',side_effect=ImportError('controlled missing ROS')), \
                contextlib.redirect_stdout(stream):
            self.assertEqual(cli.main(['pump']),2)
        value = json.loads(stream.getvalue())
        self.assertFalse(value['valid'] or value['within_distance'])
        self.assertNotIn('distance_m',value)
