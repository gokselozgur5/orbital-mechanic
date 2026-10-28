import contextlib
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS

from orbital import ros_proximity
from orbital.proximity import Unavailable
from orbital.telemetry_observation import canonical
from proximity_fixture import Harness
import test_map_binding as map_fixture
from test_stationary import odom

ROOT=Path(__file__).resolve().parents[1]


class InspectionObservationAdapterTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)

    def read(self,harness,machine='pump',timeout=3.):
        return harness.run(invoke=lambda:ros_proximity.read_machine_observation(
            self.root,machine,timeout=timeout,now=lambda:harness.wall))

    def test_all_machine_records_bind_the_selected_sample_to_exact_time_robot_pose(self):
        for name,y in (('pump',1.8),('cooler',0.),('power_unit',-1.8)):
            harness=Harness(self);harness.pose=(2.,y,0.)
            result=self.read(harness,name);sample=harness.last_telemetry
            with self.subTest(machine=name):
                self.assertTrue(result['valid'])
                self.assertEqual(result['machine_id'],name)
                self.assertEqual(result['robot_state']['machine_id'],name)
                self.assertEqual(result['sample_stamp_ns'],sample['stamp_ns'])
                self.assertEqual(harness.lookups[-1],sample['stamp_ns'])
                self.assertEqual(result['robot_state']['pose_stamp_ns'],sample['stamp_ns'])
                self.assertNotEqual(sample['stamp_ns'],result['observed_sim_stamp_ns']-100000000)
                self.assertEqual(result['robot_state']['observed_sim_stamp_ns'],result['observed_sim_stamp_ns'])
                self.assertGreaterEqual(sample['stamp_ns'],result['settled_at_sim_stamp_ns'])
                self.assertTrue(result['robot_state']['within_settled_inspection_pose'])
                self.assertEqual(result['measurements'],{'voltage_v':sample['voltage_v'],**sample['machines'][name]})
                self.assertEqual(result['telemetry_source']['sample_sha256'],hashlib.sha256(canonical(sample).encode()).hexdigest())
                self.assertEqual(result['registry_identity'],sample['registry']['identity'])
                self.assertEqual(result['machine_binding']['model_name'],name)
                self.assertEqual(len(result['observation_id']),32)
                self.assertEqual(len(result['authority_identity']),64)
                self.assertIn('/map_server/map',[p for p,_ in harness.reads])
                self.assertNotIn('fault',json.dumps(result))
                self.assertNotIn('scenario',json.dumps(result))
                self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_only_new_post_settling_telemetry_can_supply_an_inspection_record(self):
        harness=Harness(self)
        harness.spin_hook=lambda:setattr(harness,'moving',harness.sim<1600000000)
        result=self.read(harness)
        self.assertGreaterEqual(result['settled_at_sim_stamp_ns'],2100000000)
        self.assertGreaterEqual(result['sample_stamp_ns'],2500000000)
        self.assertEqual(result['telemetry_source']['sequence'],3)

    def test_lagging_fresh_odometry_is_waited_for_at_the_selected_sample_time(self):
        harness=Harness(self);harness.odom_delay_ns=80000000
        result=self.read(harness)
        self.assertGreaterEqual(result['observed_sim_stamp_ns']-result['sample_stamp_ns'],80000000)
        self.assertEqual(result['robot_state']['pose_stamp_ns'],result['sample_stamp_ns'])
        self.assertGreater(len(harness.lookups),1)

    def test_missing_or_invalid_telemetry_never_yields_a_partial_record(self):
        for kind in ('drop','publisher','unknown','catalog','replay','run'):
            harness=Harness(self)
            if kind=='drop':harness.drop_telemetry=True
            elif kind=='publisher':harness.missing_telemetry_publisher=True
            else:
                first=[None]
                def packet(value):
                    if first[0] is None:first[0]=json.loads(json.dumps(value))
                    if kind=='unknown':value['machines']['pump']['temperature_c']=None
                    elif kind=='catalog':value['registry']['identity']='wrong'
                    elif kind=='replay':value=first[0]
                    elif value['sequence']>1:value['run_id']='f'*32
                    return value
                harness.telemetry_hook=packet
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.read(harness,timeout=2.)
            self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_wrong_pose_map_or_transform_is_refused(self):
        for kind in ('heading','position','map','parameters','transform'):
            harness=Harness(self)
            if kind=='heading':harness.pose=(2.,1.8,math.pi)
            elif kind=='position':harness.pose=(1.8,1.8,0.)
            elif kind=='map':harness.wrong_map=True
            elif kind=='parameters':harness.wrong_params=True
            else:harness.wrong_tf=True
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.read(harness)

    def test_sample_expiry_does_not_fall_back_to_a_newer_telemetry_or_latest_tf(self):
        harness=Harness(self);harness.no_tf=True
        with self.assertRaisesRegex(Unavailable,'inspection_sample_expired'):self.read(harness)
        self.assertEqual(len(set(harness.lookups)),1)
        self.assertEqual(harness.last_telemetry['sequence'],2)

    def test_new_valid_telemetry_during_map_or_tf_collection_invalidates_selection(self):
        for stage in ('map','tf'):
            harness=Harness(self);fired=[False]
            def change(*args):
                if fired[0]:return
                fired[0]=True;harness.sim+=10000000;harness.wall+=.01
                packet=harness.telemetry_model.sample(harness.sim)
                harness.callbacks['/station/telemetry'](NS(data=json.dumps(packet)))
            if stage=='map':harness.call_hook=change
            else:harness.lookup_hook=change
            with self.subTest(stage=stage),self.assertRaisesRegex(Unavailable,'inspection_telemetry_changed'):
                self.read(harness)

    def test_movement_after_settling_invalidates_even_if_odometry_stops_again(self):
        harness=Harness(self)
        def move():
            harness.sim+=10000000;harness.wall+=.01
            harness.callbacks['/odom'](odom(harness.sim,(.1,0,0)))
            harness.sim+=10000000;harness.wall+=.01
            harness.callbacks['/odom'](odom(harness.sim))
        harness.lookup_hook=move
        with self.assertRaisesRegex(Unavailable,'stationarity_changed'):self.read(harness)

    def test_movement_while_waiting_for_the_post_settling_sample_invalidates_the_query(self):
        harness=Harness(self)
        harness.spin_hook=lambda:setattr(harness,'moving',1700000000<=harness.sim<1740000000)
        with self.assertRaisesRegex(Unavailable,'inspection_stationarity_changed'):self.read(harness)
        self.assertEqual(harness.lookups,[])

    def test_interrupt_cleans_up_the_shared_observer_without_a_partial_record(self):
        harness=Harness(self)
        with patch.object(harness.modules['rclpy'],'spin_once',side_effect=KeyboardInterrupt),self.assertRaises(KeyboardInterrupt):
            self.read(harness)
        self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_invalid_library_timeout_is_refused_before_ros(self):
        harness=Harness(self)
        for timeout in (True,float('nan'),0.,11.):
            with patch.object(harness.modules['rclpy'],'init',side_effect=AssertionError('must not initialize')), \
                    self.subTest(timeout=timeout),self.assertRaises(ValueError):
                self.read(harness,timeout=timeout)

    def test_final_result_preparation_rechecks_source_clock_publisher_and_deadline(self):
        original=ros_proximity.uuid.uuid4;path=self.root/'worlds/maintenance.sdf';before=path.read_bytes()
        for kind in ('source','mode','gid','deadline','clock'):
            harness=Harness(self)
            def identity():
                if harness.lookups:
                    if kind=='source':path.write_bytes(before+b'\n')
                    elif kind=='mode':harness.clock_mode_changed=True
                    elif kind=='gid':harness.changed_gid=True
                    elif kind=='deadline':harness.wall+=3.
                    else:harness.sim-=100000000
                return original()
            try:
                with patch.object(ros_proximity.uuid,'uuid4',identity),self.subTest(kind=kind),self.assertRaises(ValueError):
                    self.read(harness)
            finally:path.write_bytes(before)

    def test_shipped_cli_returns_one_complete_record_or_invalid_json(self):
        spec=importlib.util.spec_from_file_location('machine_observation_cli',ROOT/'scripts/machine-observation.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        for moving,code in ((False,0),(True,2)):
            harness=Harness(self);harness.moving=moving;output=io.StringIO()
            def measured(root,machine,**kwargs):return ros_proximity.read_machine_observation(root,machine,now=lambda:harness.wall,**kwargs)
            with patch.dict(os.environ,{'ROS_DOMAIN_ID':'42','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}), \
                    patch.object(cli,'ROOT',self.root),patch.object(cli,'read_machine_observation',measured),contextlib.redirect_stdout(output):
                self.assertEqual(harness.run(invoke=lambda:cli.main(['pump','--timeout','2'])),code)
            value=json.loads(output.getvalue());self.assertEqual(value['valid'],not moving)
            if not moving:self.assertEqual(value['kind'],'machine_inspection_observation')
            else:self.assertNotIn('measurements',value)

    def test_cli_help_invalid_inputs_and_foreign_domains_do_not_need_ros(self):
        for args,code in ((['--help'],0),(['unknown'],2),(['pump','--timeout','nan'],2),
                          (['pump','--pose','2,1.8,0'],2),(['pump','--telemetry','sample.json'],2),(['pump'],2)):
            env=dict(os.environ,ROS_DOMAIN_ID='77',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
            result=subprocess.run(['python3',str(ROOT/'scripts/machine-observation.py'),*args],capture_output=True,text=True,env=env)
            self.assertEqual(result.returncode,code)
            self.assertNotIn('ModuleNotFoundError',result.stderr)
