import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
from unittest.mock import patch

from orbital.machine_registry import MachineRegistry
from orbital.proximity import Unavailable
from orbital import ros_station_observation
from station_observation_fixture import Harness
from test_machine_registry import RegistryFixture,ROOT


class StationObservationAdapterTests(RegistryFixture):
    def test_actual_publisher_packets_pass_production_reader_with_current_identity(self):
        harness=Harness(self);result=harness.run()
        self.assertTrue(result['valid'])
        self.assertEqual(result['telemetry'],json.loads(harness.packets[1]))
        self.assertGreaterEqual(harness.spins,12)
        self.assertEqual(result['freshness']['consecutive_samples'],2)
        self.assertEqual(len(result['observation_id']),32)
        self.assertEqual(len(result['authority_identity']),64)
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_wrong_missing_duplicate_or_invalid_publisher_identity_is_refused(self):
        for field in ('missing','duplicate','wrong_owner','bad_gid'):
            harness=Harness(self);setattr(harness,field,True)
            with self.subTest(field=field),self.assertRaisesRegex(Unavailable,'deadline'):
                harness.run(timeout=1.)
            self.assertEqual(harness.delivered,0)
            self.assertTrue(harness.destroyed and harness.shutdown)

    def test_absent_paused_stale_or_replayed_data_cannot_be_observed(self):
        for kind in ('drop','paused','stale','replay','unknown'):
            harness=Harness(self)
            if kind in ('drop','paused'):setattr(harness,kind,True)
            else:
                def packet(index,text):
                    value=json.loads(text)
                    if kind=='stale':value['stamp_ns']=1
                    elif kind=='replay':return harness.packets[0]
                    else:value['machines']['pump']['current_a']=None
                    return json.dumps(value)
                harness.before_packet=packet
            with self.subTest(kind=kind),self.assertRaises(Unavailable):harness.run(timeout=1.)
            self.assertTrue(harness.destroyed and harness.shutdown)

    def test_publisher_clock_mode_and_clock_reset_are_checked_during_collection(self):
        for kind in ('gid','mode','clock'):
            harness=Harness(self)
            def change():
                if harness.spins!=8:return
                if kind=='gid':harness.gid=2
                elif kind=='mode':harness.mode=False
                else:harness.sim=1
            harness.before_spin=change
            with self.subTest(kind=kind),self.assertRaises(Unavailable):harness.run()

    def test_packet_fault_after_a_good_sample_is_not_erased_by_future_good_data(self):
        harness=Harness(self)
        harness.before_packet=lambda index,text:'{"fault":"mechanical_jam"}' if index==1 else text
        with self.assertRaisesRegex(Unavailable,'^invalid_telemetry$'):harness.run()
        self.assertEqual(harness.delivered,2)

    def test_registry_change_is_refused_before_returning_measurements(self):
        harness=Harness(self)
        def packet(index,text):
            if index==1:self.world_path.write_bytes(self.world_path.read_bytes()+b'\n')
            return text
        harness.before_packet=packet
        with self.assertRaisesRegex(ValueError,'source changed'):harness.run()
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_final_source_verification_cannot_hide_changed_generation_mode_or_expired_data(self):
        original=MachineRegistry.verify_current
        for kind in ('gid','mode','age','deadline'):
            harness=Harness(self)
            def verify(registry):
                original(registry)
                if harness.delivered!=2:return
                if kind=='gid':harness.gid=2
                elif kind=='mode':harness.mode=False
                elif kind=='age':harness.wall+=1.01
                else:harness.wall+=2.01
            with patch.object(MachineRegistry,'verify_current',verify),self.subTest(kind=kind),self.assertRaises(Unavailable):
                harness.run()
            self.assertTrue(harness.destroyed and harness.shutdown)

    def test_registry_preparation_deadline_precedes_ros_initialization(self):
        harness=Harness(self);original=MachineRegistry.load
        def slow(root):
            value=original(root);harness.wall+=.2;return value
        with patch.object(MachineRegistry,'load',slow),self.assertRaisesRegex(Unavailable,'deadline'):
            harness.run(timeout=.1)
        self.assertFalse(harness.initialized)

    def test_result_identity_work_cannot_bypass_final_validity_checks(self):
        original=ros_station_observation.uuid.uuid4
        for kind in ('gid','mode','age','deadline','source'):
            harness=Harness(self);source=self.world_path.read_bytes()
            def identity():
                if harness.delivered==2:
                    if kind=='gid':harness.gid=2
                    elif kind=='mode':harness.mode=False
                    elif kind=='age':harness.wall+=1.01
                    elif kind=='deadline':harness.wall+=2.01
                    else:self.world_path.write_bytes(source+b'\n')
                return original()
            try:
                with patch.object(ros_station_observation.uuid,'uuid4',identity),self.subTest(kind=kind),self.assertRaises(ValueError):
                    harness.run()
                self.assertTrue(harness.destroyed and harness.shutdown)
            finally:self.world_path.write_bytes(source)

    def test_interrupt_cleans_up_without_any_action_side_effect(self):
        harness=Harness(self)
        def interrupt():raise KeyboardInterrupt()
        harness.before_spin=interrupt
        with self.assertRaises(KeyboardInterrupt):harness.run()
        self.assertTrue(harness.destroyed and harness.shutdown)

    def test_shipped_cli_reports_measurements_or_invalid_json_without_a_diagnosis(self):
        spec=importlib.util.spec_from_file_location('station_cli',ROOT/'scripts/observe-station.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        for invalid,code in ((False,0),(True,2)):
            harness=Harness(self)
            if invalid:harness.before_packet=lambda index,text:'{"fault":"hidden"}'
            output=io.StringIO()
            def measured(root,**kwargs):return ros_station_observation.run(root,now=lambda:harness.wall,**kwargs)
            with patch.dict(os.environ,{'ROS_DOMAIN_ID':'42','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}), \
                    patch.object(cli,'ROOT',self.root),patch.object(cli,'run',measured),contextlib.redirect_stdout(output):
                self.assertEqual(harness.run(invoke=lambda:cli.main(['--timeout','2'])),code)
            result=json.loads(output.getvalue());self.assertEqual(result['valid'],not invalid)
            self.assertNotIn('hidden',output.getvalue())
            if invalid:self.assertNotIn('telemetry',result)

    def test_cli_help_bad_input_and_other_project_domains_need_no_ros(self):
        for args,code in ((['--help'],0),(['--timeout','nan'],2),(['--timeout','.01'],2),([],2)):
            env=dict(os.environ,ROS_DOMAIN_ID='77',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
            result=subprocess.run(['python3',str(ROOT/'scripts/observe-station.py'),*args],capture_output=True,text=True,env=env)
            self.assertEqual(result.returncode,code)
            self.assertNotIn('ModuleNotFoundError',result.stderr)
