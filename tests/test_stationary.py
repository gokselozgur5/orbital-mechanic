import contextlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from orbital.machine_registry import MachineRegistry
from orbital.proximity import ProximityEvidence,Unavailable
from orbital.stationary import StationaryWindow
from orbital import ros_proximity
from proximity_fixture import Harness
import test_map_binding as map_fixture
from test_machine_registry import RegistryFixture
from test_proximity import health,transform

ROOT = Path(__file__).resolve().parents[1]


def odom(stamp,linear=(0.,0.,0.),angular=(0.,0.,0.)):
    return NS(header=NS(frame_id='odom',stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9)),
              child_frame_id='base_footprint',pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),
                    orientation=NS(x=0.,y=0.,z=0.,w=1.))),
              twist=NS(twist=NS(linear=NS(x=linear[0],y=linear[1],z=linear[2]),
                               angular=NS(x=angular[0],y=angular[1],z=angular[2]))))


class StationaryWindowTests(unittest.TestCase):
    def window(self,wall_step=.1,sim_step=100000000):
        value = StationaryWindow()
        for i in range(6):
            stamp=1000000000+i*sim_step
            value.observe(odom(stamp),10.+i*wall_step,stamp,1000000000)
        return value

    def test_requires_observed_half_second_in_both_clocks(self):
        value=self.window().report(10.51,1500000000,1400000000)
        self.assertTrue(value['settled'])
        self.assertEqual(value['samples'],6)
        self.assertEqual(value['simulation_span_s'],.5)
        self.assertEqual(value['wall_span_s'],.5)
        self.assertFalse(self.window(.01).report(10.06,1500000000,1400000000)['settled'])
        self.assertFalse(self.window(sim_step=10000000).report(10.51,1050000000,1000000000)['settled'])

    def test_waiting_without_new_samples_or_repeating_stamp_never_settles(self):
        value=StationaryWindow()
        value.observe(odom(10**9),10.,10**9,10**9)
        self.assertFalse(value.report(10.6,1600000000,1500000000)['settled'])
        for i in range(12): value.observe(odom(10**9),10.+i*.05,10**9,10**9)
        self.assertFalse(value.report(10.56,10**9,900000000)['settled'])

    def test_all_velocity_axes_reset_the_window_and_require_new_history(self):
        for linear,angular in (((.010001,0,0),(0,0,0)),((0,.010001,0),(0,0,0)),
                               ((0,0,.010001),(0,0,0)),((0,0,0),(.020001,0,0)),
                               ((0,0,0),(0,.020001,0)),((0,0,0),(0,0,.020001))):
            value=self.window();old=value.generation
            value.observe(odom(1550000000,linear,angular),10.55,1550000000,10**9)
            value.observe(odom(1600000000),10.6,1600000000,10**9)
            self.assertFalse(value.report(10.61,1600000000,1500000000)['settled'])
            self.assertGreater(value.generation,old)
            for i in range(1,7):
                stamp=1600000000+i*100000000
                value.observe(odom(stamp),10.6+i*.1,stamp,10**9)
            self.assertTrue(value.report(11.21,2200000000,2100000000)['settled'])

    def test_velocity_norm_boundary_is_shared_and_not_component_wise(self):
        for linear,angular,expected in (((.006,.008,0),(0,0,.02),True),
                                       ((.008,.008,0),(0,0,0),False)):
            value=StationaryWindow()
            for i in range(6):
                stamp=10**9+i*100000000
                value.observe(odom(stamp,linear,angular),10.+i*.1,stamp,10**9)
            self.assertEqual(value.report(10.51,1500000000,1400000000)['settled'],expected)

    def test_receipt_or_simulation_gap_discards_previous_settling(self):
        for stamp,wall in ((1700000000,10.6),(1600000000,10.71)):
            value=self.window()
            value.observe(odom(stamp),wall,stamp,10**9)
            self.assertFalse(value.report(wall+.01,stamp,stamp-100000000)['settled'])
            self.assertEqual(value.samples,1)

    def test_invalid_frames_values_epoch_and_order_cannot_reuse_old_window(self):
        for kind in ('frame','child','tilt','nan','boolean','duplicate','regressed','future','epoch','missing'):
            value=self.window();msg=odom(1550000000);sim=1550000000;floor=10**9
            if kind=='frame':msg.header.frame_id='map'
            elif kind=='child':msg.child_frame_id='base_link'
            elif kind=='tilt':msg.pose.pose.orientation.x=.1
            elif kind=='nan':msg.twist.twist.linear.x=float('nan')
            elif kind=='boolean':msg.twist.twist.linear.x=True
            elif kind=='duplicate':msg=odom(1500000000)
            elif kind=='regressed':msg=odom(1400000000)
            elif kind=='future':sim=1300000000
            elif kind=='epoch':floor=1600000000
            else:msg=None
            with self.subTest(kind=kind):
                self.assertFalse(value.observe(msg,10.55,sim,floor))
                self.assertFalse(value.report(10.56,sim,sim-100000000)['settled'])

    def test_last_odometry_must_be_fresh_and_cover_the_pose_time(self):
        self.assertFalse(self.window().report(10.71,1500000000,1400000000)['settled'])
        self.assertFalse(self.window().report(10.51,1700000000,1600000000)['settled'])
        self.assertFalse(self.window().report(10.51,1490000000,1390000000)['settled'])
        self.assertFalse(self.window().report(10.51,1510000000,1510000000)['settled'])


class InspectionSettlingTests(RegistryFixture):
    def settled(self):
        value=ProximityEvidence(MachineRegistry.load(self.root),'pump')
        for i in range(11):
            stamp=10**9+i*50000000;wall=10.+i*.05
            value.health(health(i+1,stamp/1e9),wall)
            value.odometry(odom(stamp),wall,stamp)
        return value

    def test_fresh_geometry_and_stationary_health_alone_are_insufficient(self):
        value=ProximityEvidence(MachineRegistry.load(self.root),'pump')
        value.health(health(),10.);value.health(health(2,1.2),10.1)
        result=value.measure(1100000000,1200000000,10.11,transform(1100000000))
        self.assertTrue(result['within_inspection_pose'])
        self.assertFalse(result['within_settled_inspection_pose'])
        value=self.settled();result=value.measure(1400000000,1500000000,10.51,transform(1400000000))
        self.assertTrue(result['within_settled_inspection_pose'])
        self.assertTrue(value.lease.pending_stop)  # Observation never acknowledges motion ownership.

    def test_movement_between_health_reports_is_not_erased_by_stopped_odometry(self):
        value=self.settled();old=value.stationary.generation
        value.odometry(odom(1510000000,(.1,0,0)),10.51,1510000000)
        value.odometry(odom(1520000000),10.52,1520000000)
        result=value.measure(1420000000,1520000000,10.53,transform(1420000000))
        self.assertTrue(value.lease.stationary)
        self.assertFalse(result['within_settled_inspection_pose'])
        self.assertGreater(value.stationary.generation,old)

    def test_stale_or_future_health_cannot_accumulate_a_stopped_window(self):
        for offset in (-.3,.05):
            value=ProximityEvidence(MachineRegistry.load(self.root),'pump')
            for i in range(21):
                stamp=10**9+i*50000000;wall=10.+i*.05
                value.health(health(i+1,stamp/1e9+offset),wall)
                value.odometry(odom(stamp),wall,stamp)
            # Recover with one current sample. Earlier invalid health cannot
            # contribute the half-second history needed by this measurement.
            value.health(health(22,2.1),11.1)
            value.odometry(odom(2100000000),11.1,2100000000)
            result=value.measure(2000000000,2100000000,11.11,transform(2000000000))
            with self.subTest(offset=offset):
                self.assertFalse(result['within_settled_inspection_pose'])

    def test_missing_health_sequence_frozen_health_or_motion_resets_window(self):
        for sequence,sim,stationary in ((13,1.55,True),(12,1.5,True),(12,1.55,False)):
            value=self.settled()
            value.health(health(sequence,sim,stationary=stationary),10.55)
            value.odometry(odom(1550000000),10.56,1550000000)
            result=value.measure(1450000000,1550000000,10.57,transform(1450000000))
            self.assertFalse(result['within_settled_inspection_pose'])


class SettledAdapterTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)

    def test_production_reader_waits_for_pose_and_both_settling_clocks(self):
        harness=Harness(self)
        result=harness.run(invoke=lambda:ros_proximity.read_inspection_state(self.root,'pump',timeout=2.,now=lambda:harness.wall))
        self.assertTrue(result['within_settled_inspection_pose'])
        self.assertGreaterEqual(result['stationarity']['wall_span_s'],.5)
        self.assertGreaterEqual(result['stationarity']['simulation_span_s'],.5)
        self.assertLess(result['map_observed_age_s'],.5)
        self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_initial_motion_requires_a_complete_new_stopped_window(self):
        harness=Harness(self)
        harness.spin_hook=lambda:setattr(harness,'moving',harness.sequence<10)
        result=harness.run(require_settled=True)
        self.assertGreaterEqual(result['stationarity']['first_stamp_ns'],1200000000)
        self.assertTrue(result['within_settled_inspection_pose'])

    def test_unavailable_odom_authority_moving_missing_or_paused_inputs_do_not_pass(self):
        for name in ('moving','drop_odom','paused','missing_odom_publisher'):
            harness=Harness(self);setattr(harness,name,True)
            with self.subTest(name=name),self.assertRaises(Unavailable):harness.run(timeout=1.,require_settled=True)
            self.assertTrue(harness.destroyed and harness.shutdown)

    def test_movement_after_settling_invalidates_even_a_resettled_query(self):
        for stage in ('map','tf'):
            harness=Harness(self);fired=[False]
            def moved(*args):
                if fired[0]:return
                fired[0]=True
                harness.sim+=10000000;harness.wall+=.01
                harness.callbacks['/odom'](odom(harness.sim,(.1,0,0)))
                for _ in range(30):harness.modules['rclpy'].spin_once(None,timeout_sec=.02)
            if stage=='map':harness.call_hook=moved
            else:harness.lookup_hook=moved
            with self.subTest(stage=stage),self.assertRaisesRegex(Unavailable,'stationarity_changed'):
                harness.run(require_settled=True)

    def test_settled_robot_in_wrong_pose_is_refused(self):
        harness=Harness(self);harness.pose=(2.,1.8,math.pi)
        with self.assertRaisesRegex(Unavailable,'settled_inspection_pose_required'):
            harness.run(require_settled=True)

    def test_expired_preparation_does_not_start_ros_observation(self):
        harness=Harness(self);original=ros_proximity.MapSnapshot.prepare
        def delayed(registry):
            value=original(registry);harness.wall+=.2;return value
        with patch.object(ros_proximity.MapSnapshot,'prepare',side_effect=delayed), \
                patch.object(harness.modules['rclpy'],'init',side_effect=AssertionError('ROS must not start')), \
                self.assertRaisesRegex(Unavailable,'deadline'):
            harness.run(timeout=.1,require_settled=True)
        self.assertEqual(harness.reads,[])

    def test_prepared_map_change_during_final_lookup_is_refused(self):
        for name in ('orbital/inspection_geometry.py','runtime/maps/'+self.bundle['revision']+'/map.pgm',
                     'runtime/maps/'+self.bundle['revision']+'/manifest.json'):
            path=self.root/name;before=path.read_bytes();harness=Harness(self)
            harness.lookup_hook=lambda:path.write_bytes(before+b'\n')
            try:
                with self.subTest(name=name),self.assertRaisesRegex(ValueError,'Map snapshot'):
                    harness.run(require_settled=True)
                self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)
            finally:path.write_bytes(before)

    def test_shipped_cli_enforces_the_settled_mode(self):
        spec=importlib.util.spec_from_file_location('settled_cli',ROOT/'scripts/machine-proximity.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        for moving,expected in ((False,0),(True,2)):
            harness=Harness(self);harness.moving=moving;output=io.StringIO()
            def measured(root,machine,**kwargs):return ros_proximity.run(root,machine,now=lambda:harness.wall,**kwargs)
            with patch.dict(os.environ,{'ROS_DOMAIN_ID':'42','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}), \
                    patch.object(cli,'ROOT',self.root),patch.object(cli,'run',measured),contextlib.redirect_stdout(output):
                self.assertEqual(harness.run(invoke=lambda:cli.main(['pump','--require-settled-pose','--timeout','1'])),expected)
            result=json.loads(output.getvalue())
            if expected==0:self.assertTrue(result['within_settled_inspection_pose'])
            else:self.assertFalse(result['valid'])
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as error:
            cli.main(['pump','--require-inspection-pose','--require-settled-pose'])
        self.assertEqual(error.exception.code,2)
