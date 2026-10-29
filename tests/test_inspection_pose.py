"""Measured inspection geometry through production evidence and actual ROS adapters."""
import contextlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from orbital.machine_registry import MachineRegistry
from orbital.proximity import ProximityEvidence
from orbital import ros_proximity
from proximity_fixture import Harness
import test_map_binding as map_fixture
from test_machine_registry import RegistryFixture
from test_proximity import health,transform

ROOT = Path(__file__).resolve().parents[1]


class InspectionPoseTests(RegistryFixture):
    def measure(self,x=2.,y=1.8,yaw=0.,machine='pump',scale=1.):
        evidence = ProximityEvidence(MachineRegistry.load(self.root),machine)
        evidence.health(health(),10.)
        evidence.health(health(2,1.2),10.1)
        pose = transform(1100000000,x,y,yaw)
        pose.transform.rotation.z *= scale
        pose.transform.rotation.w *= scale
        return evidence.measure(1100000000,1200000000,10.11,pose)

    def test_all_three_nominal_poses_have_measured_position_and_heading(self):
        for machine,y in (('pump',1.8),('cooler',0.),('power_unit',-1.8)):
            value = self.measure(y=y,machine=machine)
            self.assertEqual(value['schema_version'],3)
            self.assertTrue(value['within_inspection_pose'])
            self.assertEqual(value['inspection_pose']['target_xy_yaw'],[2.,y,0.])
            self.assertEqual(value['inspection_pose']['xy_error_m'],0.)
            self.assertEqual(value['inspection_pose']['yaw_error_rad'],0.)

    def test_near_control_does_not_accept_opposite_heading_or_wrong_position(self):
        for x,y,yaw,reason in ((2.,1.8,math.pi,'heading'),(2.,2.,0.,'position'),
                               (2.8,1.8,math.pi,'position')):
            value = self.measure(x,y,yaw)
            self.assertTrue(value['valid'] and value['within_distance'])
            self.assertFalse(value['within_inspection_pose'])
            self.assertEqual(value['inspection_pose']['reason'],'outside_inspection_'+reason)

    def test_computed_position_and_yaw_bounds_are_inclusive_and_not_widened(self):
        for y,yaw,expected in ((.1,.15,True),(-.1,-.15,True),(.100001,0.,False),
                               (0.,.150001,False),(0.,-.150001,False)):
            with self.subTest(y=y,yaw=yaw):
                value = self.measure(y=y,yaw=yaw,machine='cooler')
                self.assertEqual(value['within_inspection_pose'],expected)
                self.assertEqual(value['inspection_pose']['xy_limit_m'],.1)
                self.assertEqual(value['inspection_pose']['yaw_limit_rad'],.15)

    def test_quaternion_sign_norm_and_angle_wrap_do_not_change_heading(self):
        for yaw in (.14,.14+2*math.pi,.14-2*math.pi):
            for scale in (1.,-1.,1.0001):
                with self.subTest(yaw=yaw,scale=scale):
                    value = self.measure(yaw=yaw,scale=scale)
                    self.assertTrue(value['within_inspection_pose'])
                    self.assertAlmostEqual(value['inspection_pose']['yaw_error_rad'],.14)

    def test_diagonal_nominal_heading_is_loaded_not_hardcoded_to_zero(self):
        yaw = math.atan2(-.2,.6)
        self.config['inspection_poses']['pump']['goal_xy_yaw'] = [2.,2.,yaw]
        self.save_config()
        value = self.measure(y=2.,yaw=yaw)
        self.assertTrue(value['within_inspection_pose'])
        self.assertEqual(value['inspection_pose']['target_xy_yaw'],[2.,2.,yaw])
        self.assertFalse(self.measure(y=2.,yaw=0.)['within_inspection_pose'])

    def test_distance_failure_is_distinct_from_pose_failure(self):
        value = self.measure(x=1.8)
        self.assertTrue(value['valid'])
        self.assertFalse(value['within_distance'] or value['within_inspection_pose'])
        self.assertEqual(value['inspection_pose']['reason'],'outside_control_distance')


class InspectionPoseAdapterTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)

    def test_actual_adapter_reports_wrong_pose_even_when_distance_is_valid(self):
        for pose,expected in (((2.,1.8,0.),True),((2.,1.8,math.pi),False),((2.,2.,0.),False)):
            harness = Harness(self)
            harness.pose = pose
            value = harness.run()
            self.assertTrue(value['within_distance'])
            self.assertEqual(value['within_inspection_pose'],expected)
            self.assertTrue(harness.destroyed and harness.shutdown)

    def test_shipped_cli_can_require_geometry_without_suppressing_valid_distance(self):
        spec = importlib.util.spec_from_file_location('inspection_pose_cli',ROOT/'scripts/machine-proximity.py')
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        for pose,required,exit_code in (((2.,1.8,math.pi),False,0),((2.,1.8,math.pi),True,1),
                                        ((2.,1.8,0.),True,0),((2.,2.,0.),True,1)):
            harness = Harness(self)
            harness.pose = pose
            output = io.StringIO()
            args = ['pump','--timeout','2']+(['--require-inspection-pose'] if required else [])
            def measured(root,machine_id,**kwargs):
                return ros_proximity.run(root,machine_id,now=lambda:harness.wall,**kwargs)
            with patch.dict(os.environ,{'ROS_DOMAIN_ID':'42','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}), \
                    patch.object(cli,'ROOT',self.root),patch.object(cli,'run',measured), \
                    contextlib.redirect_stdout(output):
                self.assertEqual(harness.run(invoke=lambda:cli.main(args)),exit_code)
            value = json.loads(output.getvalue())
            self.assertTrue(value['valid'] and value['within_distance'])
            self.assertTrue(harness.destroyed and harness.shutdown)
