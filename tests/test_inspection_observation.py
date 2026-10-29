import json

from orbital.inspection_observation import InspectionEvidence,InspectionPending
from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.proximity import Unavailable
from test_machine_registry import RegistryFixture
from test_proximity import health,transform
from test_stationary import odom


class InspectionObservationTests(RegistryFixture):
    def setUp(self):
        super().setUp()
        self.registry=MachineRegistry.load(self.root)
        self.value=InspectionEvidence(self.registry,'pump')
        self.model=MachineTelemetry(self.registry)
        self.packet=None

    def feed(self,index):
        stamp=10**9+index*50000000;wall=10.+index*.05
        self.value.robot.health(health(index+1,stamp/1e9),wall)
        self.value.robot.odometry(odom(stamp),wall,stamp)
        if index%10==0:
            self.packet=self.model.sample(stamp)
            self.value.station.observe(json.dumps(self.packet),wall,stamp)
        return wall,stamp

    def ready(self):
        for index in range(11):
            wall,stamp=self.feed(index)
            try:self.value.select(wall,stamp)
            except InspectionPending:pass
        self.assertEqual(self.value.sample_stamp,1500000000)
        return self.value

    def test_record_uses_exact_sample_time_and_selected_machine_measurements(self):
        value=self.ready();result=value.measure(10.51,1510000000,transform(1500000000))
        self.assertEqual(result['sample_stamp_ns'],result['robot_state']['pose_stamp_ns'])
        self.assertEqual(result['telemetry_source']['stamp_ns'],result['sample_stamp_ns'])
        self.assertEqual(result['measurements'],{'voltage_v':self.packet['voltage_v'],**self.packet['machines']['pump']})
        self.assertEqual(result['telemetry_source']['time_s'],.5)
        self.assertTrue(value.robot.lease.pending_stop)

    def test_initial_state_and_one_packet_cannot_be_composed_into_a_record(self):
        self.feed(0)
        with self.assertRaises(InspectionPending):self.value.measure(10.01,1010000000,transform(10**9))
        with self.assertRaises(ValueError):InspectionEvidence(self.registry,'unknown')

    def test_same_time_transform_is_required_even_if_a_different_pose_is_fresh(self):
        value=self.ready()
        with self.assertRaisesRegex(Unavailable,'transform_time_or_frame'):
            value.measure(10.51,1510000000,transform(1410000000))

    def test_joint_pose_age_bound_is_tighter_than_standalone_telemetry_freshness(self):
        value=self.ready()
        for index in range(11,15):self.feed(index)
        result=value.measure(10.7,1700000000,transform(1500000000))
        self.assertTrue(result['valid'])
        self.assertTrue(value.station.measure(10.700001,1700000001)['valid'])
        with self.assertRaisesRegex(Unavailable,'inspection_sample_expired'):
            value.measure(10.700001,1700000001,transform(1500000000))

    def test_selected_public_sample_cannot_be_replaced_even_by_a_valid_newer_one(self):
        value=self.ready();packet=self.model.sample(1550000000)
        self.assertTrue(value.station.observe(json.dumps(packet),10.55,1550000000))
        with self.assertRaisesRegex(Unavailable,'inspection_telemetry_changed'):
            value.measure(10.56,1560000000,transform(1500000000))

    def test_localization_epoch_change_cannot_be_hidden_by_new_healthy_packets(self):
        value=self.ready()
        value.robot.health(health(12,1.55,epoch=1),10.55)
        value.robot.health(health(13,1.6,epoch=1),10.6)
        with self.assertRaisesRegex(Unavailable,'localization_generation_changed'):
            value.measure(10.61,1610000000,transform(1500000000))

    def test_record_mutation_cannot_change_the_frozen_sample_or_robot_evidence(self):
        value=self.ready();result=value.measure(10.51,1510000000,transform(1500000000))
        result['measurements']['voltage_v']=-1
        result['units']['current_a']='wrong'
        result['robot_state']['robot_pose_xyz_xyzw'][0]=99
        result['machine_binding']['control_xy_m'][0]=99
        later=value.measure(10.52,1520000000,transform(1500000000))
        self.assertEqual(later['measurements']['voltage_v'],self.packet['voltage_v'])
        self.assertEqual(later['units']['current_a'],'A')
        self.assertEqual(later['robot_state']['robot_pose_xyz_xyzw'][0],2.)
        self.assertEqual(later['machine_binding']['control_xy_m'][0],2.6)
