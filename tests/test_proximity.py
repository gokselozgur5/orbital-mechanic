import copy
import json
import math
from types import SimpleNamespace as NS

from orbital.machine_registry import MachineRegistry
from orbital.proximity import ProximityEvidence, Unavailable
from test_machine_registry import RegistryFixture


def health(sequence=1, sim_time=1., **changes):
    return json.dumps({'schema_version':1,'healthy':True,'stationary':True,'reason':'ready',
                       'run_id':'4f96069c-1159-4a2c-9cb9-c65033c2a001','epoch':0,
                       'sequence':sequence,'sim_time':sim_time,**changes})


def transform(stamp, x=2., y=1.8):
    return NS(header=NS(frame_id='map',stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9)),
              child_frame_id='base_footprint',transform=NS(translation=NS(x=x,y=y,z=0.),
                                                       rotation=NS(x=0.,y=0.,z=0.,w=1.)))


class ProximityTests(RegistryFixture):
    def ready(self, name='pump'):
        value = ProximityEvidence(MachineRegistry.load(self.root),name)
        value.health(health(),10.)
        value.health(health(2,1.2),10.1)
        return value

    def test_all_machine_distances_use_their_registered_control(self):
        for name,y in (('pump',1.8),('cooler',0.),('power_unit',-1.8)):
            evidence = self.ready(name)
            value = evidence.measure(1100000000,1200000000,10.11,transform(1100000000,y=y))
            self.assertAlmostEqual(value['distance_m'],.6)
            self.assertTrue(value['within_distance'])
            self.assertEqual(value['machine_id'],name)
            self.assertEqual(value['registry_identity'],evidence.registry.identity)

    def test_distance_boundary_is_inclusive_and_outside_is_not_near(self):
        for x,near in ((1.85,True),(1.85-1e-6,False)):
            value = self.ready().measure(1100000000,1200000000,10.11,transform(1100000000,x=x))
            self.assertEqual(value['within_distance'],near)

    def test_wrong_frame_stamp_or_nonplanar_pose_cannot_supply_distance(self):
        original = transform(1100000000)
        variants = []
        for change in ('stamp','parent','child','z','quaternion','tilt','nan','boolean'):
            pose = copy.deepcopy(original)
            if change == 'stamp': pose.header.stamp.nanosec += 1
            elif change == 'parent': pose.header.frame_id = 'odom'
            elif change == 'child': pose.child_frame_id = 'base_link'
            elif change == 'z': pose.transform.translation.z = .05
            elif change == 'quaternion': pose.transform.rotation.w = 2.
            elif change == 'tilt':
                pose.transform.rotation.x,pose.transform.rotation.w = math.sin(.1/2),math.cos(.1/2)
            elif change == 'nan': pose.transform.translation.x = float('nan')
            else: pose.transform.translation.x = True
            variants.append((change,pose))
        for change,pose in variants:
            with self.subTest(change=change),self.assertRaises(Unavailable):
                self.ready().measure(1100000000,1200000000,10.11,pose)

    def test_missing_health_or_pre_epoch_transform_is_unavailable(self):
        evidence = ProximityEvidence(MachineRegistry.load(self.root),'pump')
        with self.assertRaises(Unavailable): evidence.requested(1200000000)
        evidence.health(health(sim_time=1.2),10.)
        with self.assertRaises(Unavailable): evidence.requested(1200000000)
        self.assertEqual(evidence.requested(1300000000),1200000000)

    def test_stale_receipt_and_health_simulation_time_are_rejected(self):
        for now,sim in ((10.35,1200000000),(10.11,1500000000),(10.11,1190000000)):
            with self.subTest(now=now,sim=sim),self.assertRaises(Unavailable):
                self.ready().measure(1100000000,sim,now,transform(1100000000))

    def test_duplicate_and_frozen_health_cannot_renew_a_measurement(self):
        replay = self.ready()
        replay.health(health(2,1.2),10.2)
        with self.assertRaises(Unavailable): replay.measure(1100000000,1200000000,10.21,transform(1100000000))
        frozen = self.ready()
        frozen.health(health(3,1.2),10.3)
        with self.assertRaises(Unavailable): frozen.measure(1100000000,1200000000,10.35,transform(1100000000))

    def test_new_epoch_or_run_invalidates_this_query_even_if_healthy(self):
        for change in ({'epoch':1},{'run_id':'4f96069c-1159-4a2c-9cb9-c65033c2a002'}):
            evidence = self.ready()
            evidence.health(health(3,1.3,**change),10.2)
            with self.subTest(change=change),self.assertRaisesRegex(Unavailable,'generation_changed'):
                evidence.requested(1400000000)

    def test_clock_regression_cannot_be_hidden_by_later_health(self):
        evidence = self.ready()
        evidence.requested(1200000000)
        with self.assertRaisesRegex(Unavailable,'regressed'): evidence.requested(1100000000)
        evidence.health(health(3,1.4),10.2)
        with self.assertRaisesRegex(Unavailable,'regressed'): evidence.requested(1400000000)

    def test_malformed_health_is_not_replaced_by_a_later_good_packet(self):
        for text in ('{}','[]','x'*4097,health().replace('"healthy": true','"healthy": true, "healthy": true'),health(sim_time=1e300)):
            evidence = self.ready()
            evidence.health(text,10.2)
            evidence.health(health(4,1.3),10.21)
            with self.subTest(text=text[:30]),self.assertRaises(Unavailable): evidence.requested(1400000000)

    def test_unhealthy_or_regressed_wall_clock_does_not_authorize_distance(self):
        unhealthy = self.ready()
        unhealthy.health(health(3,1.3,healthy=False,reason='imu_invalid'),10.2)
        with self.assertRaises(Unavailable): unhealthy.measure(1200000000,1300000000,10.21,transform(1200000000))
        with self.assertRaises(Unavailable): self.ready().measure(1100000000,1200000000,9.,transform(1100000000))

    def test_read_only_measurement_never_acknowledges_a_motion_stop(self):
        evidence = self.ready()
        value = evidence.measure(1100000000,1200000000,10.11,transform(1100000000))
        self.assertTrue(value['valid'])
        self.assertTrue(evidence.lease.pending_stop)
        self.assertIn('not inspection',value['scope'])

    def test_requested_timestamp_must_be_an_integer_not_coerced(self):
        for stamp in (1100000000.,True,-1,2**63):
            with self.subTest(stamp=stamp),self.assertRaises(Unavailable):
                self.ready().measure(stamp,1200000000,10.11,transform(1100000000))
