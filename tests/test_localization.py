import math
import unittest
from orbital.localization import LocalizationHealth, TransformSample, planar_covariance_valid
from orbital.navigation import NavigationPolicy


class LocalizationHealthTests(unittest.TestCase):
    def covariance(self):
        cov=[0.]*36
        cov[0]=cov[7]=.01
        cov[35]=.02
        return cov

    def inputs(self, h, stamp=1., now=10., pose=True, skip=None):
        if skip != 'scan':h.scan(stamp=stamp,now=now,frame='lidar_link',ranges=[2.]*360,range_min=.1,range_max=12.)
        if skip != 'odom':h.odometry(stamp=stamp,now=now,parent='odom',child='base_footprint',position=(0.,0.,0.),
                   orientation=(0.,0.,0.,1.),linear=(0.,0.,0.),angular=(0.,0.,0.))
        if skip != 'imu':h.imu(stamp=stamp,now=now,frame='base_link',orientation=(0.,0.,0.,1.))
        if pose:h.pose(stamp=stamp,now=now,frame='map',position=(0.,0.,0.),orientation=(0.,0.,0.,1.),covariance=self.covariance())

    def evaluate(self, h, stamp=1.05, now=10.05, transform=True):
        tf=TransformSample(stamp-.05,'map','base_footprint',(0.,0.,0.),(0.,0.,0.,1.)) if transform else None
        return h.evaluate(now=now,sim_time=stamp,transform=tf)

    def test_requires_all_evidence_and_explicit_post_reset_pose(self):
        h=LocalizationHealth();self.assertFalse(self.evaluate(h).healthy)
        self.inputs(h,now=10.1);self.assertTrue(self.evaluate(h,now=10.15).healthy)
        h.reset();self.inputs(h,stamp=1.2,now=10.2,pose=False)
        self.assertEqual(self.evaluate(h,stamp=1.25,now=10.25).reason,'pose_invalid')

    def test_duplicates_cannot_hide_frozen_sensors(self):
        h=LocalizationHealth();self.inputs(h)
        self.inputs(h,now=10.49)
        self.assertEqual(self.evaluate(h,stamp=1.5,now=10.5).reason,'scan_stale')

    def test_old_pose_is_allowed_only_with_current_independent_inputs(self):
        h=LocalizationHealth();self.inputs(h)
        for i in range(1,191):self.inputs(h,stamp=1.+i*.1,now=10.+i*.1,pose=False)
        self.assertTrue(self.evaluate(h,stamp=20.05,now=29.05).healthy)
        self.assertEqual(self.evaluate(h,stamp=20.05,now=29.05,transform=False).reason,'transform_invalid')

    def test_tilt_invalidates_previously_active_target(self):
        h=LocalizationHealth();self.inputs(h);health=self.evaluate(h)
        p=NavigationPolicy(['pump']);p.observe(sim_time=health.sim_time,valid=health.healthy,now=10.05)
        target=p.request('pump',now=10.06);self.assertTrue(p.arm(target,stationary=True,now=10.07))
        pitch=.522876
        h.imu(stamp=1.1,now=10.1,frame='base_link',orientation=(0.,math.sin(pitch/2),0.,math.cos(pitch/2)))
        health=self.evaluate(h,stamp=1.15,now=10.15)
        self.assertEqual(health.reason,'imu_invalid')
        p.observe(sim_time=health.sim_time,valid=health.healthy,now=10.15)
        self.assertTrue(p.stop_required);self.assertEqual(p.cancel_pending,target)
        self.inputs(h,stamp=1.2,now=10.2)
        health=self.evaluate(h,stamp=1.25,now=10.25)
        p.observe(sim_time=health.sim_time,valid=health.healthy,now=10.25)
        self.assertFalse(p.arm(target,stationary=True,now=10.26))
        self.assertIsNone(p.request('pump',now=10.26))

    def test_reset_discards_old_epoch_evidence(self):
        h=LocalizationHealth();self.inputs(h);self.assertTrue(self.evaluate(h).healthy)
        self.assertEqual(self.evaluate(h,stamp=.05,now=10.1).reason,'simulation_reset')
        self.inputs(h,stamp=.1,now=10.2,pose=False)
        self.assertFalse(self.evaluate(h,stamp=.15,now=10.25).healthy)

    def test_reset_requires_explicit_seed_even_if_new_pose_arrives(self):
        h=LocalizationHealth();self.inputs(h);self.evaluate(h)
        self.assertEqual(self.evaluate(h,stamp=.05,now=10.1).reason,'simulation_reset')
        self.inputs(h,stamp=.1,now=10.2)
        self.assertEqual(self.evaluate(h,stamp=.15,now=10.25).reason,'initial_pose_required')
        h.replace_initial_pose(.2)
        self.inputs(h,stamp=.15,now=10.3)
        self.assertEqual(self.evaluate(h,stamp=.25,now=10.35).reason,'initial_pose_required')
        self.inputs(h,stamp=.3,now=10.4)
        self.assertEqual(self.evaluate(h,stamp=.35,now=10.45).reason,'pose_invalid')
        self.assertTrue(self.evaluate(h,stamp=.36,now=10.46).healthy)

    def test_each_stream_must_advance_independently(self):
        for missing in ('scan','odom','imu'):
            h=LocalizationHealth();self.inputs(h)
            for i in range(1,4):self.inputs(h,stamp=1.+i*.2,now=10.+i*.2,pose=False,skip=missing)
            self.assertEqual(self.evaluate(h,stamp=1.65,now=10.65).reason,missing+'_stale')

    def test_bad_sample_cannot_be_hidden_by_fresh_sample_before_timer(self):
        h=LocalizationHealth();self.inputs(h);self.evaluate(h)
        h.imu(stamp=1.1,now=10.1,frame='wrong',orientation=(0.,0.,0.,1.))
        self.inputs(h,stamp=1.2,now=10.2)
        self.assertEqual(self.evaluate(h,stamp=1.25,now=10.25).reason,'imu_invalid')
        self.assertTrue(self.evaluate(h,stamp=1.26,now=10.26).healthy)

    def test_late_fresh_samples_do_not_hide_expired_input_lease(self):
        h=LocalizationHealth();self.inputs(h);self.evaluate(h)
        self.inputs(h,stamp=1.6,now=10.6)
        self.assertEqual(self.evaluate(h,stamp=1.65,now=10.65).reason,'scan_stale')
        self.assertTrue(self.evaluate(h,stamp=1.66,now=10.66).healthy)

    def test_bad_covariance_cannot_look_confident(self):
        cov=self.covariance();self.assertTrue(planar_covariance_valid(cov))
        for mutation in ('nan','negative','asymmetric','non_psd','negative_determinant','large'):
            with self.subTest(mutation=mutation):
                cov=self.covariance()
                if mutation=='nan':cov[0]=float('nan')
                elif mutation=='negative':cov[7]=-.01
                elif mutation=='asymmetric':cov[1]=.001
                elif mutation=='non_psd':cov[1]=cov[6]=.02
                elif mutation=='negative_determinant':
                    cov[1]=cov[6]=-.009
                    cov[5]=cov[30]=cov[11]=cov[31]=-.9*math.sqrt(.01*.02)
                else:cov[0]=.1
                self.assertFalse(planar_covariance_valid(cov))

    def test_invalid_scan_and_wrong_frames_fail_closed(self):
        for frame,ranges in [('wrong',[2.]*360),('lidar_link',[math.inf]*360),
                             ('lidar_link',[float('nan')]*360),('lidar_link',[2.]*359)]:
            h=LocalizationHealth();self.inputs(h)
            h.scan(stamp=1.1,now=10.1,frame=frame,ranges=ranges,range_min=.1,range_max=12.)
            self.assertEqual(self.evaluate(h,stamp=1.15,now=10.15).reason,'scan_invalid')

    def test_transform_cannot_move_the_ground_frame_up_or_tilt_it(self):
        for position,q in [((0.,0.,.22),(0.,0.,0.,1.)),
                           ((0.,0.,0.),(0.,math.sin(.1),0.,math.cos(.1)))]:
            h=LocalizationHealth();self.inputs(h)
            tf=TransformSample(1.,'map','base_footprint',position,q)
            self.assertEqual(h.evaluate(now=10.05,sim_time=1.05,transform=tf).reason,'transform_invalid')

    def test_future_transform_and_backwards_wall_clock_rejected(self):
        h=LocalizationHealth();self.inputs(h)
        tf=TransformSample(2.,'map','base_footprint',(0.,0.,0.),(0.,0.,0.,1.))
        self.assertEqual(h.evaluate(now=10.05,sim_time=1.05,transform=tf).reason,'transform_invalid')
        self.assertEqual(self.evaluate(h,now=9.).reason,'wall_clock_invalid')


if __name__=='__main__':unittest.main()
