import unittest
import uuid
from orbital.health_lease import HealthLease


class HealthLeaseTests(unittest.TestCase):
    def report(self, **changes):
        data={'schema_version':1,'healthy':True,'stationary':True,'reason':'ready','sim_time':1.,
              'run_id':'4f96069c-1159-4a2c-9cb9-c65033c2a001','epoch':0,'sequence':1}
        return {**data,**changes}

    def active(self):
        lease=HealthLease();lease.observe(self.report(),10.)
        self.assertTrue(lease.stop_required)
        lease.acknowledge_stop(lease.generation)
        self.assertFalse(lease.stop_required)
        return lease

    def test_source_binding_requires_acknowledged_stop(self):
        lease=self.active()
        self.assertTrue(lease.tick(10.1))
        self.assertFalse(lease.tick(10.25));self.assertTrue(lease.stop_required)

    def test_late_health_cannot_hide_timeout_before_timer(self):
        lease=self.active()
        lease.observe(self.report(sequence=2,sim_time=1.3),10.3)
        self.assertTrue(lease.healthy);self.assertTrue(lease.pending_stop)

    def test_missed_unhealthy_report_still_invalidates_epoch(self):
        lease=self.active();old_generation=lease.generation
        lease.observe(self.report(sequence=3,epoch=1,sim_time=1.1),10.1)
        self.assertTrue(lease.healthy);self.assertTrue(lease.stop_required)
        self.assertFalse(lease.acknowledge_stop(old_generation))
        self.assertTrue(lease.stop_required)
        self.assertTrue(lease.acknowledge_stop(lease.generation))
        self.assertFalse(lease.stop_required)

    def test_replay_and_frozen_source_time_cannot_extend_deadline(self):
        lease=self.active();lease.observe(self.report(sequence=2),10.2)
        self.assertFalse(lease.tick(10.25))
        lease=self.active();lease.observe(self.report(),10.1)
        self.assertEqual(lease.reason,'replayed_health');self.assertTrue(lease.stop_required)

    def test_retired_run_cannot_replace_a_new_source(self):
        lease=self.active();new=str(uuid.uuid4())
        lease.observe(self.report(run_id=new),10.1)
        self.assertTrue(lease.stop_required)
        lease.acknowledge_stop(lease.generation)
        lease.observe(self.report(sequence=2,sim_time=1.2),10.2)
        self.assertEqual(lease.run_id,new);self.assertEqual(lease.reason,'retired_health_source')
        self.assertTrue(lease.stop_required)

    def test_stop_ack_race_cannot_clear_a_newer_invalidation(self):
        lease=self.active()
        lease.observe(self.report(sequence=2,healthy=False,reason='imu_invalid',sim_time=1.1),10.1)
        pending=lease.generation
        lease.observe(self.report(sequence=3,epoch=1,sim_time=1.2),10.2)
        self.assertFalse(lease.acknowledge_stop(pending));self.assertTrue(lease.stop_required)

    def test_bad_payload_epoch_and_wall_clock_stop(self):
        cases=[None,{},self.report(healthy=1),self.report(sequence=True),self.report(sim_time=float('nan')),
               self.report(run_id='x'),self.report(schema_version=2),self.report(sequence=2**63)]
        for report in cases:
            lease=self.active();self.assertFalse(lease.observe(report,10.1));self.assertTrue(lease.stop_required)
        lease=self.active()
        self.assertFalse(lease.observe(self.report(epoch=1,sequence=1,sim_time=1.1),10.1))
        self.assertEqual(lease.reason,'replayed_health')
        lease=self.active();lease.observe(self.report(epoch=2,sequence=2,sim_time=1.1),10.1)
        lease.acknowledge_stop(lease.generation)
        self.assertFalse(lease.observe(self.report(epoch=1,sequence=3,sim_time=1.2),10.2))
        self.assertEqual(lease.reason,'old_health_epoch')
        self.assertFalse(lease.tick(9.));self.assertTrue(lease.stop_required)


if __name__=='__main__':unittest.main()
