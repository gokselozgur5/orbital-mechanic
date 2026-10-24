import math
import unittest
from orbital.machines import Fault, Rejected, Station


class MachineTests(unittest.TestCase):
    def act(self, station, ident, operation, **kwargs):
        args = dict(distance_m=.5, speed_mps=0., observed_at_s=station.time_s)
        args.update(kwargs)
        return station.intervene(ident, operation, 'pump', **args)

    def test_repair_requires_restore_and_sustained_recovery(self):
        s = Station(Fault.LATCHED)
        self.assertLess(s.observe()['voltage_v'], 27)
        self.act(s, 'a', 'isolate'); self.act(s, 'b', 'reset')
        for _ in range(15): s.tick(1)
        self.assertFalse(s.recovered)
        self.act(s, 'c', 'restore')
        for _ in range(9): s.tick(1)
        self.assertFalse(s.recovered)
        s.tick(1)
        self.assertTrue(s.recovered)

    def test_reset_cannot_repair_mechanical_jam(self):
        s = Station(Fault.JAM)
        for key, op in [('a','isolate'),('b','reset'),('c','restore')]: self.act(s,key,op)
        for _ in range(15): s.tick(1)
        self.assertFalse(s.recovered)
        self.assertLess(s.voltage_v, 27)

    def test_live_reset_is_rejected_without_mutation(self):
        s = Station(Fault.LATCHED)
        with self.assertRaises(Rejected): self.act(s,'a','reset')
        self.assertEqual(s.machines['pump'].reset_count, 0)
        self.assertEqual(s.machines['pump'].fault, Fault.LATCHED)

    def test_duplicate_does_not_repeat_or_reset_recovery_clock(self):
        s = Station(Fault.LATCHED)
        self.act(s,'a','isolate'); result = self.act(s,'b','reset')
        s.tick(1)
        self.assertEqual(result,self.act(s,'b','reset',observed_at_s=0))
        self.assertEqual(s.machines['pump'].reset_count,1)
        with self.assertRaises(Rejected): self.act(s,'b','restore')

    def test_stationary_proximity_and_freshness(self):
        for kwargs in [dict(distance_m=1),dict(distance_m=-1),dict(speed_mps=.2),dict(distance_m=math.nan),dict(observed_at_s=1),dict(observed_at_s=-3)]:
            s = Station()
            with self.subTest(kwargs=kwargs), self.assertRaises(Rejected): self.act(s,'a','isolate',**kwargs)
            self.assertTrue(s.machines['pump'].powered)

    def test_overheated_machine_needs_cooling(self):
        s = Station(Fault.LATCHED)
        for _ in range(90): s.tick(1)
        self.act(s,'a','isolate')
        with self.assertRaises(Rejected): self.act(s,'b','reset')
        for _ in range(60): s.tick(1)
        self.act(s,'b','reset')

    def test_no_hidden_fault_in_observations(self):
        self.assertNotIn('fault',repr(Station(Fault.JAM).observe()))
        self.assertNotIn('jam',repr(Station(Fault.JAM).observe()))

    def test_invalid_time_does_not_advance(self):
        s = Station()
        for dt in [0,-1,math.inf,math.nan,2]:
            with self.assertRaises(Rejected): s.tick(dt)
        self.assertEqual(s.time_s,0)

if __name__ == '__main__': unittest.main()
