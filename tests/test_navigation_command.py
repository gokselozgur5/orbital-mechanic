import json
import unittest
from orbital.navigation_command import CommandGate


class NavigationCommandTests(unittest.TestCase):
    def test_run_sequence_and_exact_target_are_required(self):
        gate=CommandGate('run',['pump'])
        request={'run_id':'run','sequence':1,'op':'request','waypoint':'pump'}
        self.assertEqual(gate.decode(json.dumps(request)),request)
        for bad in [request,{**request,'sequence':3},{**request,'sequence':True},
                    {**request,'sequence':2,'run_id':'old'},{**request,'sequence':2,'waypoint':'other'}]:
            with self.assertRaises(ValueError):gate.decode(json.dumps(bad))
        target={'run_id':'run','epoch':0,'sequence':1,'waypoint':'pump'}
        arm={'run_id':'run','sequence':2,'op':'arm','target':target}
        self.assertEqual(gate.decode(json.dumps(arm)),arm)
        with self.assertRaises(ValueError):gate.decode(json.dumps({**arm,'sequence':3,'target':{**target,'sequence':True}}))
        self.assertEqual(gate.sequence,2)

    def test_malformed_and_extra_fields_do_not_consume_sequence(self):
        gate=CommandGate('run',['pump'])
        for raw in ['[]','null','{','x'*4097,json.dumps({'run_id':'run','sequence':1,'op':'request','waypoint':'pump','pose':[0,0]})]:
            with self.assertRaises(ValueError):gate.decode(raw)
        self.assertEqual(gate.sequence,0)


if __name__=='__main__':unittest.main()
