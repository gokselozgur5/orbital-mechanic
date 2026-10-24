"""Single-operator, run-bound command sequencing for the local ROS gateway."""
import json


class CommandGate:
    def __init__(self, run_id, waypoints):
        self.run_id=run_id
        self.waypoints=frozenset(waypoints)
        self.sequence=0

    def decode(self, text):
        if not isinstance(text,str) or len(text)>4096:raise ValueError('command too large or not text')
        value=json.loads(text)
        if not isinstance(value,dict):raise ValueError('command must be an object')
        op=value.get('op')
        fields={'run_id','sequence','op','waypoint' if op=='request' else 'target'}
        if op not in ('request','arm','cancel') or set(value)!=fields:raise ValueError('unsupported command schema')
        if value['run_id']!=self.run_id:raise ValueError('wrong supervisor run')
        seq=value['sequence']
        if type(seq) is not int or not 0<seq<2**63 or seq!=self.sequence+1:raise ValueError('replayed or out-of-order command')
        if op=='request':
            if not isinstance(value['waypoint'],str) or value['waypoint'] not in self.waypoints:raise ValueError('unknown waypoint')
        else:
            t=value['target']
            if not isinstance(t,dict) or set(t)!={'run_id','epoch','sequence','waypoint'}:raise ValueError('target identity required')
            if t['run_id']!=self.run_id or not isinstance(t['waypoint'],str) or t['waypoint'] not in self.waypoints:raise ValueError('wrong target run or waypoint')
            if any(type(t[k]) is not int or not 0<=t[k]<2**63 for k in ('epoch','sequence')) or t['sequence']==0:raise ValueError('invalid target identity')
        self.sequence=seq
        return value
