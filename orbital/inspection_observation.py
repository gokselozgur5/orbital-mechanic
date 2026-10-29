"""Bind one public machine sample to same-time localized, settled robot evidence."""
import hashlib

from orbital.proximity import ProximityEvidence,Unavailable,MAX_POSE_AGE_NS
from orbital.telemetry_observation import TelemetryEvidence


class InspectionPending(Unavailable):
    """The initial settling/sample prerequisites may still arrive within the query."""


class InspectionEvidence:
    """Single-executor observation state, never a merger of caller-supplied records."""
    def __init__(self, registry, machine_id):
        self.registry = registry
        self.robot = ProximityEvidence(registry,machine_id)
        self.station = TelemetryEvidence(registry)
        self.settled_generation = self.settled_stamp = None
        self.selected = self.sample_stamp = self.sample_digest = None
        self.failure = None

    def fail(self, code):
        self.failure = self.failure or code
        raise Unavailable(self.failure)

    def settled(self, now, sim_ns):
        if self.failure:
            raise Unavailable(self.failure)
        self.station.tick(now,sim_ns)
        if self.robot.error:
            self.fail(self.robot.error)
        try:
            state = self.robot.settled(now,sim_ns)
        except Unavailable:
            if self.robot.error:
                self.fail(self.robot.error)
            if self.settled_generation is not None:
                self.fail('inspection_robot_state_lost')
            raise InspectionPending('awaiting_settled_robot') from None
        if self.settled_generation is not None and (state['generation']!=self.settled_generation or not state['settled']):
            self.fail('inspection_stationarity_changed')
        if not state['settled']:
            raise InspectionPending('awaiting_settled_robot')
        if self.settled_generation is None:
            self.settled_generation,self.settled_stamp = state['generation'],sim_ns

    def select(self, now, sim_ns):
        """Wait for a fresh pair and a sample produced after initial settling."""
        self.settled(now,sim_ns)
        if self.selected is not None:
            self.check(now,sim_ns)
            return self.sample_stamp
        try:
            observation = self.station.measure(now,sim_ns)
        except Unavailable:
            if self.station.failure:
                self.fail(self.station.failure)
            raise InspectionPending('awaiting_fresh_inspection_telemetry') from None
        stamp = observation['telemetry']['stamp_ns']
        if stamp<self.settled_stamp or sim_ns-stamp>MAX_POSE_AGE_NS:
            raise InspectionPending('awaiting_post_settling_sample')
        self.selected = self.station.candidate
        self.sample_stamp = stamp
        self.sample_digest = hashlib.sha256(self.selected.encode()).hexdigest()
        return stamp

    def check(self, now, sim_ns):
        self.settled(now,sim_ns)
        if self.selected is None:
            raise InspectionPending('awaiting_selected_sample')
        if self.station.candidate!=self.selected:
            self.fail('inspection_telemetry_changed')
        if not 0<=sim_ns-self.sample_stamp<=MAX_POSE_AGE_NS:
            self.fail('inspection_sample_expired')

    def measure_robot(self, stamp, now, sim_ns, transform):
        robot = self.robot.measure(stamp,sim_ns,now,transform)
        if robot['within_inspection_pose'] and robot['stationarity']['reason']=='awaiting_time_matched_odometry':
            raise InspectionPending('awaiting_sample_time_odometry')
        if not robot['within_settled_inspection_pose']:
            self.fail('inspection_pose_required')
        return robot

    def measure(self, now, sim_ns, transform):
        self.check(now,sim_ns)
        observation = self.station.measure(now,sim_ns)
        robot = self.measure_robot(self.sample_stamp,now,sim_ns,transform)
        sample = observation['telemetry']
        machine_id = self.robot.machine.machine_id
        return {'schema_version':1,'kind':'machine_inspection_observation','valid':True,
                'machine_id':machine_id,'registry_identity':self.registry.identity,
                'source_sha256':dict(sample['registry']['source_sha256']),
                'machine_binding':self.robot.machine.public(),
                'sample_stamp_ns':self.sample_stamp,'observed_sim_stamp_ns':sim_ns,
                'settled_at_sim_stamp_ns':self.settled_stamp,
                'measurements':{'voltage_v':sample['voltage_v'],**sample['machines'][machine_id]},
                'units':dict(sample['units']),
                'telemetry_source':{**{k:sample[k] for k in ('schema_version','source','run_id','sequence','stamp_ns','time_s')},
                                    **({'control_revision': sample['control']['revision']} if sample['schema_version']==2 else {}),
                                    'sample_sha256':self.sample_digest},
                'telemetry_freshness':observation['freshness'],'robot_state':robot,
                'scope':'same-time local control pose and public model telemetry with sampled settling; '
                        'not camera evidence, diagnosis, a hold or maintenance authorization'}
