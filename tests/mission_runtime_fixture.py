"""Persistent gateway with actual captures/owners over controlled ROS peers."""
import copy
import hashlib
import json
from types import SimpleNamespace as NS

from orbital import ros_proximity
from orbital.camera_observation import CAMERA_TOPICS
from orbital.inspection_owner import InspectionOwner
from orbital.inspection_protocol import canonical
from orbital.inspection_worker import OwnerLock
from orbital.mission_journal import MissionJournal
from orbital.mission_runtime import MissionRuntime
from orbital.navigation_command import CommandGate
from mission_policy_fixture import PolicyFixture
from proximity_fixture import Harness
from test_maintenance_reset import ResetFixture
import test_inspection_owner as inspection
import test_navigation_client as navigation


class RuntimeBus:
    def __init__(self, case):
        self.case,self.f,self.clock=case,case.f,case.clock
        self.callbacks={}
        self.sent=[]
        self.before_publish=self.after_publish=self.after_spin=None
        self.broken_sources=set()
        self.blocked_navigation=self.drop_navigation=self.hold_navigation=False
        self.drop_telemetry=False
        self.closed=False
        self.gate=CommandGate(navigation.RUN,('pump','cooler','power_unit'))
        self.nav=navigation.state(sim_stamp_ns=self.clock.sim)
        self.navigation_ticks=0
        self.navigation_epoch=0
        initial=self.harness('pump')
        sources={**ros_proximity.proximity_sources(),**CAMERA_TOPICS,
                 '/station/telemetry':[('station_telemetry','std_msgs/msg/String')]}
        identity=hashlib.sha256(json.dumps(ros_proximity.authority(initial.modules['rclpy'].create_node(),sources),
            separators=(',',':')).encode()).hexdigest()
        guard=inspection.Guard()
        guard.current={'health':{'run_id':'4f96069c-1159-4a2c-9cb9-c65033c2a001','epoch':0}}
        guard.identity=lambda:{'registry_identity':self.f.model.registry.identity,'authority_identity':identity}
        self.inspection=InspectionOwner(self.f.root,guard,self.capture,now=self.now,sim_ns=self.sim_ns,
            on_result=lambda value:self.deliver('/inspection/result',canonical(value)))
        old_result=self.f.owner.on_result
        def maintenance_result(value):
            old_result(value);self.deliver('/maintenance/result',canonical(value))
        self.f.owner.on_result=maintenance_result
        old_load=self.f.load
        def load(root,reference,action):
            value=old_load(root,reference,action)
            value['machine_id']=action.machine
            value['measurements']={'voltage_v':self.f.packet['voltage_v'],**self.f.packet['machines'][action.machine]}
            return value
        self.f.load=load
        self.f.owner.load=load

    def now(self):return self.clock.wall
    def sim_ns(self):return self.clock.sim
    def install(self, callbacks):self.callbacks=callbacks
    def fence(self, family=None):
        if self.broken_sources and (family is None or family in self.broken_sources):
            raise ValueError('controlled_gateway_source_change')
        return {'family':family,'identity':'controlled_persistent_transport'}
    def close(self):self.closed=True;return True
    def deliver(self, topic, text):
        if topic in self.callbacks:self.callbacks[topic](text)

    def harness(self, machine):
        c=self.case
        h=Harness(NS(root=self.f.root,directory=c.prepared.yaml_path.parent,payload=c.prepared.bundle()['grid']))
        h.pose=tuple(self.f.model.registry.public()['machines'][machine]['inspection_xy_yaw'])
        h.wall,h.sim=self.clock.wall,self.clock.sim
        h.next_telemetry=h.sim+700000000;h.next_camera=h.sim+200000000
        h.telemetry_model=self.f.model
        return h

    def capture(self, payload):
        h=self.harness(payload['machine_id'])
        def clock():self.clock.wall,self.clock.sim=h.wall,h.sim
        def telemetry(packet):
            self.f.packet=packet
            self.f.owner.telemetry(canonical(packet))
            self.deliver('/station/telemetry',canonical(packet))
            return packet
        h.spin_hook,h.telemetry_hook=clock,telemetry
        record=h.run(invoke=lambda:ros_proximity.capture_machine_observation(self.f.root,payload['machine_id'],
            timeout=payload['timeout_s'],now=self.now,action_context=payload['context']))
        self.case.capture_records.append(record)
        worker=inspection.Worker(payload)
        worker.reply={'status':'succeeded','observation_id':record['observation_id'],
                      'manifest_sha256':record['capture_artifact']['manifest_sha256']}
        worker.exited=True
        return worker

    def publish(self, family, text):
        request=json.loads(text)
        if self.before_publish:self.before_publish(family,request)
        self.sent.append((family,copy.deepcopy(request)))
        if family=='navigation':
            command=self.gate.decode(text);self.gate.sequence=command['sequence']
            self.nav['command_sequence']=command['sequence']
            self.nav['last_reply']={'sequence':command['sequence'],'op':command['op'],'accepted':True}
            if command['op']=='request':
                self.navigation_epoch+=1
                self.nav.update(target={'run_id':navigation.RUN,'epoch':self.navigation_epoch,
                    'sequence':command['sequence'],'waypoint':command['waypoint']},action_state='waiting_for_arm')
            elif command['op']=='arm':
                self.nav.update(action_state='running',guard_latched=False)
                self.navigation_ticks=2
            elif command['op']=='cancel':
                self.nav['action_state']='cancelling';self.navigation_ticks=2
        else:
            owner=self.inspection if family=='inspection' else self.f.owner
            self.deliver('/'+family+'/reply',canonical(owner.handle(text)))
        if self.after_publish:self.after_publish(family,request)

    def spin(self, duration):
        self.f.hold.acknowledged=True
        if self.f.hold.release_at is not None:self.f.hold.released=True
        for worker in self.f.workers:worker.exited=True
        # Publish the sampled packet before ticking an owner whose controlled
        # capture may synchronously advance the station and publish newer ones.
        # This matches the production timer/publication-before-owner-tick order.
        self.clock.advance(max(duration,.001))
        packet=self.f.model.sample(self.clock.sim)
        self.f.packet=packet
        self.f.owner.telemetry(canonical(packet))
        if not self.drop_telemetry:self.deliver('/station/telemetry',canonical(packet))
        self.f.owner.tick()
        self.inspection.tick()
        self.deliver('/inspection/state',canonical(self.inspection.state()))
        self.deliver('/maintenance/state',canonical(self.f.owner.state()))
        if self.navigation_ticks and (not self.hold_navigation or self.nav['action_state']=='cancelling'):
            self.navigation_ticks-=1
            if not self.navigation_ticks:
                cancelled=self.nav['action_state']=='cancelling'
                self.nav.update(last_completion={'target':self.nav['target'],
                    'outcome':'cancelled' if cancelled else 'aborted' if self.blocked_navigation else 'arrived',
                    'measured':{'within_tolerance':not (cancelled or self.blocked_navigation)}},
                    target=None,action_state='idle',guard_latched=True)
        self.nav['sim_stamp_ns']=self.clock.sim
        if not self.drop_navigation:self.deliver('/navigation/state',canonical(self.nav))
        if self.after_spin:self.after_spin()


class RuntimeFixture(PolicyFixture):
    def setUp(self):
        super().setUp()
        self.bus=RuntimeBus(self)
        self.mission_lock=OwnerLock(self.f.root,service='mission-service')
        self.addCleanup(self.mission_lock.close)
        self.mission_journal=MissionJournal(self.mission_lock)
        self.addCleanup(self.mission_journal.close)
        self.cancel_requested=False
        self.driver=MissionRuntime(self.f.model.registry,self.bus,self.mission_journal,
            mission_id=self.f.request['mission_id'],timeout_s=60.,stop_requested=lambda:self.cancel_requested)

    def run_mission(self):
        self.report=self.driver.run()
        return self.report
