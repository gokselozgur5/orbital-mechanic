"""Actual capture adapter/client/owner with controlled ROS peers and station."""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
from types import SimpleNamespace as NS
import uuid

from orbital import ros_proximity
from orbital.camera_observation import CAMERA_TOPICS
from orbital.inspection_client import InspectionClient
from orbital.inspection_owner import InspectionOwner
from orbital.inspection_protocol import canonical
from orbital.map_binding import GENERATOR_SOURCES
from orbital.map_snapshot import MapSnapshot
from orbital.mission_policy import MissionPolicy
from orbital.mission_state import MissionState
from proximity_fixture import Harness
from test_mission_state import MissionFixture
from test_maintenance_reset import ResetFixture
from test_maintenance_journal import JournalFixture
import test_inspection_owner as inspection
import test_navigation_client as navigation

ROOT=Path(__file__).resolve().parents[1]


class PolicyFixture(MissionFixture):
    def setUp(self):
        self.f=JournalFixture()
        self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.clock=self.f.clock
        self.f.sample(.05);self.baseline=self.f.sample(.05)
        self.m=MissionState(self.f.model.registry,self.f.request['mission_id'],self.baseline,
            now=lambda:self.clock.wall,sim_ns=lambda:self.clock.sim)
        self.policy=MissionPolicy(self.m)
        self.public_change=lambda packet:packet
        for relative in (*GENERATOR_SOURCES,'config/robot.urdf','config/bridge.yaml'):
            target=self.f.root/relative
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(ROOT/relative,target)
        spec=importlib.util.spec_from_file_location('policy_map_generator',self.f.root/'scripts/generate-known-map.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with contextlib.redirect_stdout(io.StringIO()):module.generate()
        self.prepared=MapSnapshot.prepare(self.f.model.registry)
        self.initial_events=[]
        def sample(delta=.05):
            packet=ResetFixture.sample(self.f,delta)
            self.policy.observe_telemetry(canonical(packet))
            return packet
        self.f.sample=sample
        self.nav_sequence=0
        for _ in range(2):
            packet=self.f.sample(.05)
            self.initial_events.append({'text':canonical(packet),'wall_s':self.clock.wall,'sim_ns':self.clock.sim})
        self.capture_records=[]
        self.inspection_owner=None

    def navigate_to(self,machine,*,blocked=False):
        before=self.nav_sequence
        self.nav_sequence+=1
        token={'run_id':navigation.RUN,'epoch':self.nav_sequence,'sequence':self.nav_sequence,'waypoint':machine}
        request={'run_id':navigation.RUN,'sequence':self.nav_sequence,'op':'request','waypoint':machine}
        response=navigation.state(command_sequence=self.nav_sequence,target=token,action_state='waiting_for_arm',
            last_reply={'sequence':self.nav_sequence,'op':'request','accepted':True})
        channel=navigation.Channel(navigation.state(command_sequence=before),response)
        self.m.dispatch(request,lambda _:channel.execute(waypoint=machine)[0])
        self.nav_sequence+=1
        command={'run_id':navigation.RUN,'sequence':self.nav_sequence,'op':'arm','target':token}
        reply=navigation.state(command_sequence=self.nav_sequence,target=token,action_state='waiting_for_arm',
            last_reply={'sequence':self.nav_sequence,'op':'arm','accepted':True})
        arm=navigation.Channel(response,reply)
        self.m.dispatch_arm(command,lambda _:arm.execute('arm',target=token)[0])
        self.clock.advance(.02)
        self.assertTrue(self.m.navigation_state(navigation.state(command_sequence=self.nav_sequence,
            sim_stamp_ns=self.clock.sim,last_completion={'target':token,
                'outcome':'aborted' if blocked else 'arrived','measured':{'within_tolerance':not blocked}}),self.clock.wall))

    def actual_inspect(self,machine):
        case=NS(root=self.f.root,directory=self.prepared.yaml_path.parent,payload=self.prepared.bundle()['grid'])
        h=Harness(case);h.pose=tuple(self.f.model.registry.public()['machines'][machine]['inspection_xy_yaw'])
        h.wall,h.sim=self.clock.wall,self.clock.sim
        h.next_telemetry=h.sim+700000000;h.next_camera=h.sim+200000000
        h.telemetry_model=self.f.model
        def clocks():
            self.clock.wall,self.clock.sim=h.wall,h.sim
        def telemetry(packet):
            packet=self.public_change(packet)
            self.f.packet=packet
            self.f.owner.telemetry(canonical(packet))
            self.policy.observe_telemetry(canonical(packet))
            return packet
        h.spin_hook,h.telemetry_hook=clocks,telemetry
        sources={**ros_proximity.proximity_sources(),**CAMERA_TOPICS,
                 '/station/telemetry':[('station_telemetry','std_msgs/msg/String')]}
        identity=hashlib.sha256(json.dumps(ros_proximity.authority(h.modules['rclpy'].create_node(),sources),
                                          separators=(',',':')).encode()).hexdigest()
        guard=inspection.Guard()
        guard.current={'health':{'run_id':'4f96069c-1159-4a2c-9cb9-c65033c2a001','epoch':0}}
        guard.identity=lambda:{'registry_identity':self.m.registry.identity,'authority_identity':identity}
        children=[]
        def worker(payload):
            record=h.run(invoke=lambda:ros_proximity.capture_machine_observation(self.f.root,machine,
                timeout=payload['timeout_s'],now=lambda:self.clock.wall,action_context=payload['context']))
            self.capture_records.append(record)
            value=inspection.Worker(payload)
            value.reply={'status':'succeeded','observation_id':record['observation_id'],
                         'manifest_sha256':record['capture_artifact']['manifest_sha256']}
            value.exited=True;children.append(value)
            return value
        self.capture_factory=worker
        if self.inspection_owner is None:
            self.inspection_owner=InspectionOwner(self.f.root,guard,lambda payload:self.capture_factory(payload),
                now=lambda:self.clock.wall,sim_ns=lambda:self.clock.sim)
        else:
            self.assertEqual(self.inspection_owner.guard.identity(),guard.identity())
        owner=self.inspection_owner
        def publish(text):client.observe_reply(canonical(owner.handle(text)))
        def spin(_):
            owner.tick()
            client.observe_state(canonical(owner.state()))
        client=InspectionClient(publish,spin,lambda:{'owner':'inspection'},now=lambda:self.clock.wall)
        client.discover()
        request={'schema_version':1,'run_id':owner.run_id,'mission_id':self.m.mission_id,
            'action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex,'op':'inspect',
            'arguments':{'machine_id':machine,'timeout_s':5.}}
        receipt=self.m.dispatch(request,client.execute)
        self.assertEqual(receipt['transport'],'terminal',receipt)
        self.assertEqual(receipt['result']['status'],'succeeded',receipt)
        self.assertEqual(len(children),1)
        self.assertTrue(h.destroyed and h.unregistered and h.shutdown)
        return receipt

    def capture_step(self):
        plan=self.policy.next_machine()
        self.navigate_to(plan['machine_id'])
        return self.actual_inspect(plan['machine_id'])

    def inspection_step(self):
        self.capture_step()
        return self.policy.apply()
