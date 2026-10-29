"""Configured actual station, private truth and production ROS mission adapter."""
from types import MethodType
import copy

from orbital.inspection_owner import load_result

from mission_runtime_fixture import RuntimeFixture, RuntimeBus
from station_evaluation_fixture import StationFixture
from test_maintenance_reset import ResetFixture
import test_ros_mission as adapter


class MissionEvaluationFixture(RuntimeFixture):
    composed=adapter.RosMissionTests.composed
    release_initial_gateway=adapter.RosMissionTests.release_initial_gateway

    def configure(self, profile='recoverable', seed=0):
        self.f.journal.close();self.f.lock.close()
        self.f.load=MethodType(ResetFixture.load,self.f)
        StationFixture.attach(self.f,profile,seed)
        self.f.sample=MethodType(ResetFixture.sample,self.f)
        self.f.sample(.05);self.f.sample(.05)
        self.bus=RuntimeBus(self)
        # Generate and validate the actual capture bundle for maintenance too;
        # the original runtime fixture intentionally used a controlled loader.
        self.f.live.identity=self.bus.inspection.guard.identity
        self.f.live.current['health']=copy.deepcopy(self.bus.inspection.guard.current['health'])
        self.f.load=self.f.owner.load=load_result
        self.f.owner.worker=self.bus.capture
        return self.f.assignment['trial_id']

    def close_station(self):
        self.assertIsNone(self.f.owner.active)
        self.assertIsNone(self.f.owner.cancellation)
        self.f.owner.shutdown()
        StationFixture.close_station(self.f)
