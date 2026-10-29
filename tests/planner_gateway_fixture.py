"""Actual clients/owners/causal model, with controlled transport and robot peers."""
import uuid

from orbital.inspection_protocol import canonical
from orbital.planner_runtime import PlannerRuntime
from mission_runtime_fixture import RuntimeFixture


class GatewayFixture(RuntimeFixture):
    def setUp(self):
        super().setUp()
        self.driver = PlannerRuntime(self.f.model.registry, self.bus, self.mission_journal,
            mission_id=self.f.request['mission_id'], timeout_s=60., stop_requested=lambda:self.cancel_requested)

    def request(self, tool, arguments, *, action_id=None, observation_id=None):
        return {'schema_version':1, 'kind':'planner_tool_request', 'run_id':self.driver.inputs.run_id,
            'mission_id':self.driver.mission_id, 'action_id':action_id or uuid.uuid4().hex,
            'tool':tool, 'arguments':arguments, 'observation_id':observation_id}

    def observe(self):
        value=self.request('observe_station',{})
        return self.driver.gateway.execute(canonical(value))

    def command(self, tool, arguments):
        view=self.observe()
        return self.request(tool,arguments,observation_id=view['result']['observation_ids'][0])

    def diagnose(self):
        self.driver.start()
        self.driver.call('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        self.driver.call('inspect_machine',{'machine_id':'pump','timeout_s':5.})
        self.assertEqual(self.driver.mission.phase,'intervene')

    def isolated(self):
        self.diagnose()
        self.driver.call('isolate_power',{'machine_id':'pump','timeout_s':5.})
        self.assertEqual(self.f.model.control_revision,1)
