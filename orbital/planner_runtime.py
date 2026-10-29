"""Scripted baseline executed through the provider-neutral durable tool gateway."""
import copy
import threading
import uuid

from orbital.inspection_protocol import canonical
from orbital.mission_runtime import MissionRuntime
from orbital.planner_gateway import PlannerGateway
from orbital.planner_observation import PlannerObservation
from orbital.planner_schema import FRESH_TOOLS


class PlannerRuntime(MissionRuntime):
    execution_mode = 'scripted-tools-v1'

    def __init__(self, registry, transport, journal, **kwargs):
        mission = kwargs.get('mission_id') or uuid.uuid4().hex
        kwargs['mission_id'] = mission
        self.inputs = PlannerObservation(registry, run_id=uuid.uuid4().hex, mission_id=mission,
            now=transport.now, sim_ns=transport.sim_ns)
        self.gateway = None
        self.stop_signal = threading.Event()
        requested = kwargs.get('stop_requested', lambda:False)
        kwargs['stop_requested'] = lambda:self.stop_signal.is_set() or requested()
        super().__init__(registry, transport, journal, **kwargs)

    def _telemetry(self, text):
        # Capture receipt clocks before the superclass writes the mission event.
        self.inputs.observe(text)
        super()._telemetry(text)

    def _navigation_client(self):
        # Copy a real persistent receipt without renewing its receipt timestamp.
        if self.navigation.state is not None and not self.navigation.failure:
            client = copy.copy(self.navigation)
            client.state = copy.deepcopy(self.navigation.state)
            client.command, client.published = None, False
            return client
        return super()._navigation_client()

    def _before_command(self, family, command):
        if self.gateway is None:raise ValueError('planner_gateway_unavailable')
        self.gateway.bind(family, command)

    def _before_publish(self, family, command):
        if self.gateway is None:raise ValueError('planner_gateway_unavailable')
        self.gateway.before_publish(family, command)

    def _poll(self, duration=.02, *, cleanup_family=None):
        super()._poll(duration, cleanup_family=cleanup_family)
        if cleanup_family is None and self.gateway is not None:
            self.gateway.check_deadline()

    def start(self):
        super().start()
        self.gateway = PlannerGateway(self, self.inputs)

    def call(self, tool, arguments):
        observation = None
        if tool in FRESH_TOOLS:
            viewed = self.call('observe_station', {})
            if viewed['result']['status'] != 'succeeded':raise ValueError('planner_observation_unavailable')
            observation = viewed['result']['observation_ids'][0]
        return self.gateway.execute(canonical({'schema_version': 1, 'kind': 'planner_tool_request',
            'run_id': self.inputs.run_id, 'mission_id': self.mission_id, 'action_id': uuid.uuid4().hex,
            'tool': tool, 'observation_id': observation, 'arguments': arguments}))

    def drive(self):
        while self.mission.outcome is None:
            self._poll()
            if self.mission.stop_reason:break
            phase = self.mission.phase
            if phase == 'inspect':
                machine = self.policy.next_machine()['machine_id']
                self.call('navigate_to', {'waypoint_id': machine, 'timeout_s': 120.})
                if self.mission.phase == 'inspect':
                    self.call('inspect_machine', {'machine_id': machine, 'timeout_s': 5.})
            elif phase == 'intervene':
                tool = {'isolate': 'isolate_power', 'reset': 'reset_machine', 'restore': 'restore_power'}[self.mission.next_operation]
                self.call(tool, {'machine_id': self.mission.arrival, 'timeout_s': 30.})
            elif phase == 'verify':
                self.call('verify_recovery', {'machine_id': self.mission.arrival, 'observation_window_s': 10.})
            elif phase == 'escalate':break
            else:raise ValueError('planner_unresolved_transition')
        outcome = 'escalated' if self.mission.phase == 'escalate' else self.mission.outcome
        if outcome in ('healthy', 'recovered', 'escalated'):
            ids = list(self.gateway.state()['observations'])[-16:]
            response = self.call('report_outcome', {'outcome': outcome, 'evidence_ids': ids,
                'summary': 'Scripted public-evidence baseline outcome; independent evaluation remains separate.'})
            if response['result']['status'] != 'succeeded':raise ValueError('planner_claim_unavailable')

    def run(self):
        report = super().run()
        report['planner'] = {'run_id': self.inputs.run_id, 'mode': self.execution_mode,
            'stop_delivery_failed': False if self.gateway is None else self.gateway.stop_delivery_failed,
            'calls': None if self.gateway is None else self.journal.tools.get(self.mission_id),
            'scope': 'Owned tool execution constrained by the scripted public diagnosis policy; no native acceptance.'}
        return report
