"""Public telemetry projection and current-observation request preconditions.

The host supplies the registry and receipt clocks. This object cannot publish a
command, read camera files or access private scenario/evaluator state. Same-user
Python code is not an OS security boundary, and preflight reserves no action.
"""
import hashlib
import uuid

from orbital.inspection_protocol import canonical, identifier
from orbital.machines import MACHINE_IDS
from orbital.planner_protocol import PlannerError, request
from orbital.planner_schema import FRESH_TOOLS
from orbital.proximity import Unavailable
from orbital.telemetry_observation import TelemetryEvidence


class PlannerObservation:
    def __init__(self, registry, *, run_id, mission_id, now, sim_ns):
        self.registry = registry
        self.run_id, self.mission_id = identifier(run_id), identifier(mission_id)
        self.now, self.sim_ns = now, sim_ns
        self.reader = TelemetryEvidence(registry)
        self.last_digest = self.observation_id = None
        self.failure = False

    def observe(self, text):
        """Invoke at the real public callback receipt, before other callback work."""
        if self.failure:
            return False
        return self.reader.observe(text, self.now(), self.sim_ns())

    def view(self):
        try:
            if self.failure:
                raise ValueError
            self.registry.verify_current()
            public = self.registry.public()
            # Project explicit fields only; no paths, private parameters or labels.
            machines = {name: {'waypoint_id': name,
                'inspection_xy_yaw': list(public['machines'][name]['inspection_xy_yaw']),
                'control_xy_m': list(public['machines'][name]['control_xy_m'])} for name in MACHINE_IDS}
            # All source reads finish before final receipt/simulation age checks.
            measured = self.reader.measure(self.now(), self.sim_ns())
            packet = measured['telemetry']
            if packet['schema_version'] != 2:
                raise ValueError
            digest = hashlib.sha256(canonical(packet).encode()).hexdigest()
            if digest != self.last_digest:
                self.last_digest, self.observation_id = digest, uuid.uuid4().hex
            return {'schema_version': 1, 'kind': 'planner_observation',
                'run_id': self.run_id, 'mission_id': self.mission_id, 'observation_id': self.observation_id,
                'registry_identity': public['identity'], 'frame': 'map', 'machines': machines,
                'station': {key: packet[key] for key in ('run_id', 'sequence', 'stamp_ns', 'time_s',
                    'units', 'voltage_v', 'machines', 'control')},
                'observed_sim_stamp_ns': measured['observed_sim_stamp_ns'], 'freshness': measured['freshness'],
                'scope': 'Public sampled measurements and configured geometry; not diagnosis, local inspection or action authority.'}
        except Unavailable:
            raise PlannerError('stale_observation') from None
        except (ValueError, OSError, TypeError, KeyError, OverflowError):
            self.failure = True
            raise PlannerError('unavailable') from None

    def validate(self, text):
        """Return a normalized request, without reservation, replay credit or effect.

        Stop remains syntactically available when telemetry/sources are invalid.
        An executor must recheck owners/interlocks and durably admit before send.
        """
        value = request(text, run_id=self.run_id, mission_id=self.mission_id)
        if value['tool'] in FRESH_TOOLS:
            # A provider cannot invent a reference before the host exposes a view.
            issued = self.observation_id
            if issued is None or value['observation_id'] != issued:
                raise PlannerError('stale_observation')
            if self.view()['observation_id'] != issued:
                raise PlannerError('stale_observation')
        return value

    def current(self, observation_id):
        """Final in-memory packet/age check; source I/O must already be finished."""
        try:
            if self.failure or observation_id != self.observation_id:
                raise ValueError
            measured = self.reader.measure(self.now(), self.sim_ns())
            digest = hashlib.sha256(canonical(measured['telemetry']).encode()).hexdigest()
            if self.last_digest != digest:raise ValueError
        except (Unavailable, ValueError, TypeError, OverflowError):
            raise PlannerError('stale_observation') from None
