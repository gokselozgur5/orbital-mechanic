"""Bind the causal model's public observations to a configured scene registry."""
import uuid
import copy

from orbital.machines import Fault, Station, Rejected
from orbital.inspection_protocol import canonical, identifier, finite
from orbital.station_control import EffectRejected

UNITS = {'stamp_ns': 'simulation_nanoseconds', 'time_s': 'model_seconds',
         'voltage_v': 'V', 'temperature_c': 'degC', 'current_a': 'A',
         'vibration_au': 'arbitrary_units', 'position': 'm', 'yaw': 'rad',
         'quaternion': 'dimensionless_xyzw'}


class MachineTelemetry:
    def __init__(self, registry, fault=Fault.NONE):
        self.registry, self.fault = registry, fault
        self.previous = None
        self.reset()

    def reset(self):
        self.station = Station(self.fault)
        self.run_id = uuid.uuid4().hex
        self.sequence = 0
        self.control_revision = 0
        self.last_applied = None
        self.effects = {}
        self.reset_attempts = {}

    def _advance(self, stamp_ns):
        if type(stamp_ns) is not int or not 0 <= stamp_ns < 2**63:
            raise ValueError('Expected a nonnegative simulation timestamp')
        if self.previous is not None:
            delta = stamp_ns-self.previous
            if delta == 0:
                return False  # A repeated clock is not a new measurement.
            if delta < 0 or delta > 10000000000:
                self.reset()  # Preserve the prototype's clock-reset semantics, with a new identity.
            else:
                remaining = delta
                while remaining:
                    step = min(remaining, 1000000000)
                    self.station.tick(step/1e9)
                    remaining -= step
        self.previous = stamp_ns
        return True

    def sample(self, stamp_ns):
        self.registry.verify_current()
        if not self._advance(stamp_ns):
            return None
        if stamp_ns == 0:
            return None
        names = {binding.machine_id for binding in self.registry.bindings}
        if set(self.station.machines) != names or any(m.name != name for name, m in self.station.machines.items()):
            raise ValueError('Station identities differ from the scene registry')
        self.sequence += 1
        return {'schema_version': 2, 'source': 'simplified_station_model',
                'registry': self.registry.public(), 'run_id': self.run_id,
                'sequence': self.sequence, 'stamp_ns': stamp_ns, 'units': dict(UNITS),
                'control': {'revision': self.control_revision, 'last_applied': copy.deepcopy(self.last_applied)},
                **self.station.observe()}

    def isolate(self, context, expected_run, expected_revision, *, admission):
        return self._control('isolate', context, expected_run, expected_revision, admission=admission)

    def reset_machine(self, context, expected_run, expected_revision, *, isolation_action_id, admission):
        return self._control('reset', context, expected_run, expected_revision,
                             isolation_action_id=isolation_action_id, admission=admission)

    def reset_available(self, machine_id):
        return machine_id not in self.reset_attempts and self.station.machines[machine_id].reset_count == 0

    def _control(self, operation, context, expected_run, expected_revision, *, admission, isolation_action_id=None):
        """Serialize an effect on this published Station, after a live local gate.

        Only the ROS owner supplies admission; no wire request supplies geometry.
        A replay returns history without another effect or renewed freshness.
        """
        fields = {'owner_run_id', 'mission_id', 'action_id', 'request_sha256', 'machine_id'}
        try:
            if not isinstance(context, dict) or set(context) != fields:
                raise ValueError('invalid_effect_context')
            for name in ('owner_run_id', 'mission_id', 'action_id'):
                identifier(context[name])
            identifier(context['request_sha256'], 64)
            identifier(expected_run)
            if operation not in ('isolate', 'reset'):
                raise ValueError('unknown_control')
            if operation == 'reset': identifier(isolation_action_id)
            if (context['machine_id'] not in self.station.machines or type(expected_revision) is not int
                    or not 0 <= expected_revision < 2**63 or expected_run != self.run_id):
                raise ValueError('wrong_station_state')
            key = (context['owner_run_id'], context['action_id'])
            intent = canonical({'context': context, 'run_id': expected_run, 'revision': expected_revision,
                                'operation': operation, 'isolation_action_id': isolation_action_id})
            previous = self.effects.get(key)
            if previous is not None:
                if previous[0] != intent:
                    raise ValueError('effect_conflict')
                return copy.deepcopy(previous[1])
            if len(self.effects) >= 64 or expected_revision != self.control_revision:
                raise ValueError('effect_capacity_or_revision')
            if operation == 'reset':
                if not self.reset_available(context['machine_id']):
                    raise ValueError('reset_attempt_spent_or_unknown')
                prior = self.effects.get((context['owner_run_id'], isolation_action_id))
                if (prior is None or prior[1]['operation'] != 'isolate'
                        or any(prior[1][k] != context[k] for k in ('owner_run_id', 'mission_id', 'machine_id'))):
                    raise ValueError('owned_isolation_required')
            self.registry.verify_current()
            live = admission()  # All file/manifest work precedes this final gate.
            stamp = live['stamp_ns']
            if type(stamp) is not int or not 0 < stamp < 2**63:
                raise ValueError('invalid_effect_clock')
            self._advance(stamp)
            if expected_run != self.run_id or expected_revision != self.control_revision:
                raise ValueError('station_run_changed')
            machine = self.station.machines[context['machine_id']]
            if operation == 'isolate' and not machine.powered:
                raise ValueError('already_isolated')
            if operation == 'reset' and (machine.powered or machine.temperature_c > 40):
                raise ValueError('reset_requires_isolation_and_cooldown')
            effect = {**context, 'operation': operation, 'control_revision': self.control_revision+1,
                      'applied_stamp_ns': stamp}
            receipt = copy.deepcopy(effect)
            history = (intent, receipt)
            # These bounded values are prepared before the synchronous mutation.
            distance, speed = finite(live['distance_m']), finite(live['speed_mps'])
        except (ValueError, KeyError, TypeError, OSError, RuntimeError) as error:
            raise EffectRejected(str(error)) from None
        if operation == 'reset':
            # Reserve before the pulse: an unexpected actuator exception cannot
            # silently give another action or owner a fresh attempt budget.
            self.reset_attempts[context['machine_id']] = key
        try:
            self.station.intervene(context['owner_run_id']+context['action_id'], operation, context['machine_id'],
                                   distance_m=distance, speed_mps=speed, observed_at_s=self.station.time_s)
        except Rejected as error:
            if operation == 'reset': del self.reset_attempts[context['machine_id']]
            raise EffectRejected(str(error)) from None
        self.control_revision = effect['control_revision']
        self.last_applied = effect
        self.effects[key] = history
        return copy.deepcopy(receipt)
