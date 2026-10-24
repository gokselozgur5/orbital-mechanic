"""Simplified causal electrical/thermal model, not a hardware digital twin.

The evaluator owns Machine.fault. Planner integrations must use observe(), never
serialize model instances or expose fault injection methods.
"""
from dataclasses import dataclass
from enum import Enum
import math


class Fault(str, Enum):
    NONE = 'none'
    LATCHED = 'latched_controller'
    JAM = 'mechanical_jam'


class Rejected(ValueError):
    pass


@dataclass
class Machine:
    name: str
    fault: Fault = Fault.NONE
    powered: bool = True
    temperature_c: float = 30.0
    reset_count: int = 0

    @property
    def load_w(self):
        return 0.0 if not self.powered else (300.0 if self.fault != Fault.NONE else 100.0)

    def tick(self, dt):
        # First-order thermal model; exact discretization avoids step-size instability.
        target = 25.0 + self.load_w * 0.08
        self.temperature_c = target + (self.temperature_c - target) * math.exp(-dt / 30.0)


class Station:
    def __init__(self, fault=Fault.NONE):
        self.machines = {name: Machine(name) for name in ('pump', 'cooler', 'power_unit')}
        self.machines['pump'].fault = fault
        self.time_s = 0.0
        self.healthy_s = 0.0
        self._completed = {}

    @property
    def voltage_v(self):
        # 450 W supply capacity; excess load causes bus droop.
        return max(0.0, 28.0 - max(0.0, sum(m.load_w for m in self.machines.values()) - 450.0) * 0.08)

    def tick(self, dt):
        if not math.isfinite(dt) or dt <= 0 or dt > 1:
            raise Rejected('dt must be finite and in (0, 1] seconds')
        for m in self.machines.values():
            m.tick(dt)
        self.time_s += dt
        # Power isolation alone is not repair: every machine must be operating.
        healthy = self.voltage_v >= 27 and all(m.powered and m.load_w <= 110 for m in self.machines.values())
        self.healthy_s = self.healthy_s + dt if healthy else 0.0

    def observe(self):
        voltage = self.voltage_v
        return {'time_s': self.time_s, 'voltage_v': voltage, 'machines': {
            name: {'powered': m.powered, 'temperature_c': m.temperature_c,
                   'current_a': m.load_w / max(voltage, 1.0),
                   'vibration_au': 0.0 if not m.powered else (3.0 if m.fault == Fault.JAM else 0.2)}
            for name, m in self.machines.items()}}

    def intervene(self, action_id, operation, machine_id, *, distance_m, speed_mps, observed_at_s):
        args = (operation, machine_id, distance_m, speed_mps, observed_at_s)
        if not isinstance(action_id, str) or not action_id or len(action_id) > 128:
            raise Rejected('invalid action ID')
        if action_id in self._completed:
            previous, result = self._completed[action_id]
            if previous != args:
                raise Rejected('action ID reused with different arguments')
            return dict(result)
        if len(self._completed) >= 100:
            raise Rejected('station action budget exhausted')
        if operation not in ('isolate', 'reset', 'restore') or machine_id not in self.machines:
            raise Rejected('unknown control or machine')
        if not all(math.isfinite(x) for x in (distance_m, speed_mps, observed_at_s)):
            raise Rejected('non-finite precondition')
        if not 0 <= distance_m <= 0.75 or abs(speed_mps) > 0.01:
            raise Rejected('robot must be stationary and near control')
        if not 0 <= self.time_s - observed_at_s <= 2:
            raise Rejected('stale or future observation')
        m = self.machines[machine_id]
        if operation == 'isolate':
            m.powered = False
        elif operation == 'reset':
            if m.powered or m.temperature_c > 40 or m.reset_count >= 1:
                raise Rejected('reset requires isolation, cooldown and remaining attempt')
            m.reset_count += 1
            if m.fault == Fault.LATCHED:
                m.fault = Fault.NONE
        else:
            if m.powered or m.reset_count != 1 or m.temperature_c > 40:
                raise Rejected('restore requires isolated, reset and cooled machine')
            # A jam remains physically present; restoring cannot conceal it.
            m.powered = True
        self.healthy_s = 0.0
        result = {'action_id': action_id, 'status': 'succeeded', 'time_s': self.time_s}
        self._completed[action_id] = (args, result)
        return dict(result)

    @property
    def recovered(self):
        return self.healthy_s >= 10.0
