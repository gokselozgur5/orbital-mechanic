"""Bind the causal model's public observations to a configured scene registry."""
import uuid

from orbital.machines import Fault, Station

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

    def sample(self, stamp_ns):
        if type(stamp_ns) is not int or not 0 <= stamp_ns < 2**63:
            raise ValueError('Expected a nonnegative simulation timestamp')
        self.registry.verify_current()
        if self.previous is not None:
            delta = stamp_ns-self.previous
            if delta == 0:
                return None  # A repeated clock is not a new measurement.
            if delta < 0 or delta > 10000000000:
                self.reset()  # Preserve the prototype's clock-reset semantics, with a new identity.
            else:
                remaining = delta
                while remaining:
                    step = min(remaining, 1000000000)
                    self.station.tick(step/1e9)
                    remaining -= step
        self.previous = stamp_ns
        if stamp_ns == 0:
            return None
        names = {binding.machine_id for binding in self.registry.bindings}
        if set(self.station.machines) != names or any(m.name != name for name, m in self.station.machines.items()):
            raise ValueError('Station identities differ from the scene registry')
        self.sequence += 1
        return {'schema_version': 1, 'source': 'simplified_station_model',
                'registry': self.registry.public(), 'run_id': self.run_id,
                'sequence': self.sequence, 'stamp_ns': stamp_ns, 'units': dict(UNITS),
                **self.station.observe()}
