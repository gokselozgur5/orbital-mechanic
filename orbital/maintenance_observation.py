"""Observable recovery postcondition; no actuator, diagnosis or motion authority."""
import hashlib

from orbital.proximity import Unavailable
from orbital.telemetry_observation import TelemetryEvidence, canonical

RECOVERY_WINDOW_NS = 10_000_000_000
MIN_VOLTAGE_V = 27.0
MAX_MACHINE_POWER_W = 110.0


class RecoveryEvidence:
    """Require a complete, fresh sequence of public healthy station measurements.

    The adapter owns both clocks and the registry. This checker cannot establish
    causation, continuous physical health or whether a reset was applied.
    """
    def __init__(self, registry):
        self.registry = registry
        self._telemetry = TelemetryEvidence(registry)
        self._clear_window()
        self._healthy = False

    def _clear_window(self):
        self._first = None
        self._last = None
        self._samples = 0
        self._digest = hashlib.sha256()

    def observe(self, text, now, sim_ns):
        if not self._telemetry.observe(text, now, sim_ns):
            self._clear_window()
            return False
        try:
            observation = self._telemetry.measure(now, sim_ns)
        except Unavailable:
            # Initial warmup, a missing sequence or an expired receipt cannot
            # bridge two healthy periods into one sustained recovery window.
            self._clear_window()
            return True
        sample = observation['telemetry']
        voltage = sample['voltage_v']
        self._healthy = voltage >= MIN_VOLTAGE_V and all(
            machine['powered'] and voltage * machine['current_a'] <= MAX_MACHINE_POWER_W
            for machine in sample['machines'].values())
        if not self._healthy:
            self._clear_window()
            return True
        reference = {key: sample[key] for key in ('sequence', 'stamp_ns', 'time_s')}
        if self._first is None:
            self._first = reference
        self._last = reference
        self._samples += 1
        self._digest.update(canonical(sample).encode('utf-8') + b'\n')
        return True

    def measure(self, now, sim_ns):
        try:
            observation = self._telemetry.measure(now, sim_ns)
        except Unavailable:
            self._clear_window()
            raise
        sample = observation['telemetry']
        elapsed_ns = 0 if self._first is None else self._last['stamp_ns'] - self._first['stamp_ns']
        verified = self._healthy and elapsed_ns >= RECOVERY_WINDOW_NS
        window = None if self._first is None else {
            'first': dict(self._first), 'last': dict(self._last),
            'duration_s': elapsed_ns / 1e9, 'samples': self._samples,
            'samples_sha256': self._digest.hexdigest(),
        }
        return {
            'schema_version': 1, 'kind': 'station_recovery_observation', 'valid': True,
            'verified': verified,
            'reason': 'verified' if verified else (
                'awaiting_recovery_window' if self._healthy else 'unhealthy_measurement'),
            'run_id': sample['run_id'], 'registry_identity': self.registry.identity,
            'criteria': {'min_voltage_v': MIN_VOLTAGE_V,
                         'max_machine_power_w': MAX_MACHINE_POWER_W,
                         'all_machines_powered': True,
                         'required_sim_window_s': RECOVERY_WINDOW_NS / 1e9},
            'window': window,
            'latest': {key: sample[key] for key in ('sequence', 'stamp_ns', 'time_s')},
            'freshness': observation['freshness'],
            'scope': 'sampled public station recovery; not reset confirmation, repair causation, '
                     'continuous physical health or maintenance authorization',
        }
