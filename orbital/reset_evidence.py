"""Retained powered measurements for one reset decision; never hidden fault state."""
from collections import deque
import copy
import hashlib
import json

from orbital.inspection_protocol import canonical, finite, identifier
from orbital.machines import MACHINE_IDS
from orbital.proximity import Unavailable
from orbital.telemetry_observation import TelemetryEvidence

MAX_RESET_TEMPERATURE_C = 40.
MAX_RESET_VIBRATION_AU = 1.
MIN_RESET_LOAD_W = 110.


def eligible(measurement):
    return (measurement['powered'] is True
            and measurement['voltage_v']*measurement['current_a'] > MIN_RESET_LOAD_W
            and measurement['vibration_au'] <= MAX_RESET_VIBRATION_AU)


def basis(value):
    """Validate historical references; this never grants live reset permission."""
    if not isinstance(value, dict) or set(value) != {
            'registry_identity', 'run_id', 'machine_id', 'control_revision', 'samples'}:
        raise ValueError('invalid_reset_basis')
    identifier(value['registry_identity'], 64); identifier(value['run_id'])
    revision = value['control_revision']
    if (value['machine_id'] not in MACHINE_IDS or type(revision) is not int or not 0 <= revision < 2**63
            or not isinstance(value['samples'], list) or len(value['samples']) != 2):
        raise ValueError('invalid_reset_basis')
    for sample in value['samples']:
        if not isinstance(sample, dict) or set(sample) != {'sequence', 'stamp_ns', 'sample_sha256', 'measurements'}:
            raise ValueError('invalid_reset_basis')
        for name in ('sequence', 'stamp_ns'):
            if type(sample[name]) is not int or not 0 < sample[name] < 2**63:
                raise ValueError('invalid_reset_basis')
        identifier(sample['sample_sha256'], 64)
        measurement = sample['measurements']
        if not isinstance(measurement, dict) or set(measurement) != {'powered', 'voltage_v', 'current_a', 'vibration_au'}:
            raise ValueError('invalid_reset_basis')
        for name in ('voltage_v', 'current_a', 'vibration_au'): finite(measurement[name])
        if not eligible(measurement):
            raise ValueError('reset_basis_not_eligible')
    first, last = value['samples']
    if last['sequence'] != first['sequence']+1 or not 0 < last['stamp_ns']-first['stamp_ns'] < 1000000000:
        raise ValueError('invalid_reset_basis')
    return value


class ResetEvidence:
    """Sticky high-vibration/missing-source refusal within this station generation."""
    def __init__(self, registry):
        self.registry = registry
        self.reader = TelemetryEvidence(registry)
        self.samples = deque(maxlen=2)
        self.high_vibration = set()
        self.failure = None

    def fail(self, reason):
        self.failure = self.failure or reason

    def observe(self, text, now, sim):
        if self.failure:
            return
        if self.samples and (now-self.samples[-1][1] >= 1. or sim-self.samples[-1][0]['stamp_ns'] >= 1000000000):
            self.fail('missing_powered_history')
            return
        if not self.reader.observe(text, now, sim):
            self.fail(self.reader.failure)
            return
        if self.reader.missed:
            self.fail('missing_powered_history')
            return
        packet = json.loads(self.reader.candidate)
        if packet['schema_version'] != 2:
            self.fail('reset_requires_control_telemetry')
            return
        if self.samples:
            previous, current = self.samples[-1][0]['control'], packet['control']
            if (current['revision'] < previous['revision'] or current['revision'] > previous['revision']+1
                    or (current['revision'] == previous['revision'] and current != previous)):
                self.fail('contradictory_control_history')
                return
        for name, values in packet['machines'].items():
            if values['powered'] and values['vibration_au'] > MAX_RESET_VIBRATION_AU:
                self.high_vibration.add(name)
        self.samples.append((packet, now))

    def retain(self, machine, record, now, sim):
        """Freeze a powered pair while the owned isolation inspection is still live."""
        if self.failure or machine in self.high_vibration or len(self.samples) != 2:
            return None
        try:
            latest = self.reader.measure(now, sim)['telemetry']
            packets = [entry[0] for entry in self.samples]
            selected = record['telemetry_source']
            if (latest != packets[-1] or record['machine_id'] != machine
                    or record['registry_identity'] != self.registry.identity
                    or any(not 0 <= now-received < 1. or not 0 <= sim-p['stamp_ns'] < 1000000000
                           for p, received in self.samples)
                    or any(p['control']['revision'] != selected['control_revision'] for p in packets)
                    or selected['run_id'] != latest['run_id']):
                return None
            values = {'registry_identity': self.registry.identity, 'run_id': latest['run_id'],
                'machine_id': machine, 'control_revision': selected['control_revision'], 'samples': []}
            for packet in packets:
                measurement = {'voltage_v': packet['voltage_v'], **{
                    k:packet['machines'][machine][k] for k in ('powered', 'current_a', 'vibration_au')}}
                values['samples'].append({'sequence': packet['sequence'], 'stamp_ns': packet['stamp_ns'],
                    'sample_sha256': hashlib.sha256(canonical(packet).encode()).hexdigest(), 'measurements': measurement})
            basis(values)
            if not any(sample['sample_sha256'] == selected['sample_sha256']
                       and sample['stamp_ns'] == record['sample_stamp_ns']
                       and all(record['measurements'][key] == measurement for key, measurement in sample['measurements'].items())
                       for sample in values['samples']):
                return None
            return values
        except (Unavailable, ValueError, KeyError, TypeError):
            return None

    def confirm(self, value, now, sim):
        if value is None or self.failure or value['machine_id'] in self.high_vibration:
            return False
        try:
            packet = self.reader.measure(now, sim)['telemetry']
            return (packet['run_id'] == value['run_id']
                    and hashlib.sha256(canonical(packet).encode()).hexdigest() == value['samples'][-1]['sample_sha256'])
        except Unavailable:
            return False

    def isolated(self, machine, run_id, revision, now, sim):
        """Current cooldown input, separate from the frozen powered basis."""
        if self.failure:
            raise ValueError('reset_evidence_unavailable')
        packet = self.reader.measure(now, sim)['telemetry']
        values = packet['machines'][machine]
        if (packet['run_id'] != run_id or packet['control']['revision'] != revision
                or values['powered'] is not False or values['current_a'] != 0 or values['vibration_au'] != 0):
            raise ValueError('reset_requires_verified_isolation')
        return copy.deepcopy(values)
