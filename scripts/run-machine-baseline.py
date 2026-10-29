#!/usr/bin/env python3
"""Deterministic, ROS-free scenario baseline. No navigation or LLM is claimed."""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from orbital.machines import Fault
from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.maintenance_observation import RecoveryEvidence
from orbital.proximity import Unavailable

ROOT = Path(__file__).resolve().parents[1]


def run(fault):
    registry = MachineRegistry.load(ROOT)
    model = MachineTelemetry(registry, fault)
    station = model.station
    trace = []
    before = model.sample(1_000_000_000)
    if before['voltage_v'] >= 27:
        return {'scenario': fault.value, 'outcome': 'healthy', 'interventions': 0, 'passed': True}
    suspect = max(before['machines'], key=lambda key: before['machines'][key]['current_a'])
    # The scripted policy sees measurements, never the fault enum.
    if before['machines'][suspect]['vibration_au'] > 1:
        return {'scenario': fault.value, 'outcome': 'escalated', 'interventions': 0, 'passed': fault == Fault.JAM}
    for index, operation in enumerate(('isolate', 'reset', 'restore')):
        # Trusted geometry is simulated here; ROS adapter integration is still pending.
        result = station.intervene(str(index), operation, suspect, distance_m=.5, speed_mps=0, observed_at_s=before['time_s'])
        trace.append({'operation': operation, 'machine_id': suspect, 'result': result})
    recovery = RecoveryEvidence(registry)
    verification = None
    # At 2 Hz, two samples establish freshness; another ten simulation seconds
    # of consecutive healthy public measurements establish the postcondition.
    for index in range(1, 24):
        stamp = 1_000_000_000 + index * 500_000_000
        after = model.sample(stamp)
        if not recovery.observe(json.dumps(after), stamp / 1e9, stamp):
            break
        try:
            verification = recovery.measure(stamp / 1e9, stamp)
        except Unavailable:
            continue
        if verification['verified']:
            break
    recovered = verification is not None and verification['verified']
    return {'scenario': fault.value, 'outcome': 'repaired' if recovered else 'escalated',
            'interventions': len(trace), 'trace': trace, 'before': before, 'after': after,
            'verification': verification,
            'passed': recovered and fault == Fault.LATCHED}

if __name__ == '__main__':
    results = [run(fault) for fault in Fault]
    print(json.dumps({'scope': 'ROS-free machine model only', 'results': results}, indent=2))
    raise SystemExit(0 if all(row['passed'] for row in results) else 1)
