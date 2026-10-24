#!/usr/bin/env python3
"""Deterministic, ROS-free scenario baseline. No navigation or LLM is claimed."""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from orbital.machines import Fault, Station


def run(fault):
    station = Station(fault)
    trace = []
    before = station.observe()
    if before['voltage_v'] >= 27:
        return {'scenario': fault.value, 'outcome': 'healthy', 'interventions': 0, 'passed': True}
    suspect = max(before['machines'], key=lambda key: before['machines'][key]['current_a'])
    # The scripted policy sees measurements, never the fault enum.
    if before['machines'][suspect]['vibration_au'] > 1:
        return {'scenario': fault.value, 'outcome': 'escalated', 'interventions': 0, 'passed': fault == Fault.JAM}
    for index, operation in enumerate(('isolate', 'reset', 'restore')):
        # Trusted geometry is simulated here; ROS adapter integration is still pending.
        result = station.intervene(str(index), operation, suspect, distance_m=.5, speed_mps=0, observed_at_s=station.time_s)
        trace.append({'operation': operation, 'machine_id': suspect, 'result': result})
    for _ in range(10): station.tick(1)
    return {'scenario': fault.value, 'outcome': 'repaired' if station.recovered else 'escalated',
            'interventions': len(trace), 'trace': trace, 'before': before, 'after': station.observe(),
            'passed': station.recovered and fault == Fault.LATCHED}

if __name__ == '__main__':
    results = [run(fault) for fault in Fault]
    print(json.dumps({'scope': 'ROS-free machine model only', 'results': results}, indent=2))
    raise SystemExit(0 if all(row['passed'] for row in results) else 1)
