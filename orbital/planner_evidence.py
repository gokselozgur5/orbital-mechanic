"""Bounded public records returned by the trusted tool executive.

Owner results are reported evidence, not independent robot measurements.
No provider-supplied object is admitted here as an execution observation.
"""
from types import SimpleNamespace

from orbital import inspection_protocol as inspection, maintenance_protocol as maintenance
from orbital.inspection_protocol import canonical, decode, finite, identifier
from orbital.machines import MACHINE_IDS
from orbital.navigation_client import _target
from orbital.telemetry_observation import decode as telemetry_decode


def observation(value, *, baseline, run_id, mission_id):
    value = decode(canonical(value), 32768)
    for key, expected in (('schema_version', 1), ('run_id', run_id), ('mission_id', mission_id)):
        if value.get(key) != expected or (key == 'schema_version' and type(value[key]) is not int):
            raise ValueError('invalid_tool_observation')
    identifier(value.get('observation_id'))
    stamp = maintenance.integer(value.get('observed_sim_stamp_ns'), 1)
    kind = value.get('kind')
    if kind == 'planner_observation':
        if set(value) != {'schema_version', 'kind', 'run_id', 'mission_id', 'observation_id',
                'registry_identity', 'frame', 'machines', 'station', 'observed_sim_stamp_ns', 'freshness', 'scope'}:
            raise ValueError('invalid_tool_observation')
        registry = baseline['registry']
        expected = {name: {'waypoint_id': name, 'inspection_xy_yaw': item['inspection_xy_yaw'],
            'control_xy_m': item['control_xy_m']} for name, item in registry['machines'].items()}
        if (value['registry_identity'] != registry['identity'] or value['frame'] != 'map'
                or canonical(value['machines']) != canonical(expected)):
            raise ValueError('invalid_tool_geometry')
        if set(value['station']) != {'run_id', 'sequence', 'stamp_ns', 'time_s',
                                     'units', 'voltage_v', 'machines', 'control'}:
            raise ValueError('invalid_tool_station')
        packet = telemetry_decode(canonical({'schema_version': 2, 'source': 'simplified_station_model',
            'registry': registry, **value['station']}), SimpleNamespace(public=lambda: registry))
        if packet['run_id'] != baseline['run_id'] or not 0 <= stamp-packet['stamp_ns'] < 1000000000:
            raise ValueError('invalid_tool_station')
        fresh = value['freshness']
        if set(fresh) != {'received_age_s', 'simulation_age_s', 'max_age_s', 'consecutive_samples', 'missing_sequences'}:
            raise ValueError('invalid_tool_freshness')
        if (not 0 <= finite(fresh['received_age_s']) < 1. or fresh['max_age_s'] != 1.
                or fresh['simulation_age_s'] != (stamp-packet['stamp_ns'])/1e9):
            raise ValueError('invalid_tool_freshness')
        maintenance.integer(fresh['consecutive_samples'], 2)
        maintenance.integer(fresh['missing_sequences'])
    else:
        if set(value) != {'schema_version', 'kind', 'run_id', 'mission_id',
                         'observation_id', 'observed_sim_stamp_ns', 'data', 'scope'}:
            raise ValueError('invalid_tool_observation')
        data = value['data']
        if kind == 'planner_inspection_observation':
            inspection.result(data)
            if data['mission_id'] != mission_id:raise ValueError('invalid_tool_action')
        elif kind == 'planner_maintenance_observation':
            maintenance.result(data)
            if data['mission_id'] != mission_id or data['station_run_id'] != baseline['run_id']:
                raise ValueError('invalid_tool_action')
        elif kind == 'planner_navigation_observation':
            if set(data) != {'target', 'arrived', 'within_tolerance'}:
                raise ValueError('invalid_tool_arrival')
            _target(data['target'])
            if type(data['arrived']) is not bool or type(data['within_tolerance']) is not bool:
                raise ValueError('invalid_tool_arrival')
            if data['arrived'] and not data['within_tolerance']:raise ValueError('invalid_tool_arrival')
        elif kind == 'planner_recovery_observation':
            if set(data) != {'station_run_id', 'control_revision', 'machine_id', 'window', 'required_window_s'}:
                raise ValueError('invalid_tool_recovery')
            if data['station_run_id'] != baseline['run_id'] or data['machine_id'] not in MACHINE_IDS:
                raise ValueError('invalid_tool_recovery')
            maintenance.integer(data['control_revision'])
            finite(data['required_window_s'], 10., 30.)
            window = data['window']
            if set(window) != {'first', 'last', 'duration_s', 'samples', 'samples_sha256'}:
                raise ValueError('invalid_tool_recovery')
            for sample in (window['first'], window['last']):
                if set(sample) != {'sequence', 'stamp_ns', 'time_s'}:raise ValueError('invalid_tool_recovery')
                maintenance.integer(sample['sequence'], 1); maintenance.integer(sample['stamp_ns'], 1)
                finite(sample['time_s'])
            if (window['duration_s'] != (window['last']['stamp_ns']-window['first']['stamp_ns'])/1e9
                    or not data['required_window_s'] <= window['duration_s'] <= 600.
                    or window['last']['stamp_ns'] > stamp):
                raise ValueError('invalid_tool_recovery')
            maintenance.integer(window['samples'], 2); identifier(window['samples_sha256'], 64)
        elif kind == 'planner_stop_observation':
            if set(data) != {'navigation_run_id', 'inspection_run_id', 'maintenance_run_id',
                             'control_revision', 'guard_latched', 'owners_idle'}:
                raise ValueError('invalid_tool_stop')
            if (type(data['navigation_run_id']) is not str or not 1 <= len(data['navigation_run_id']) <= 128
                    or data['guard_latched'] is not True or data['owners_idle'] is not True):
                raise ValueError('invalid_tool_stop')
            identifier(data['inspection_run_id']); identifier(data['maintenance_run_id'])
            maintenance.integer(data['control_revision'])
        else:raise ValueError('invalid_tool_observation')
    if not isinstance(value['scope'], str) or not 1 <= len(value['scope']) <= 256:
        raise ValueError('invalid_tool_observation')
    return value
