"""Private station observations, not a planner API or a successful-run assertion."""
import copy
import hashlib

from orbital.inspection_protocol import canonical, finite
from orbital.machines import Fault, MACHINE_IDS
from orbital.station_control import control

DIRECTORY = 'station-observations'
MAX_BYTES = 32 * 1024 * 1024
MAX_RECORDS = 20000
MAX_LINE = 16384


def integer(value, minimum=0):
    if type(value) is not int or not minimum <= value < 2**63:
        raise ValueError('invalid_station_observation_integer')
    return value


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def snapshot(model):
    value = {'sim_stamp_ns':model.previous, 'model_time_s':model.station.time_s,
             'sequence':model.sequence,
             'control':{'revision':model.control_revision, 'last_applied':copy.deepcopy(model.last_applied)},
             'voltage_v':model.station.voltage_v,
             'machines':{name:{'fault':m.fault.value, 'powered':m.powered,
                 'temperature_c':m.temperature_c, 'reset_count':m.reset_count, 'load_w':m.load_w}
                 for name,m in model.station.machines.items()}}
    validate_snapshot(value)
    return value


def validate_snapshot(value):
    if not isinstance(value, dict) or set(value) != {
            'sim_stamp_ns','model_time_s','sequence','control','voltage_v','machines'}:
        raise ValueError('invalid_station_snapshot')
    if value['sim_stamp_ns'] is not None: integer(value['sim_stamp_ns'])
    finite(value['model_time_s']); integer(value['sequence'])
    finite(value['voltage_v'], 0., 1000.)
    control(value['control'], value['sim_stamp_ns'] or 0)
    if not isinstance(value['machines'], dict) or set(value['machines']) != set(MACHINE_IDS):
        raise ValueError('invalid_station_snapshot')
    for m in value['machines'].values():
        if (not isinstance(m, dict) or set(m) != {'fault','powered','temperature_c','reset_count','load_w'}
                or type(m['fault']) is not str or m['fault'] not in tuple(f.value for f in Fault)
                or type(m['powered']) is not bool):
            raise ValueError('invalid_station_snapshot')
        finite(m['temperature_c'], 0., 1000.); finite(m['load_w'], 0., 1e6); integer(m['reset_count'])
    return value
