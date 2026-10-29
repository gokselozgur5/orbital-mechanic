"""Private reproducible station inputs, separate from the public assignment.

This module belongs to the trusted scenario/station side. Its initial conditions
are not observations, a planner tool, an independent score or native acceptance.
"""
import copy
import hashlib
from pathlib import Path
import uuid

from orbital.inspection_protocol import canonical, decode, finite, identifier
from orbital.machines import Fault, MACHINE_IDS

PROFILES = ('healthy', 'recoverable', 'persistent', 'ambiguous', 'blocked')
GENERATOR = 'station-profile-v1'
MAX_BYTES = 16384
SOURCE_FILES = ('orbital/machines.py', 'orbital/machine_telemetry.py',
                'orbital/scenario_spec.py', 'orbital/scenario_input.py')
SOURCE_ROOT = Path(__file__).resolve().parents[1]


def source_revision():
    """Pin these declared input/model sources, not an attestation of loaded code."""
    values = {name: hashlib.sha256((SOURCE_ROOT/name).read_bytes()).hexdigest() for name in SOURCE_FILES}
    return hashlib.sha256(canonical(values).encode()).hexdigest()


def initial_state(value):
    if not isinstance(value, dict) or set(value) != set(MACHINE_IDS):
        raise ValueError('invalid_private_initial_state')
    result = {}
    for name, machine in value.items():
        if (not isinstance(machine, dict) or set(machine) != {'fault', 'powered', 'temperature_c'}
                or type(machine['fault']) is not str or machine['fault'] not in tuple(v.value for v in Fault)
                or machine['powered'] is not True):
            raise ValueError('invalid_private_initial_state')
        result[name] = {'fault': machine['fault'], 'powered': True,
                        'temperature_c': finite(machine['temperature_c'], 25., 35.)}
    return result


def conditions(profile, seed):
    if type(profile) is not str or profile not in PROFILES or type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError('invalid_scenario_specification')
    selected = MACHINE_IDS[seed % len(MACHINE_IDS)]
    machines = {}
    for name in MACHINE_IDS:
        # Temperature variation is independent of the fault class. No Python
        # hash/random implementation or process-global RNG controls replay.
        digest = hashlib.sha256(f'{GENERATOR}:temperature:{seed}:{name}'.encode()).digest()
        machines[name] = {'fault': 'none', 'powered': True,
                          'temperature_c': 28.+int.from_bytes(digest[:4], 'big') % 601 / 100.}
    if profile != 'healthy':
        machines[selected]['fault'] = 'mechanical_jam' if profile == 'persistent' else 'latched_controller'
    if profile == 'ambiguous':
        machines[MACHINE_IDS[(seed+1) % len(MACHINE_IDS)]]['fault'] = 'latched_controller'
    navigation = {'mode': 'blocked_scene_required' if profile == 'blocked' else 'normal',
                  'waypoint_id': selected if profile == 'blocked' else None}
    return initial_state(machines), navigation


def create(profile, seed, registry_identity, *, trial_id=None):
    machines, navigation = conditions(profile, seed)
    return {'schema_version': 1, 'kind': 'private_scenario_input', 'generator': GENERATOR,
            'trial_id': identifier(uuid.uuid4().hex if trial_id is None else trial_id),
            'registry_identity': identifier(registry_identity, 64), 'source_revision': source_revision(),
            'profile': profile, 'seed': seed, 'initial_state': machines, 'navigation': navigation}


def validate(text, registry_identity):
    value = decode(text, MAX_BYTES)
    fields = {'schema_version', 'kind', 'generator', 'trial_id', 'registry_identity',
              'source_revision', 'profile', 'seed', 'initial_state', 'navigation'}
    if (set(value) != fields or type(value['schema_version']) is not int or value['schema_version'] != 1
            or value['kind'] != 'private_scenario_input' or value['generator'] != GENERATOR):
        raise ValueError('invalid_scenario_specification')
    identifier(value['trial_id']); identifier(value['source_revision'], 64)
    if value['registry_identity'] != identifier(registry_identity, 64) or value['source_revision'] != source_revision():
        raise ValueError('scenario_source_mismatch')
    machines, navigation = conditions(value['profile'], value['seed'])
    if (canonical(value['initial_state']) != canonical(machines)
            or canonical(value['navigation']) != canonical(navigation)):
        raise ValueError('scenario_conditions_mismatch')
    return copy.deepcopy(value)


def require_station_fixture(value):
    if value['navigation']['mode'] != 'normal':
        # Do not turn a blocked descriptor into an unblocked native trial or a
        # fabricated navigation failure. A geometry-bound fixture is still due.
        raise ValueError('scenario_requires_compiled_navigation_fixture')


def assignment(value):
    """An allowlist, never a copy followed by deletion of known secret fields."""
    return {'schema_version': 1, 'kind': 'mission_assignment',
            'trial_id': identifier(value['trial_id']),
            'registry_identity': identifier(value['registry_identity'], 64),
            'task': 'restore_station_power'}
