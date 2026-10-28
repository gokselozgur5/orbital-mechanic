"""Strict public telemetry decoding and freshness; never a diagnosis or action token."""
import json
import math
import re

from orbital.machine_telemetry import UNITS
from orbital.machines import MACHINE_IDS
from orbital.navigation_client import _json
from orbital.proximity import Unavailable
from orbital.station_control import control

MAX_AGE_S = 1.0  # Two nominal 0.5-second publisher periods, in both clocks.
MAX_AGE_NS = 1000000000
MAX_PACKET_BYTES = 16384
FIELDS = {'schema_version','source','registry','run_id','sequence','stamp_ns','units',
          'time_s','voltage_v','machines'}
MACHINE_FIELDS = {'powered','temperature_c','current_a','vibration_au'}


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)


def integer(value, *, positive=False):
    return type(value) is int and int(positive)<=value<2**63


def number(value, low=0., high=1e6):
    return type(value) in (int,float) and math.isfinite(value) and low<=value<=high


def decode(text, registry):
    """Accept exactly the existing public wire schema, including its full catalog."""
    if (not isinstance(text,str) or len(text)>MAX_PACKET_BYTES
            or len(text.encode('utf-8'))>MAX_PACKET_BYTES):
        raise ValueError('invalid_telemetry')
    value = _json(text,MAX_PACKET_BYTES)
    version = value.get('schema_version')
    expected_fields = FIELDS | {'control'} if version == 2 else FIELDS
    if (set(value)!=expected_fields or type(version) is not int or version not in (1, 2)
            or value['source']!='simplified_station_model' or value['units']!=UNITS
            or canonical(value['registry'])!=canonical(registry.public())
            or not isinstance(value['run_id'],str) or re.fullmatch('[0-9a-f]{32}',value['run_id']) is None
            or not integer(value['sequence'],positive=True) or not integer(value['stamp_ns'],positive=True)
            or not number(value['time_s'],high=(2**63-1)/1e9) or not number(value['voltage_v'],high=10000)
            or not isinstance(value['machines'],dict) or set(value['machines'])!=set(MACHINE_IDS)):
        raise ValueError('invalid_telemetry')
    for machine in value['machines'].values():
        if (not isinstance(machine,dict) or set(machine)!=MACHINE_FIELDS
                or type(machine['powered']) is not bool
                or not number(machine['temperature_c'],low=-273.15)
                or not number(machine['current_a']) or not number(machine['vibration_au'])):
            raise ValueError('invalid_telemetry')
    if version == 2:
        control(value['control'], value['stamp_ns'])
    return value


class TelemetryEvidence:
    """One query pins one run; invalid input cannot be erased by later valid input."""
    def __init__(self, registry):
        self.registry = registry
        self.failure = None
        self.last_wall = self.last_clock = self.first_clock = None
        self.run_id = self.previous = self.received_at = self.candidate = None
        self.samples = self.missed = 0

    def fail(self, code):
        self.failure = self.failure or code
        raise Unavailable(self.failure)

    def tick(self, now, sim_ns):
        if self.failure:
            raise Unavailable(self.failure)
        if not number(now,high=float('inf')) or not integer(sim_ns):
            self.fail('invalid_observation_clock')
        if ((self.last_wall is not None and now<self.last_wall)
                or (self.last_clock is not None and sim_ns<self.last_clock)):
            self.fail('observation_clock_regressed')
        self.last_wall,self.last_clock = now,sim_ns
        if self.first_clock is None and sim_ns>0:
            self.first_clock = sim_ns
        if self.candidate is not None and (now-self.received_at>=MAX_AGE_S
                or sim_ns-self.previous['stamp_ns']>=MAX_AGE_NS):
            self.candidate = None
            self.samples = 0

    def observe(self, text, now, sim_ns):
        try:
            self.tick(now,sim_ns)
            value = decode(text,self.registry)
            stamp = value['stamp_ns']
            if not 0<=sim_ns-stamp<MAX_AGE_NS:
                self.fail('stale_or_future_telemetry')
            if self.run_id is None:
                self.run_id = value['run_id']
            elif value['run_id']!=self.run_id:
                self.fail('telemetry_run_changed')
            if self.previous is not None:
                previous = self.previous
                if value['sequence']<=previous['sequence'] or stamp<=previous['stamp_ns']:
                    self.fail('replayed_or_unordered_telemetry')
                if not math.isclose(value['time_s']-previous['time_s'],
                                    (stamp-previous['stamp_ns'])/1e9,rel_tol=1e-6,abs_tol=1e-6):
                    self.fail('model_clock_mismatch')
                missing = value['sequence']-previous['sequence']-1
                if missing:
                    self.missed += missing
                    self.samples = 0
            self.previous = {k:value[k] for k in ('sequence','stamp_ns','time_s')}
            self.received_at = now
            self.candidate = canonical(value)
            self.samples += 1
            return True
        except Unavailable:
            return False
        except (ValueError,TypeError,OverflowError,RecursionError,UnicodeError):
            self.failure = self.failure or 'invalid_telemetry'
            return False

    def measure(self, now, sim_ns):
        self.tick(now,sim_ns)
        if self.candidate is None or self.samples<2:
            raise Unavailable('awaiting_fresh_telemetry_pair')
        if sim_ns<=self.first_clock or self.previous['stamp_ns']<self.first_clock:
            raise Unavailable('awaiting_advancing_observation_clock')
        value = json.loads(self.candidate)  # Independent public data; no cached mutable object escapes.
        return {'schema_version':1,'valid':True,'telemetry':value,
                'observed_sim_stamp_ns':sim_ns,
                'freshness':{'received_age_s':now-self.received_at,
                             'simulation_age_s':(sim_ns-value['stamp_ns'])/1e9,
                             'max_age_s':MAX_AGE_S,'consecutive_samples':self.samples,
                             'missing_sequences':self.missed},
                'scope':'fresh public model telemetry; not diagnosis, local inspection or action authorization'}
