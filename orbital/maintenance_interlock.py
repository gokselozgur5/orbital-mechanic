"""Fresh exact guard inhibition and navigation quiescence for one station owner."""
import copy
import uuid

from orbital.inspection_protocol import canonical, decode, finite
from orbital.maintenance_hold import identity, state as hold_state
from orbital.navigation_client import NavigationClient


class HoldEvidence:
    def __init__(self, *, now, sim_ns, publish, authority):
        self.now, self.sim_ns, self.publish, self.authority = now, sim_ns, publish, authority
        self.guard = self.nav = None
        self.guard_at = self.nav_at = None
        self.guard_samples = self.nav_samples = 0
        self.nav_reader = NavigationClient(now)
        self.failure = None
        self.source_seen = False
        self.last_wall = self.last_sim = None
        self.owner = None
        self.release_at = None

    def clocks(self):
        now, sim = finite(self.now()), self.sim_ns()
        if type(sim) is int and sim == 0 and self.last_sim is None:
            raise ValueError('awaiting_simulation_clock')
        if (type(sim) is not int or not 0 < sim < 2**63
                or (self.last_wall is not None and now < self.last_wall)
                or (self.last_sim is not None and sim < self.last_sim)):
            self.failure = self.failure or 'maintenance_clock_changed'
        self.last_wall, self.last_sim = now, sim
        if self.failure:
            raise ValueError(self.failure)
        return now, sim

    def on_guard(self, text):
        try:
            now, sim = self.clocks()
            self.authority()
            self.source_seen = True
            value = decode(text)
            if type(value.get('latched')) is not bool:
                raise ValueError('invalid_guard_state')
            hold = hold_state(value['maintenance'])
            if not 0 <= sim-hold['sim_stamp_ns'] <= 200000000:
                raise ValueError('stale_guard_state')
            if hold['active'] is not None and value['latched'] is not True:
                raise ValueError('hold_not_latched')
            if self.guard is not None:
                old = self.guard['maintenance']
                if hold['guard_run_id'] != old['guard_run_id']:
                    raise ValueError('guard_restarted')
                if hold['sequence'] <= old['sequence'] or hold['sim_stamp_ns'] < old['sim_stamp_ns']:
                    raise ValueError('replayed_guard_state')
                if hold['sim_stamp_ns'] == old['sim_stamp_ns']:
                    return  # A frozen simulation clock does not renew evidence.
                if now-self.guard_at >= .2 or hold['sim_stamp_ns']-old['sim_stamp_ns'] > 200000000:
                    self.guard_samples = 0
            self.guard, self.guard_at = value, now
            self.guard_samples += 1
        except (ValueError, KeyError, TypeError, OSError, RuntimeError):
            if self.source_seen:
                self.failure = self.failure or 'guard_evidence_unavailable'

    def on_navigation(self, text):
        try:
            now, sim = self.clocks()
            self.authority()
            self.source_seen = True
            self.nav_reader.observe(text)
            if self.nav_reader.failure:
                raise ValueError('invalid_navigation_state')
            value = self.nav_reader.state
            stamp, sequence = value['sim_stamp_ns'], value['maintenance_state_sequence']
            if (type(stamp) is not int or not 0 <= sim-stamp <= 200000000
                    or type(sequence) is not int or not 0 < sequence < 2**63
                    or type(value['maintenance_quiescent']) is not bool
                    or value['maintenance_error'] is not False):
                raise ValueError('invalid_navigation_hold')
            if value['maintenance_hold'] is not None:
                identity(value['maintenance_hold'])
            if self.nav is not None:
                if sequence <= self.nav['maintenance_state_sequence'] or stamp < self.nav['sim_stamp_ns']:
                    raise ValueError('replayed_navigation_state')
                if stamp == self.nav['sim_stamp_ns']:
                    return
                if now-self.nav_at >= .2 or stamp-self.nav['sim_stamp_ns'] > 200000000:
                    self.nav_samples = 0
            self.nav, self.nav_at = copy.deepcopy(value), now
            self.nav_samples += 1
        except (ValueError, KeyError, TypeError, OSError, RuntimeError):
            if self.source_seen:
                self.failure = self.failure or 'navigation_evidence_unavailable'

    def fresh(self):
        now, sim = self.clocks()
        self.authority()
        if self.guard is None or self.nav is None or min(self.guard_samples, self.nav_samples) < 2:
            return False
        return (0 <= now-self.guard_at < .2 and 0 <= now-self.nav_at < .2
                and 0 <= sim-self.guard['maintenance']['sim_stamp_ns'] <= 200000000
                and 0 <= sim-self.nav['sim_stamp_ns'] <= 200000000)

    def available(self):
        return self.fresh() and self.owner is None and self.guard['maintenance']['active'] is None

    def begin(self, owner):
        if not self.available():
            raise ValueError('maintenance_hold_unavailable')
        self.owner = identity(owner)
        self.release_at = None
        self.send('acquire')

    def send(self, operation):
        self.publish(canonical({'schema_version': 1, 'guard_run_id': self.guard['maintenance']['guard_run_id'],
                                'command_id': uuid.uuid4().hex, 'op': operation, 'identity': self.owner}))

    def held(self):
        if not self.fresh() or self.owner is None:
            return False
        hold = self.guard['maintenance']
        return (hold['active'] == self.owner and self.guard['latched'] is True
                and self.nav['maintenance_guard_run_id'] == hold['guard_run_id']
                and self.nav['maintenance_hold'] == self.owner and self.nav['maintenance_quiescent'] is True
                and self.nav['action_state'] == 'idle' and self.nav['target'] is None
                and self.nav['healthy'] is True and self.nav['guard_latched'] is True)

    def release(self):
        if not self.fresh() or self.owner is None:
            return False
        hold = self.guard['maintenance']
        idle = (self.nav['action_state'] == 'idle' and self.nav['target'] is None
                and self.nav['healthy'] is True and self.nav['guard_latched'] is True)
        if (hold['active'] is None and hold['last_released'] == self.owner
                and self.guard['latched'] is True and self.nav['maintenance_hold'] is None and idle):
            self.owner = None
            return True
        if self.held() or (hold['active'] is None and idle):
            now, _ = self.clocks()
            if self.release_at is None or now-self.release_at >= .25:
                self.send('release')
                self.release_at = now
        return False
