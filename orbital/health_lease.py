"""Consume health reports without letting replay/recovery erase a stop obligation."""
import math
import uuid


class HealthLease:
    TIMEOUT = .25
    MAX_RETIRED_RUNS = 64

    def __init__(self):
        self.run_id = None
        self.epoch = None
        self.sequence = None
        self.sim_time = None
        self.received_at = None
        self.last_wall = None
        self.retired_runs = set()
        self.healthy = False
        self.stationary = False
        self.reason = 'startup'
        self.pending_stop = True
        self.generation = 0

    def invalidate(self, reason):
        if self.healthy or not self.pending_stop or reason != self.reason:
            self.generation += 1
        self.healthy = False
        self.stationary = False
        self.pending_stop = True
        self.reason = reason

    def tick(self, now):
        if not math.isfinite(now) or (self.last_wall is not None and now < self.last_wall):
            self.invalidate('invalid_wall_clock')
            return False
        self.last_wall = now
        if self.healthy and (self.received_at is None or now-self.received_at >= self.TIMEOUT):
            self.invalidate('health_timeout')
        return self.healthy and not self.pending_stop

    @staticmethod
    def valid_report(report):
        if not isinstance(report, dict):
            return False
        expected={'schema_version','healthy','stationary','reason','sim_time','run_id','epoch','sequence'}
        if set(report) != expected or type(report['schema_version']) is not int or report['schema_version'] != 1:
            return False
        if type(report['healthy']) is not bool or type(report['stationary']) is not bool:
            return False
        if not isinstance(report['reason'],str) or not 0<len(report['reason'])<=128:
            return False
        if any(type(report[k]) is not int or not 0<=report[k]<2**63 for k in ('epoch','sequence')) or report['sequence']==0:
            return False
        if type(report['sim_time']) not in (int,float) or not math.isfinite(report['sim_time']) or report['sim_time']<0:
            return False
        try:
            return isinstance(report['run_id'],str) and str(uuid.UUID(report['run_id']))==report['run_id']
        except (ValueError,AttributeError,TypeError):
            return False

    def observe(self, report, now):
        self.tick(now)
        if not math.isfinite(now) or self.last_wall != now:
            return False
        if not self.valid_report(report):
            self.invalidate('malformed_health')
            return False
        run,epoch,seq,sim=(report[k] for k in ('run_id','epoch','sequence','sim_time'))
        if run == self.run_id and self.sequence is not None and seq <= self.sequence:
            self.invalidate('replayed_health')
            return False
        if run in self.retired_runs:
            self.invalidate('retired_health_source')
            return False
        if self.run_id != run:
            if len(self.retired_runs)>=self.MAX_RETIRED_RUNS:
                self.invalidate('health_source_history_full')
                return False
            if self.run_id is not None:self.retired_runs.add(self.run_id)
            self.run_id=run;self.epoch=epoch;self.sequence=None;self.sim_time=None
            self.invalidate('health_source_changed')
        elif epoch < self.epoch:
            self.invalidate('old_health_epoch')
            return False
        elif epoch > self.epoch:
            self.epoch=epoch
            self.sequence=None;self.sim_time=None
            self.invalidate('health_epoch_changed')
        if self.sequence is not None and seq<=self.sequence:
            self.invalidate('replayed_health')
            return False
        self.sequence=seq
        if self.sim_time is not None and sim<self.sim_time:
            self.invalidate('simulation_regressed_without_epoch')
            return False
        if not report['healthy']:
            self.sim_time=sim
            self.invalidate(report['reason'])
            return False
        if self.sim_time is not None and sim==self.sim_time:
            return False  # A fresh sequence with a frozen clock cannot renew the lease.
        self.sim_time=sim
        self.received_at=now
        self.healthy=True
        self.stationary=report['stationary']
        self.reason='ready'
        return True

    def acknowledge_stop(self, generation):
        if generation != self.generation:
            return False
        self.pending_stop=False
        return True

    @property
    def stop_required(self):
        return self.pending_stop or not self.healthy
