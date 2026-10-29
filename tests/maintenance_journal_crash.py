"""Owned subprocess cuts at actual persisted station boundaries; controlled sensors."""
import json
import os
from pathlib import Path
import signal
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.inspection_worker import OwnerLock
from orbital.maintenance_journal import Journal, prepare_scenario
from orbital.maintenance_owner import MaintenanceOwner
from test_machine_registry import RegistryFixture
from test_maintenance_cancellation import CancellationFixture


def build(root):
    def registry_setup(self):
        self.root = root
        self.config_path, self.world_path = root/'config/known-map.json', root/'worlds/maintenance.sdf'
        self.config = json.loads(self.config_path.read_text())
    fixture = CancellationFixture()
    with patch.object(RegistryFixture,'setUp',registry_setup): fixture.setUp()
    prepare_scenario(root)
    lock = OwnerLock(root,service='station-service')
    fixture.addCleanup(lock.close)
    journal = Journal(lock)
    fixture.addCleanup(journal.close)
    old = fixture.owner
    fixture.owner = MaintenanceOwner(root,fixture.model,fixture.live,fixture.hold,old.worker,
        now=old.now,sim_ns=old.sim_ns,on_result=fixture.results.append,
        inspection_factory=old.inspection_factory,load=fixture.load,journal=journal)
    fixture.request['run_id'] = fixture.owner.run_id
    fixture.journal, fixture.lock = journal, lock
    return fixture


def main():
    root, cut = Path(sys.argv[1]), sys.argv[2]
    if cut not in ('admit','intent','mutated','applied','terminal'): raise ValueError('invalid_cut')
    fixture = build(root)
    record = fixture.journal.record
    def stop_here():
        print(json.dumps({'cut':cut,'pid':os.getpid(),'mission_id':fixture.request['mission_id'],
                'action_id':fixture.request['action_id'],'owner_run_id':fixture.owner.run_id,
                'station_run_id':fixture.model.run_id,'control_revision':fixture.model.control_revision,
                'powered':fixture.model.station.machines['pump'].powered,
                'published_results':len(fixture.results),'scenario_id':fixture.journal.claimed['scenario_id']}),flush=True)
        os.kill(os.getpid(),signal.SIGSTOP)
    def stop_at(kind,value):
        record(kind,value)
        if kind == cut: stop_here()
    fixture.journal.record = stop_at
    if cut == 'mutated':
        intervene = fixture.model.station.intervene
        def mutation(*args,**kwargs):
            value = intervene(*args,**kwargs)
            stop_here()  # Actual state changed; applied metadata has not been recorded.
            return value
        fixture.model.station.intervene = mutation
    try: fixture.isolated()
    finally: fixture.doCleanups()


if __name__ == '__main__': main()
