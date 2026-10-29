"""Real private station/owner/filesystem with controlled robot and capture inputs."""
from orbital.inspection_worker import OwnerLock
from orbital.machine_registry import MachineRegistry
from orbital.maintenance_journal import Journal, prepare_configured_scenario
from orbital.maintenance_owner import MaintenanceOwner
from orbital.scenario_input import PreparedInput
from orbital.station_truth import StationTruth
from orbital.station_evaluation import evaluate
from test_maintenance_restore import RestoreFixture


class StationFixture(RestoreFixture):
    def attach(self, profile='recoverable', seed=0):
        self.assignment=prepare_configured_scenario(self.root,profile,seed)
        self.lock=OwnerLock(self.root,service='station-service');self.addCleanup(self.lock.close)
        self.journal=Journal(self.lock);self.addCleanup(self.journal.close)
        self.configuration=PreparedInput(self.journal,MachineRegistry.load(self.root));self.addCleanup(self.configuration.close)
        old=self.owner
        self.model=self.configuration.model()
        self.owner=MaintenanceOwner(self.root,self.model,self.live,self.hold,old.worker,
            now=old.now,sim_ns=old.sim_ns,on_result=self.results.append,
            inspection_factory=old.inspection_factory,load=self.load,journal=self.journal)
        self.truth=StationTruth(self.lock,self.journal,self.configuration,self.model,now=old.now)
        self.addCleanup(self.truth.close);self.model.truth=self.truth
        self.request={**self.request,'run_id':self.owner.run_id,
            'arguments':{**self.request['arguments'],'station_run_id':self.model.run_id}}
        self.path=self.root/'runtime/station-service/station-observations'/(self.assignment['trial_id']+'.jsonl')

    def close_station(self, complete=True):
        self.truth.finish(complete)
        self.configuration.close();self.journal.close();self.lock.close()

    def score(self):
        self.close_station()
        return evaluate(self.root,self.assignment['trial_id'])
