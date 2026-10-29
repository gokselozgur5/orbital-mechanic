"""Private setup must survive actual ownership/effect boundaries without leaking."""
import uuid

from orbital.inspection_worker import OwnerLock
from orbital.machine_registry import MachineRegistry
from orbital.maintenance_journal import Journal, prepare_configured_scenario, read_history
from orbital.maintenance_owner import MaintenanceOwner
from orbital.scenario_input import PreparedInput
from test_maintenance_reset import ResetFixture


class ConfiguredMaintenanceTests(ResetFixture):
    def setUp(self):
        super().setUp()
        self.assignment = prepare_configured_scenario(self.root, 'recoverable', 0)
        self.lock = OwnerLock(self.root, service='station-service'); self.addCleanup(self.lock.close)
        self.journal = Journal(self.lock); self.addCleanup(self.journal.close)
        self.configuration = PreparedInput(self.journal, MachineRegistry.load(self.root)); self.addCleanup(self.configuration.close)
        old = self.owner
        self.model = self.configuration.model()
        self.owner = MaintenanceOwner(self.root, self.model, self.live, self.hold, old.worker,
            now=old.now, sim_ns=old.sim_ns, on_result=self.results.append,
            inspection_factory=old.inspection_factory, load=self.load, journal=self.journal)
        self.request = {**self.request, 'run_id':self.owner.run_id,
            'arguments':{**self.request['arguments'], 'station_run_id':self.model.run_id}}

    def test_configured_model_is_the_durable_owner_model_used_for_isolation_and_reset(self):
        self.sample(.05); self.sample(.05)
        self.start(self.request); self.apply(); self.sample(.05); self.sample(.05)
        isolated = self.finish()
        reset = {**self.request, 'op':'reset', 'action_id':uuid.uuid4().hex, 'command_id':uuid.uuid4().hex,
                 'arguments':{**self.request['arguments'], 'control_revision':1,
                              'isolation_action_id':isolated['action_id']}}
        self.start(reset); self.apply(); self.sample(.05); self.sample(.05)
        result = self.finish()
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(self.model.control_revision, 2)
        self.assertEqual(self.model.station.machines['pump'].fault.value, 'none')
        history = read_history(self.root)
        current = history.scenarios[history.current]
        self.assertEqual(current['configuration_sha256'], self.configuration.digest)
        self.assertEqual(current['claim']['station_run_id'], result['station_run_id'])
        self.assertEqual(history.actions[reset['action_id']]['result'], result)

    def test_late_private_input_change_cannot_reach_actual_station_effect(self):
        self.sample(.05); self.sample(.05); self.start(self.request)
        path = self.root/'runtime/station-service/scenario-inputs'/self.configuration.name
        path.write_bytes(path.read_bytes()+b' ')
        self.apply()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertEqual(self.model.control_revision, 0)
        action = read_history(self.root).actions[self.request['action_id']]
        self.assertTrue(action['rejected'])
        self.assertIsNone(action['effect'])
        self.assertTrue(self.configuration.broken)
