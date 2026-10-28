import copy
import json

from orbital.machine_registry import MachineRegistry
from orbital.machine_telemetry import MachineTelemetry
from orbital.machines import Fault
from orbital.proximity import Unavailable
from orbital.telemetry_observation import TelemetryEvidence
from test_machine_registry import RegistryFixture


class TelemetryObservationTests(RegistryFixture):
    def setUp(self):
        super().setUp()
        self.registry=MachineRegistry.load(self.root)
        self.model=MachineTelemetry(self.registry)

    def pair(self):
        value=TelemetryEvidence(self.registry)
        for stamp,wall in ((10**9,10.),(1500000000,10.5)):
            self.assertTrue(value.observe(json.dumps(self.model.sample(stamp)),wall,stamp))
        return value

    def test_real_model_projection_for_every_fault_is_preserved_without_a_diagnosis(self):
        for fault in Fault:
            self.model=MachineTelemetry(self.registry,fault)
            value=self.pair();result=value.measure(10.51,1510000000)
            self.assertTrue(result['valid'])
            for key,expected in self.model.station.observe().items():
                self.assertEqual(result['telemetry'][key],expected)
            self.assertEqual(result['freshness']['consecutive_samples'],2)
            for secret in ('fault','scenario','reset_count','recovered','latched_controller','mechanical_jam'):
                self.assertNotIn(secret,json.dumps(result))

    def test_one_packet_is_not_an_advancing_stream(self):
        value=TelemetryEvidence(self.registry)
        value.observe(json.dumps(self.model.sample(10**9)),10.,10**9)
        with self.assertRaisesRegex(Unavailable,'pair'):value.measure(10.1,1100000000)

    def test_freshness_boundaries_apply_to_both_receipt_and_simulation_time(self):
        for wall,sim in ((11.5,1500000000),(10.6,2500000000)):
            self.model=MachineTelemetry(self.registry);value=self.pair()
            self.assertTrue(value.measure(11.499999,2499999999)['valid'])
            # Use a separate pair to isolate expiry from clock regression.
            self.model=MachineTelemetry(self.registry);value=self.pair()
            with self.subTest(wall=wall,sim=sim),self.assertRaises(Unavailable):value.measure(wall,sim)

    def test_expired_history_needs_two_new_samples(self):
        value=self.pair()
        with self.assertRaises(Unavailable):value.measure(11.5,2500000000)
        for stamp,wall in ((2600000000,11.6),(3100000000,12.1)):
            value.observe(json.dumps(self.model.sample(stamp)),wall,stamp)
            if stamp==2600000000:
                with self.assertRaises(Unavailable):value.measure(wall,stamp)
        self.assertTrue(value.measure(12.11,3110000000)['valid'])

    def test_sequence_gap_discards_pair_and_reports_missing_samples(self):
        value=self.pair()
        self.model.sample(2*10**9)
        value.observe(json.dumps(self.model.sample(2500000000)),11.1,2500000000)
        with self.assertRaises(Unavailable):value.measure(11.11,2510000000)
        value.observe(json.dumps(self.model.sample(3*10**9)),11.6,3*10**9)
        result=value.measure(11.61,3010000000)
        self.assertEqual(result['freshness']['missing_sequences'],1)
        self.assertEqual(result['freshness']['consecutive_samples'],2)

    def test_replay_duplicate_stamp_and_model_clock_mismatch_are_sticky(self):
        for kind in ('sequence','stamp','model_time','run'):
            self.model=MachineTelemetry(self.registry);value=self.pair()
            message=self.model.sample(2*10**9)
            if kind=='sequence':message['sequence']=2
            elif kind=='stamp':message['stamp_ns']=1500000000
            elif kind=='model_time':message['time_s']=.5
            else:message['run_id']='f'*32
            self.assertFalse(value.observe(json.dumps(message),11.,2*10**9))
            self.assertFalse(value.observe(json.dumps(self.model.sample(2500000000)),11.5,2500000000))
            with self.subTest(kind=kind),self.assertRaises(Unavailable):value.measure(11.51,2510000000)

    def test_clock_regression_is_not_hidden_by_a_later_good_sample(self):
        for wall,sim in ((10.4,1600000000),(10.6,1400000000)):
            self.model=MachineTelemetry(self.registry);value=self.pair()
            with self.assertRaisesRegex(Unavailable,'regressed'):value.measure(wall,sim)
            self.assertFalse(value.observe(json.dumps(self.model.sample(2*10**9)),11.,2*10**9))

    def test_queued_packets_cannot_make_a_frozen_query_clock_fresh(self):
        value=TelemetryEvidence(self.registry)
        value.tick(10.,1800000000)
        for stamp,wall in ((10**9,10.1),(1500000000,10.2)):
            value.observe(json.dumps(self.model.sample(stamp)),wall,1800000000)
        with self.assertRaisesRegex(Unavailable,'advancing'):value.measure(10.21,1800000000)
        with self.assertRaisesRegex(Unavailable,'advancing'):value.measure(10.22,1900000000)

    def test_stale_and_future_packets_never_renew_a_lease(self):
        message=json.dumps(self.model.sample(10**9))
        for sim in (999999999,2*10**9):
            value=TelemetryEvidence(self.registry)
            self.assertFalse(value.observe(message,10.,sim))
            with self.assertRaisesRegex(Unavailable,'stale_or_future'):value.measure(10.1,sim)

    def test_missing_unknown_nonfinite_and_wrong_typed_measurements_are_refused(self):
        original=self.model.sample(10**9)
        mutations=[('missing_machine',lambda v:v['machines'].pop('cooler')),
                   ('extra_machine',lambda v:v['machines'].update(other=v['machines']['pump'])),
                   ('missing_value',lambda v:v['machines']['pump'].pop('current_a')),
                   ('unknown',lambda v:v['machines']['pump'].update(current_a=None)),
                   ('nan',lambda v:v['machines']['pump'].update(current_a=float('nan'))),
                   ('infinite',lambda v:v.update(voltage_v=float('inf'))),
                   ('boolean_value',lambda v:v['machines']['pump'].update(current_a=True)),
                   ('negative_current',lambda v:v['machines']['pump'].update(current_a=-.1)),
                   ('impossible_temperature',lambda v:v['machines']['pump'].update(temperature_c=-274)),
                   ('numeric_power',lambda v:v['machines']['pump'].update(powered=1)),
                   ('fault_field',lambda v:v['machines']['pump'].update(fault='mechanical_jam')),
                   ('scenario_field',lambda v:v.update(scenario='latched_controller'))]
        for name,change in mutations:
            packet=copy.deepcopy(original);change(packet);value=TelemetryEvidence(self.registry)
            with self.subTest(name=name):
                self.assertFalse(value.observe(json.dumps(packet),10.,10**9))
                self.assertEqual(value.failure,'invalid_telemetry')

    def test_catalog_units_source_and_header_cannot_be_substituted(self):
        original=self.model.sample(10**9)
        for key,replacement in (('schema_version',True),('schema_version',2),('run_id','not-a-run'),
                                ('sequence',True),('sequence',0),('sequence',2**63),
                                ('stamp_ns',1e9),('stamp_ns',2**63),('time_s',True),
                                ('source','other'),('units',{}),('registry',{'identity':self.registry.identity})):
            packet=copy.deepcopy(original);packet[key]=replacement;value=TelemetryEvidence(self.registry)
            with self.subTest(key=key,value=replacement):
                self.assertFalse(value.observe(json.dumps(packet),10.,10**9))
        for path in ('control','unknown','boolean'):
            packet=copy.deepcopy(original)
            if path=='control':packet['registry']['machines']['pump']['control_xy_m'][0]+=1
            elif path=='unknown':packet['registry']['secret']='hidden'
            else:packet['registry']['schema_version']=True
            value=TelemetryEvidence(self.registry)
            self.assertFalse(value.observe(json.dumps(packet),10.,10**9))

    def test_malformed_bounded_json_cannot_leak_input_into_errors(self):
        original=json.dumps(self.model.sample(10**9))
        packets=[None,42,'{','[]',' '*16385,'{"a":1,"a":2}',
                 '{"hidden":'+('['*1500)+'0'+(']'*1500)+'}',
                 original.replace('"sequence": 1','"sequence": 1, "sequence": 2'),
                 original.replace('"voltage_v": 28.0','"voltage_v": 1e9999'),
                 '\ud800','{"x":"'+'é'*8192+'"}']
        for packet in packets:
            value=TelemetryEvidence(self.registry)
            self.assertFalse(value.observe(packet,10.,10**9))
            self.assertEqual(value.failure,'invalid_telemetry')

    def test_result_mutation_cannot_change_later_public_evidence(self):
        value=self.pair();result=value.measure(10.51,1510000000)
        result['telemetry']['machines']['pump']['current_a']=0
        result['telemetry']['registry']['identity']='fake'
        later=value.measure(10.52,1520000000)
        self.assertGreater(later['telemetry']['machines']['pump']['current_a'],0)
        self.assertEqual(later['telemetry']['registry']['identity'],self.registry.identity)

    def test_finite_noisy_values_are_preserved_without_model_based_correction(self):
        value=TelemetryEvidence(self.registry)
        for stamp,wall in ((10**9,10.),(1500000000,10.5)):
            packet=self.model.sample(stamp)
            packet['machines']['pump'].update(temperature_c=39.8,current_a=7.9,vibration_au=1.7)
            value.observe(json.dumps(packet),wall,stamp)
        result=value.measure(10.51,1510000000)
        self.assertEqual(result['telemetry']['machines']['pump'],
                         {'powered':True,'temperature_c':39.8,'current_a':7.9,'vibration_au':1.7})
        self.assertNotIn('diagnosis',result)
        self.assertNotIn('healthy',result)

    def test_real_model_restart_cannot_continue_an_old_query(self):
        value=self.pair()
        replacement=MachineTelemetry(self.registry,Fault.JAM)
        self.assertFalse(value.observe(json.dumps(replacement.sample(2*10**9)),11.,2*10**9))
        with self.assertRaisesRegex(Unavailable,'telemetry_run_changed'):value.measure(11.01,2010000000)

    def test_model_elapsed_time_is_distinct_from_absolute_source_time(self):
        value=TelemetryEvidence(self.registry)
        for stamp,wall in ((50*10**9,10.),(50500000000,10.5)):
            value.observe(json.dumps(self.model.sample(stamp)),wall,stamp)
        result=value.measure(10.51,50510000000)
        self.assertEqual(result['telemetry']['time_s'],.5)
        self.assertEqual(result['telemetry']['stamp_ns'],50500000000)
