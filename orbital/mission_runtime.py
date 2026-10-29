"""Scripted mission over persistent public clients and a durable gateway journal.

The transport supplies actual receipt clocks and sampled source identities.
Server interlocks retain actuator authority; this driver never supplies geometry,
hidden fault state or a fabricated arrival/result to the servers.
"""
import copy
import uuid

from orbital.inspection_client import InspectionClient
from orbital.maintenance_client import MaintenanceClient
from orbital.inspection_protocol import canonical, finite, identifier
from orbital.mission_policy import MissionPolicy, POLICY_VERSION
from orbital.mission_state import MissionState
from orbital.navigation_client import NavigationClient
from orbital.proximity import Unavailable
from orbital.telemetry_observation import TelemetryEvidence


class MissionRuntime:
    def __init__(self, registry, transport, journal, *, mission_id=None, timeout_s=600.,
                 stop_requested=lambda:False):
        self.registry,self.transport,self.journal=registry,transport,journal
        self.now,self.sim_ns=transport.now,transport.sim_ns
        self.mission_id=identifier(uuid.uuid4().hex if mission_id is None else mission_id)
        self.deadline=finite(self.now())+finite(timeout_s,1.,600.)
        self.stop_requested=stop_requested
        self.mission=self.policy=None
        self.started=self.cancelling=False
        self.failure=None
        self.command_client=None
        self.cancellation_attempted=False
        self.navigation=NavigationClient(self.now)
        self.telemetry=TelemetryEvidence(registry)
        self.inspection=InspectionClient(lambda text:self._publish('inspection',text),
            self._client_spin,lambda:transport.fence('inspection'),now=self.now)
        self.maintenance=MaintenanceClient(lambda text:self._publish('maintenance',text),
            self._client_spin,lambda:transport.fence('maintenance'),now=self.now)
        self.clients={'inspection':self.inspection,'maintenance':self.maintenance}
        if journal.active is not None:
            raise ValueError('unfinished_mission_requires_reconciliation')
        transport.install({
            '/station/telemetry':self._telemetry,'/navigation/state':self._navigation,
            **{f'/{family}/{kind}':getattr(client,'observe_'+kind)
                for family,client in self.clients.items() for kind in ('state','reply','result')}})

    def _record(self, kind, data):
        return self.journal.append(self.mission_id,kind,data)

    def _telemetry(self, text):
        wall,sim=self.now(),self.sim_ns()
        try:
            if not self.telemetry.observe(text,wall,sim):
                raise ValueError('gateway_telemetry_invalid')
            if self.policy is not None:
                # Preserve the actual callback receipt time before journal I/O.
                self.policy.observe_telemetry(text)
            if self.started:
                self._record('telemetry',{'text':text,'received_wall_s':wall,'sim_ns':sim})
        except (ValueError,OSError,TypeError,OverflowError) as error:
            self.failure=self.failure or 'gateway_telemetry_unavailable'

    def _navigation(self, text):
        self.navigation.observe(text)
        if self.command_client is not None:
            self.command_client.observe(text)

    def _poll(self, duration=.02, *, cleanup_family=None):
        self.transport.spin(max(0.,min(.02,duration)))
        self.transport.fence(cleanup_family)
        if self.mission is not None:
            self.mission.tick()
            if self.stop_requested():
                self.mission.cancel()
        if cleanup_family is not None:
            return
        self.journal.verify()
        if self.failure or self.navigation.failure or any(c.failure for c in self.clients.values()):
            raise ValueError('gateway_observation_failed')
        self.telemetry.tick(self.now(),self.sim_ns())
        if self.started and self.telemetry.candidate is None:
            raise ValueError('gateway_telemetry_expired')
        if self.stop_requested() and self.mission is None:
            raise ValueError('mission_cancelled_before_admission')

    def _client_spin(self, duration):
        self._poll(duration)
        if self.mission.stop_reason and not self.cancelling:
            self.cancelling=True
            # Existing clients perform one exact cancellation, never retransmit.
            raise KeyboardInterrupt

    def _fresh_sources(self):
        """Recheck receipt ages after all file/graph reads, without further I/O."""
        wall,sim=self.now(),self.sim_ns()
        n=self.navigation
        if (not n.fresh() or not 0<=sim-n.state.get('sim_stamp_ns',-1)<300000000
                or any(c.failure or c.latest is None or c.received is None
                       or not 0<=wall-c.received<.3 for c in self.clients.values())
                or not 0<=sim-self.maintenance.latest['sim_stamp_ns']<300000000):
            raise ValueError('gateway_owner_state_expired')
        return wall,sim

    def _publish(self, family, text):
        """Bind the actual serialized publication to the reserved mission step."""
        from orbital import inspection_protocol, maintenance_protocol
        protocol=inspection_protocol if family=='inspection' else maintenance_protocol
        value=protocol.command(text)
        step=self.mission.pending
        if step is None or step['family']!=family:
            raise ValueError('gateway_has_no_matching_reservation')
        original=step['request']
        if value['op']=='cancel':
            if any(value[k]!=original[k] for k in ('run_id','mission_id','action_id')):
                raise ValueError('gateway_cancel_identity_mismatch')
            if self.cancellation_attempted:
                raise ValueError('gateway_cancel_already_attempted')
            if not step['possibly_sent']:
                raise ValueError('gateway_original_not_published')
            self.cancellation_attempted=True
            try:self._record('cancel_intent',{'family':family,'command':value})
            except (ValueError,OSError):self.failure=self.failure or 'gateway_journal_unavailable'
            self.transport.fence(family)
        else:
            if value!=original or step['possibly_sent']:
                raise ValueError('gateway_command_changed_or_repeated')
            self.registry.verify_current()
            self.transport.fence()
            self._fresh_sources()
            self.telemetry.measure(self.now(),self.sim_ns())
            self.mission.tick()
            if self.mission.stop_reason or self.stop_requested():
                self.mission.cancel()
                raise ValueError('gateway_mission_stopping')
            step['possibly_sent']=True
        self.transport.publish(family,text)

    def _wait(self, predicate, deadline):
        while self.now()<deadline:
            self._poll(min(.02,deadline-self.now()))
            if self.mission is not None and self.mission.stop_reason:
                raise ValueError('gateway_mission_stopping')
            if predicate():return
        raise ValueError('gateway_wait_deadline')

    def start(self):
        def ready():
            try:
                public=self.telemetry.measure(self.now(),self.sim_ns())['telemetry']
            except Unavailable:
                return False
            n,i,m=self.navigation,self.inspection,self.maintenance
            return (n.fresh() and n.state['action_state']=='idle' and n.state['healthy']
                and n.state['guard_latched'] is True and all(c.state_count>=2 and c.valid() for c in (i,m))
                and i.latest['active'] is None and m.latest['active'] is None
                and m.latest['station_run_id']==public['run_id']
                and m.latest['control_revision']==public['control']['revision'])
        self._wait(ready,min(self.deadline,self.now()+10.))
        self.registry.verify_current()
        self.transport.fence()
        self._fresh_sources()
        baseline=self.telemetry.measure(self.now(),self.sim_ns())['telemetry']
        self.mission=MissionState(self.registry,self.mission_id,baseline,now=self.now,sim_ns=self.sim_ns,
            timeout_s=self.deadline-self.now())
        self.policy=MissionPolicy(self.mission)
        self._record('start',{'registry_identity':self.registry.identity,'station_run_id':baseline['run_id'],
            'policy_version':POLICY_VERSION,'baseline':baseline,'started_wall_s':self.mission.started,
            'started_sim_ns':self.mission.started_sim,'deadline_wall_s':self.mission.deadline})
        self.started=True
        self._wait(lambda:len(self.policy.samples)>=2,min(self.deadline,self.now()+2.))

    def _nav_command(self, operation, *, machine=None, target=None):
        client=self.command_client=NavigationClient(self.now)
        deadline=min(self.deadline,self.now()+5.) if operation!='cancel' else self.now()+2.
        try:
            while not client.fresh() and self.now()<deadline:
                self._poll(cleanup_family='navigation' if operation=='cancel' else None)
            if operation!='cancel' and (self.mission.stop_reason or self.stop_requested()):
                raise ValueError('gateway_mission_stopping')
            client.prepare(operation,waypoint=machine,target=target)
            request=copy.deepcopy(client.command)
            if operation=='request':self.mission.begin_navigation(request)
            elif operation=='arm':self.mission.begin_navigation_arm(request)
            else:
                if self.cancellation_attempted:raise ValueError('gateway_cancel_already_attempted')
                self.cancellation_attempted=True
            try:self._record('cancel_intent' if operation=='cancel' else 'command',{'family':'navigation','command':request})
            except (ValueError,OSError):
                if operation!='cancel':raise
                self.failure=self.failure or 'gateway_journal_unavailable'
            if operation!='cancel':
                self.registry.verify_current()
            self.transport.fence('navigation' if operation=='cancel' else None)
            if operation!='cancel':
                self.mission.tick()
                if self.mission.stop_reason or self.stop_requested() or self.now()>=deadline:
                    raise ValueError('gateway_mission_stopping')
            client.publish_once(lambda text:self.transport.publish('navigation',text))
            while self.now()<deadline:
                self._poll(cleanup_family='navigation' if operation=='cancel' else None)
                ack=client.acknowledgment()
                if ack is not None:
                    receipt=client.receipt(operation,acknowledgment=ack)
                    break
                if operation!='cancel' and self.mission.stop_reason:break
            else:receipt=None
            if 'receipt' not in locals() or receipt is None:
                receipt=client.receipt(operation,error='gateway_acknowledgement_unconfirmed')
        except (ValueError,OSError,RuntimeError,KeyboardInterrupt) as error:
            receipt=client.receipt(operation,error='gateway_command_unavailable')
        finally:
            self.command_client=None
        if client.command is not None:
            if operation=='request' and self.mission.pending is not None:
                self.mission.navigation_receipt(receipt)
            elif operation=='arm' and self.mission.pending is not None:
                self.mission.navigation_arm_receipt(receipt)
            self._record('receipt',{'family':'navigation','receipt':receipt,'wall_s':self.now(),'sim_ns':self.sim_ns()})
        if receipt['delivery']!='acknowledged':
            raise ValueError('gateway_navigation_command_unconfirmed')
        return receipt

    def navigate(self, machine):
        self.cancellation_attempted=False
        receipt=self._nav_command('request',machine=machine)
        self._nav_command('arm',target=receipt['target'])
        deadline=min(self.deadline,self.now()+120.)
        while self.mission.pending is not None and self.now()<deadline:
            self._poll()
            if self.mission.stop_reason:break
            self.mission.navigation_state(self.navigation.state,self.navigation.received_at)
        if self.mission.pending is not None:
            self.mission.fail('navigation_step_deadline_or_cancellation')
            raise ValueError('gateway_navigation_unresolved')

    def action(self, operation, machine):
        family='inspection' if operation=='inspect' else 'maintenance'
        client=self.clients[family]
        self._wait(lambda:client.valid() and client.latest['ready'],min(self.deadline,self.now()+3.))
        left=self.deadline-self.now()-2.
        if left<.1:raise ValueError('gateway_action_budget_exhausted')
        args={'machine_id':machine,'timeout_s':min(5. if operation=='inspect' else 30.,left)}
        if family=='maintenance':
            args.update(station_run_id=self.mission.station_run,control_revision=self.mission.control['revision'])
            field={'reset':'isolation_action_id','restore':'reset_action_id'}.get(operation)
            if field:args[field]=self.mission._last_maintenance()['action_id']
        request={'schema_version':1,'run_id':client.latest['run_id'],'mission_id':self.mission_id,
            'action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex,'op':operation,'arguments':args}
        self.mission.begin_action(request)
        self._record('command',{'family':family,'command':request})
        self.cancellation_attempted=self.cancelling=False
        receipt=client.execute(request)
        if not self.mission.pending['possibly_sent']:
            # This gateway owns the publication boundary. The client can report
            # uncertainty after our preflight throws, although transport was
            # never entered. Do not invent a cancellation or server rejection.
            if client.ack is not None or client.terminal is not None:
                raise ValueError('gateway_unsent_receipt_contradiction')
            receipt=client.receipt('not_sent','gateway_original_not_published')
        self.mission.action_receipt(receipt)
        self._record('receipt',{'family':family,'receipt':receipt,'wall_s':self.now(),'sim_ns':self.sim_ns()})
        if self.mission.pending is not None:
            raise ValueError('gateway_action_unresolved')

    def _settle(self):
        """One exact cancel, then bounded observation; no action retransmission."""
        step=self.mission.pending
        if step is None:return
        family=step['family']
        self.cancelling=True
        if not self.cancellation_attempted:
            try:
                if family=='navigation':
                    if step['target'] is not None:self._nav_command('cancel',target=step['target'])
                else:
                    original=step['request']
                    self._publish(family,canonical({**original,'command_id':uuid.uuid4().hex,'op':'cancel','arguments':{}}))
            except (ValueError,RuntimeError,OSError,KeyboardInterrupt):pass
        deadline=self.now()+3.
        while self.mission.pending is not None and self.now()<deadline:
            try:
                self._poll(cleanup_family=family)
                if family=='navigation':
                    self.mission.navigation_state(self.navigation.state,self.navigation.received_at)
                else:
                    client=self.clients[family]
                    if client.terminal is not None and client.valid():
                        receipt=client.receipt('terminal')
                        self.mission.action_receipt(receipt)
                        self._record('receipt',{'family':family,'receipt':receipt,'wall_s':self.now(),'sim_ns':self.sim_ns()})
            except (ValueError,RuntimeError,OSError,KeyboardInterrupt):
                continue

    def quiescence(self):
        self.transport.fence()
        n=self.navigation
        if (not n.fresh() or n.state['action_state']!='idle' or n.state['guard_latched'] is not True
                or not 0<=self.sim_ns()-n.state.get('sim_stamp_ns',-1)<300000000
                or not all(c.valid() and c.latest['active'] is None for c in self.clients.values())
                or self.maintenance.latest['control_revision']!=self.mission.control['revision']):
            raise ValueError('gateway_final_quiescence_unavailable')
        wall,sim=self._fresh_sources()
        return {'navigation':copy.deepcopy(n.state),
                'inspection':copy.deepcopy(self.inspection.latest),
                'maintenance':copy.deepcopy(self.maintenance.latest),
                'received_wall_s':{'navigation':n.received_at,
                    **{name:c.received for name,c in self.clients.items()}},
                'observed_wall_s':wall,'observed_sim_ns':sim,
                'scope':'fresh owner idle/guard-latched reports, not independent physical stopping'}

    def run(self):
        complete=False
        try:
            self.start()
            while self.mission.outcome is None:
                self._poll()
                if self.mission.stop_reason:break
                phase=self.mission.phase
                if phase=='inspect':
                    machine=self.policy.next_machine()['machine_id']
                    self.navigate(machine)
                    if self.mission.phase=='inspect':self.action('inspect',machine)
                elif phase=='diagnose':self._record('decision',self.policy.apply())
                elif phase=='intervene':self.action(self.mission.next_operation,self.mission.arrival)
                elif phase=='verify':
                    if self.policy.high_vibration:self.mission.fail('verification_vibration_contradiction')
                    else:self.mission.verify()
                elif phase=='escalate':self.mission.escalate()
                else:raise ValueError('gateway_unresolved_transition')
        except (ValueError,RuntimeError,OSError,KeyboardInterrupt) as error:
            self.failure=self.failure or 'gateway_mission_failed'
            if self.mission is not None:self.mission.fail(self.failure)
            if self.started:
                try:self._record('error',{'code':self.failure,'type':type(error).__name__})
                except (ValueError,OSError):pass
        if self.started:
            self._settle()
            if self.mission.outcome is not None and self.mission.pending is None:
                try:
                    quiet=self.quiescence()
                    self._record('terminal',{'outcome':self.mission.outcome,'reason':self.mission.reason,
                        'control':self.mission.control,'pending':None,'quiescence':quiet})
                    complete=True
                except (ValueError,RuntimeError,OSError,KeyboardInterrupt):
                    self.failure=self.failure or 'gateway_finalization_unavailable'
        return {'schema_version':1,'complete':complete,'mission_id':self.mission_id,'failure':self.failure,
            'mission':None if self.mission is None else self.mission.snapshot(),
            'policy':None if self.policy is None else self.policy.snapshot(),
            'journal':{'sequence':self.journal.sequence,'last_sha256':self.journal.previous,'active':self.journal.active},
            'scope':'scripted gateway observations and owned server results; not independent native scenario acceptance'}
