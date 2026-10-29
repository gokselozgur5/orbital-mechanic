"""Explicit synthetic Messages recipes over real owned executive fixtures."""
from orbital.planner_provider import ProviderRuntime
from orbital.planner_recording import RecordedProvider
from orbital.planner_schema import FRESH_TOOLS
from planner_gateway_fixture import GatewayFixture

MODEL='orbital-recorded-fixture-v1'


def message(tool,arguments,call_id='call_1'):
    return {'id':'msg_'+call_id,'type':'message','role':'assistant','model':MODEL,
        'content':[{'type':'tool_use','id':call_id,'name':tool,
                    'input':{'basis_id':'$current_basis' if tool in FRESH_TOOLS else None,'arguments':arguments}}],
        'stop_reason':'tool_use','stop_sequence':None,'usage':{'input_tokens':100,'output_tokens':50}}


def step(tool,arguments,call_id='call_1',delay=1.5):
    return {'delay_s':delay,'response':message(tool,arguments,call_id),'bind':True}


def recording(steps):
    return {'schema_version':1,'kind':'planner_messages_recording','provenance':'synthetic-test-fixture',
            'model':MODEL,'steps':steps}


def recovery_steps(delay=1.5):
    return [step(tool,args,'call_'+str(index),delay) for index,(tool,args) in enumerate([
        ('navigate_to',{'waypoint_id':'pump','timeout_s':5.}),
        ('inspect_machine',{'machine_id':'pump','timeout_s':5.}),
        ('isolate_power',{'machine_id':'pump','timeout_s':5.}),
        ('reset_machine',{'machine_id':'pump','timeout_s':5.}),
        ('restore_power',{'machine_id':'pump','timeout_s':5.}),
        ('verify_recovery',{'machine_id':'pump','observation_window_s':10.}),
        ('report_outcome',{'outcome':'recovered','evidence_ids':['$known_evidence'],'summary':'Synthetic replay recovery claim.'})])]


class ProviderFixture(GatewayFixture):
    def make_provider(self,steps=None,**kwargs):
        self.provider=RecordedProvider(recording(recovery_steps() if steps is None else steps))
        self.driver=ProviderRuntime(self.f.model.registry,self.bus,self.mission_journal,
            provider=self.provider,mission_id=self.f.request['mission_id'],timeout_s=60.,
            stop_requested=lambda:self.cancel_requested,**kwargs)
        return self.driver
