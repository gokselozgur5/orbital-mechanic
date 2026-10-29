"""Bounded offline Messages turns over the durable owned tool gateway."""
import copy
import hashlib
import uuid

from orbital.inspection_protocol import canonical
from orbital.planner_basis import frame, compatible, MAX_PLAN_AGE_S
from orbital.planner_messages import (ProviderError, action_id, bounded_integer, digest, message_request,
                                     response, SYSTEM, tools, MAX_REPLY_BYTES)
from orbital.planner_protocol import PlannerError
from orbital.planner_runtime import PlannerRuntime
from orbital.planner_schema import FRESH_TOOLS


class ProviderRuntime(PlannerRuntime):
    execution_mode='recorded-tools-v1'

    def __init__(self,registry,transport,journal,*,provider,mission_text='Station power is dropping. Find the cause and fix it.',
                 max_turns=32,max_output_tokens=2048,max_reported_tokens=100000,max_rejections=2,**kwargs):
        self.provider=provider
        self.config={'model':provider.model,'mission_text':mission_text,
            'max_turns':bounded_integer(max_turns,1,32),'max_output_tokens':bounded_integer(max_output_tokens,16,4096),
            'max_reported_tokens':bounded_integer(max_reported_tokens,1,1000000),
            'max_rejections':bounded_integer(max_rejections,0,4),'recording_sha256':provider.source_sha256,
            'system_sha256':digest(SYSTEM),'tools_sha256':digest(tools())}
        if type(mission_text) is not str or not 1<=len(mission_text)<=1024 or not mission_text.strip():
            raise ProviderError('invalid_recording')
        self.messages=[{'role':'user','content':[{'type':'text','text':mission_text}]}]
        self.provider_failure=None
        super().__init__(registry,transport,journal,**kwargs)

    def start(self):
        super().start()
        self._record('provider_session',{'run_id':self.inputs.run_id,**self.config})

    def dialog(self):
        self.journal.verify()
        return self.journal.provider.get(self.mission_id)

    def _receive(self):
        state=self.dialog();index=len(state['turns'])+1
        if (index>self.config['max_turns'] or state['reported_tokens']>=self.config['max_reported_tokens']
                or state['rejections']>self.config['max_rejections']):raise ProviderError('budget_exhausted')
        self._poll()
        basis=frame(self)
        self.messages[-1]['content'].append({'type':'text','text':canonical({'kind':'planning_context','frame':basis})})
        raw=message_request(self.config['model'],self.config['max_output_tokens'],self.messages)
        started=self.now();deadline=min(self.deadline,started+MAX_PLAN_AGE_S)
        if self.stop_requested() or started>=deadline:raise ProviderError('timeout')
        self._record('provider_turn',{'index':index,'frame':basis,'request_sha256':hashlib.sha256(raw.encode()).hexdigest(),
            'started_wall_s':started,'started_sim_ns':self.sim_ns(),'deadline_wall_s':deadline})
        received=None;error=None;parsed=None
        try:
            if self.now()>=deadline:raise ProviderError('timeout')
            self.provider.submit(raw,self.now())
            while received is None:
                self._poll()
                if self.stop_requested():raise ProviderError('unavailable')
                if self.now()>=deadline:raise ProviderError('timeout')
                received=self.provider.poll(self.now())
            if type(received) is not str or len(received.encode())>MAX_REPLY_BYTES:
                received=None;raise ProviderError('invalid_reply')
            parsed=response(received,model=self.config['model'])
            if parsed['proposal'] is None:raise ProviderError('ended_without_outcome')
            if (parsed['output_tokens']>self.config['max_output_tokens'] or
                    state['reported_tokens']+parsed['reported_tokens']>self.config['max_reported_tokens']):
                raise ProviderError('budget_exhausted')
        except ProviderError as invalid:
            error='unavailable' if received is None and invalid.code=='invalid_recording' else invalid.code
        except (ValueError,OSError,RuntimeError,KeyboardInterrupt):error='unavailable'
        finally:self.provider.cancel()
        if self.now()>=deadline:error='timeout'
        self._record('provider_reply',{'index':index,'raw':received,'error':error,
            'received_wall_s':self.now(),'received_sim_ns':self.sim_ns()})
        if error is not None:raise ProviderError(error)
        self.messages.append({'role':'assistant','content':copy.deepcopy(parsed['message']['content'])})
        return self.dialog()['turns'][-1],parsed['proposal']

    def _dispatch(self,turn,selected):
        state=self.dialog();known=state['calls'].get(selected['id']);replayed=known is not None and known['response'] is not None
        reply=None;error=None
        try:
            if known is not None:
                if known['proposal']!=selected:
                    replayed=False
                    raise ProviderError('identity_conflict')
                if known['response'] is None:raise ProviderError('unresolved')
                reply=copy.deepcopy(known['response']['reply']);error=known['response']['error']
                if reply is not None:reply['replayed']=True
            else:
                if selected['name'] in FRESH_TOOLS and selected['input']['basis_id']!=turn['frame']['basis_id']:
                    raise ProviderError('basis_changed')
                current=None if selected['name']=='stop' else frame(self)
                if selected['name']!='stop' and self.now()>=turn['deadline_wall_s']:
                    raise ProviderError('basis_expired')
                if selected['name'] not in ('observe_station','stop'):
                    compatible(turn['frame'],current,started_wall=turn['started_wall_s'],now=self.now(),
                               started_sim=turn['started_sim_ns'],sim=self.sim_ns())
                wire={'schema_version':1,'kind':'planner_tool_request','run_id':self.inputs.run_id,
                    'mission_id':self.mission_id,'action_id':action_id(self.inputs.run_id,selected['id']),
                    'tool':selected['name'],'arguments':selected['input']['arguments'],
                    'observation_id':current['observation']['observation_id'] if selected['name'] in FRESH_TOOLS else None}
                self._record('provider_binding',{'index':turn['index'],'proposal':selected,'request':wire,'fresh':current,
                    'admitted_wall_s':self.now(),'admitted_sim_ns':self.sim_ns()})
                reply=self.gateway.execute(canonical(wire))
        except (ProviderError,PlannerError) as invalid:
            if invalid.code=='unresolved':raise ProviderError('unresolved') from None
            error=invalid.code
        self._record('provider_receipt',{'index':turn['index'],'reply':reply,'error':error,'replayed':replayed})
        content={'reply':reply,'error_code':error,'scope':'Executive observation; independent acceptance remains separate.'}
        self.messages.append({'role':'user','content':[{'type':'tool_result','tool_use_id':selected['id'],
            'is_error':error is not None or reply['result']['status']!='succeeded','content':canonical(content)}]})
        return reply,error

    def _abort(self,code):
        self.provider_failure=code
        self.mission.fail('provider_'+code)
        self.provider.cancel()
        try:
            state=self.dialog()
            if state['turns'] and state['turns'][-1]['reply'] is None:
                turn=state['turns'][-1]
                self._record('provider_reply',{'index':turn['index'],'raw':None,'error':'unavailable',
                    'received_wall_s':self.now(),'received_sim_ns':self.sim_ns()})
            self.gateway.execute(canonical({'schema_version':1,'kind':'planner_tool_request',
                'run_id':self.inputs.run_id,'mission_id':self.mission_id,'action_id':uuid.uuid4().hex,
                'tool':'stop','observation_id':None,'arguments':{'reason':'budget_exhausted' if code=='budget_exhausted' else 'uncertain'}}))
            state=self.dialog()
            if state['turns'] and not state['turns'][-1]['complete']:
                turn=state['turns'][-1];selected=turn['reply']['parsed']['proposal']
                call=state['calls'].get(selected['id'])
                if call is not None and call['response'] is None and call['request'] is not None:
                    native=self.gateway.state()['calls'].get(call['request']['action_id'])
                    if native is not None:
                        prior=self.gateway.execute(canonical(call['request']))
                        self._record('provider_receipt',{'index':turn['index'],'reply':prior,'error':None,'replayed':False})
                    else:
                        self._record('provider_receipt',{'index':turn['index'],'reply':None,'error':'unavailable','replayed':False})
                elif call is None:
                    self._record('provider_receipt',{'index':turn['index'],'reply':None,'error':'unavailable','replayed':False})
            self._record('provider_end',{'reason':'failed','error':code})
        except (ValueError,OSError,RuntimeError,KeyboardInterrupt):
            self.failure=self.failure or 'provider_finalization_unavailable'

    def drive(self):
        try:
            while True:
                turn,selected=self._receive()
                reply,error=self._dispatch(turn,selected)
                if error is not None:continue
                if reply['result']['status']!='succeeded':raise ProviderError('unavailable')
                if selected['name'] in ('report_outcome','stop'):
                    self._record('provider_end',{'reason':'reported' if selected['name']=='report_outcome' else 'stopped','error':None})
                    return
        except ProviderError as error:self._abort(error.code)
        except (ValueError,OSError,RuntimeError,KeyboardInterrupt):self._abort('unavailable')

    def run(self):
        try:report=super().run()
        finally:self.provider.close()
        report['provider']={'mode':'synthetic-messages-replay-v1','failure':self.provider_failure,
            'history':self.journal.provider.get(self.mission_id),'recording_sha256':self.provider.source_sha256,
            'scope':'Offline synthetic responses through actual owned tools; no model quality, provider network, OS or native acceptance.'}
        return report
