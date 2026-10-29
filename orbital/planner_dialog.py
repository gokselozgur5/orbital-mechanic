"""Durable provider turn and proposal ownership in the mission journal."""
import copy

from orbital.inspection_protocol import canonical, finite, identifier
from orbital.maintenance_protocol import integer
from orbital.planner_basis import validate as frame, compatible, MAX_PLAN_AGE_S
from orbital.planner_messages import (ProviderError, action_id, bounded_integer, digest,
    proposal, provider_id, response, MAX_REPLY_BYTES, MAX_TURNS, SYSTEM, tools)
from orbital.planner_protocol import request
from orbital.planner_schema import FRESH_TOOLS

KINDS=frozenset(('provider_session','provider_turn','provider_reply','provider_binding',
                 'provider_receipt','provider_end'))
REJECTIONS={'basis_changed','basis_expired','identity_conflict','invalid_argument','wrong_run',
            'stale_observation','precondition_failed','busy','budget_exhausted','unavailable','action_conflict'}


class ProviderHistory:
    def __init__(self):self._sessions={}

    def get(self,mission_id):return copy.deepcopy(self._sessions.get(mission_id))

    def advance(self,mission_id,kind,data,context,owned):
        try:return self._advance(mission_id,kind,data,context,owned)
        except (KeyError,TypeError,AttributeError,OverflowError,RecursionError):
            raise ValueError('invalid_provider_history') from None

    def _advance(self,mission_id,kind,data,context,owned):
        if kind=='terminal':
            state=self._sessions.get(mission_id)
            if state is not None and not state['closed']:raise ValueError('provider_still_owned')
            return self
        if kind not in KINDS:return self
        tools_state=owned.get(mission_id)
        if tools_state is None:raise ValueError('provider_requires_tool_session')
        if kind=='provider_session':
            fields={'run_id','model','mission_text','max_turns','max_output_tokens','max_reported_tokens',
                    'max_rejections','recording_sha256','system_sha256','tools_sha256'}
            if mission_id in self._sessions or set(data)!=fields:raise ValueError('invalid_provider_session')
            if data['run_id']!=tools_state['run_id'] or tools_state['mode']!='recorded-tools-v1':
                raise ValueError('provider_run_mismatch')
            provider_id(data['model']);bounded_integer(data['max_turns'],1,MAX_TURNS)
            bounded_integer(data['max_output_tokens'],16,4096);bounded_integer(data['max_reported_tokens'],1,1000000)
            bounded_integer(data['max_rejections'],0,4);identifier(data['recording_sha256'],64)
            if (type(data['mission_text']) is not str or not 1<=len(data['mission_text'])<=1024
                    or not data['mission_text'].strip() or data['system_sha256']!=digest(SYSTEM)
                    or data['tools_sha256']!=digest(tools())):raise ValueError('invalid_provider_session')
            state={'config':copy.deepcopy(data),'turns':[],'calls':{},'reported_tokens':0,'rejections':0,'closed':False}
        else:
            state=self.get(mission_id)
            if state is None or state['closed']:raise ValueError('provider_session_unavailable')
            config=state['config'];turns=state['turns'];calls=state['calls']
            if kind=='provider_turn':
                if (set(data)!={'index','frame','request_sha256','started_wall_s','started_sim_ns','deadline_wall_s'}
                        or data['index']!=len(turns)+1 or type(data['index']) is not int
                        or len(turns)>=config['max_turns'] or (turns and not turns[-1]['complete'])
                        or state['reported_tokens']>=config['max_reported_tokens']
                        or state['rejections']>config['max_rejections']):raise ValueError('provider_turn_not_available')
                value=frame(data['frame'],context=context,run_id=config['run_id'],mission_id=mission_id)
                if any(v not in tools_state['observations'] for v in value['executive']['evidence_ids']):
                    raise ValueError('provider_unknown_evidence')
                identifier(data['request_sha256'],64)
                start=finite(data['started_wall_s'],context['started_wall_s'],context['deadline_wall_s'])
                finite(data['deadline_wall_s'],start,min(context['deadline_wall_s'],start+MAX_PLAN_AGE_S))
                integer(data['started_sim_ns'],context['started_sim_ns'])
                turns.append({**copy.deepcopy(data),'reply':None,'receipt':None,'complete':False})
            elif kind=='provider_end':
                if (set(data)!={'reason','error'} or data['reason'] not in ('reported','failed','stopped')
                        or (turns and not turns[-1]['complete'])
                        or any(c['response'] is None for c in calls.values())):
                    raise ValueError('provider_still_owned')
                if data['reason']=='reported':
                    if tools_state['reported'] is None or data['error'] is not None:
                        raise ValueError('provider_claim_not_recorded')
                elif data['reason']=='stopped':
                    if (data['error'] is not None or owned.stop_owner!=mission_id or not turns
                            or turns[-1]['receipt'] is None or turns[-1]['receipt']['error'] is not None
                            or turns[-1]['receipt']['reply']['result']['tool']!='stop'
                            or turns[-1]['receipt']['reply']['result']['status']!='succeeded'):
                        raise ValueError('provider_stop_not_recorded')
                elif data['error'] is None or ProviderError(data['error']).code!=data['error']:
                    raise ValueError('invalid_provider_end')
                state['closed']=True;state['end']=copy.deepcopy(data)
            else:
                if not turns or data.get('index')!=len(turns) or type(data['index']) is not int:
                    raise ValueError('provider_turn_mismatch')
                turn=turns[-1]
                if turn['complete']:raise ValueError('provider_turn_already_complete')
                if kind=='provider_reply':
                    if (set(data)!={'index','raw','error','received_wall_s','received_sim_ns'}
                            or turn['reply'] is not None):raise ValueError('invalid_provider_reply_record')
                    wall=finite(data['received_wall_s'],turn['started_wall_s'])
                    integer(data['received_sim_ns'],turn['started_sim_ns'])
                    raw=data['raw'];parsed=None
                    if raw is None:
                        if data['error'] not in ('timeout','unavailable','invalid_reply','budget_exhausted'):
                            raise ValueError('invalid_provider_reply_record')
                    else:
                        if type(raw) is not str or len(raw.encode())>MAX_REPLY_BYTES:raise ValueError('provider_reply_too_large')
                        try:
                            parsed=response(raw,model=config['model'])
                            error=('ended_without_outcome' if parsed['proposal'] is None else None)
                            if (parsed['output_tokens']>config['max_output_tokens'] or
                                    state['reported_tokens']+parsed['reported_tokens']>config['max_reported_tokens']):
                                error='budget_exhausted'
                        except ProviderError as invalid:error=invalid.code
                        if wall>=turn['deadline_wall_s']:error='timeout'
                        if error!=data['error']:raise ValueError('provider_reply_classification_mismatch')
                    if parsed is not None:state['reported_tokens']+=parsed['reported_tokens']
                    turn['reply']={**copy.deepcopy(data),'parsed':parsed}
                    if data['error'] is not None:turn['complete']=True
                elif kind=='provider_binding':
                    if (set(data)!={'index','proposal','request','fresh','admitted_wall_s','admitted_sim_ns'}
                            or turn['reply'] is None or turn['reply']['error'] is not None):
                        raise ValueError('provider_binding_without_reply')
                    selected=proposal(data['proposal'])
                    if selected!=turn['reply']['parsed']['proposal'] or selected['id'] in calls:
                        raise ValueError('provider_call_identity_reused')
                    current=(None if selected['name']=='stop' and data['fresh'] is None else
                             frame(data['fresh'],context=context,run_id=config['run_id'],mission_id=mission_id))
                    wall=finite(data['admitted_wall_s'],turn['reply']['received_wall_s'])
                    sim=integer(data['admitted_sim_ns'],turn['reply']['received_sim_ns'])
                    if selected['name']!='stop' and wall>=turn['deadline_wall_s']:
                        raise ValueError('provider_admission_deadline')
                    if selected['name'] in FRESH_TOOLS and selected['input']['basis_id']!=turn['frame']['basis_id']:
                        raise ValueError('provider_basis_mismatch')
                    if selected['name'] not in ('observe_station','stop'):
                        compatible(turn['frame'],current,started_wall=turn['started_wall_s'],now=wall,
                                   started_sim=turn['started_sim_ns'],sim=sim)
                    value=request(canonical(data['request']),run_id=config['run_id'],mission_id=mission_id)
                    if (value['action_id']!=action_id(config['run_id'],selected['id'])
                            or value['tool']!=selected['name'] or value['arguments']!=selected['input']['arguments']
                            or value['observation_id']!=(current['observation']['observation_id']
                                if selected['name'] in FRESH_TOOLS else None)):
                        raise ValueError('provider_native_binding_mismatch')
                    calls[selected['id']]={'proposal':selected,'request':value,'response':None,
                                          'binding':copy.deepcopy(data)}
                elif kind=='provider_receipt':
                    if (set(data)!={'index','reply','error','replayed'} or type(data['replayed']) is not bool
                            or turn['reply'] is None or turn['reply']['error'] is not None):
                        raise ValueError('invalid_provider_receipt')
                    selected=turn['reply']['parsed']['proposal'];known=calls.get(selected['id'])
                    if known is not None and known['proposal']!=selected:
                        if data['error']!='identity_conflict' or data['reply'] is not None or data['replayed']:
                            raise ValueError('provider_identity_conflict')
                    else:
                        native=tools_state['calls'].get(action_id(config['run_id'],selected['id']))
                        if data['error'] is None:
                            if known is None or native is None or native['response'] is None:
                                raise ValueError('provider_result_not_observed')
                            reply=data['reply']
                            if (set(reply)!={'schema_version','kind','replayed','result','observations'}
                                    or type(reply['schema_version']) is not int or reply['schema_version']!=1
                                    or reply['kind']!='planner_tool_reply' or type(reply['replayed']) is not bool
                                    or native['request']!=known['request']
                                    or {k:reply[k] for k in ('result','observations')}!=native['response']):
                                raise ValueError('provider_result_binding_mismatch')
                            stored={'reply':{**copy.deepcopy(reply),'replayed':False},'error':None}
                        else:
                            if data['error'] not in REJECTIONS or data['reply'] is not None or native is not None:
                                raise ValueError('provider_effect_may_be_unresolved')
                            stored={'reply':None,'error':data['error']}
                        prior=None if known is None else known['response']
                        if data['replayed']!=(prior is not None) or (prior is not None and prior!=stored):
                            raise ValueError('provider_receipt_replay_mismatch')
                        if known is None:
                            calls[selected['id']]={'proposal':copy.deepcopy(selected),'request':None,'binding':None,'response':stored}
                        else:known['response']=stored
                    if data['error'] is not None:state['rejections']+=1
                    turn['receipt']=copy.deepcopy(data);turn['complete']=True
        new=copy.copy(self);new._sessions={**self._sessions,mission_id:state}
        return new
