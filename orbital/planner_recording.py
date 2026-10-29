"""Nonblocking synthetic Messages replay; explicit fixture bindings, no inference.

Only the operator's recording format interprets the two reference markers.
The Messages parser never interprets model text or arbitrary templates.
"""
import copy
import hashlib
import json
import os
import stat

from orbital.inspection_protocol import canonical, decode, finite
from orbital.planner_messages import ProviderError, provider_id, MAX_REPLY_BYTES, MAX_REQUEST_BYTES, MAX_TURNS

MAX_BYTES=1024*1024


class RecordedProvider:
    def __init__(self,value,*,source_sha256=None):
        try:
            value=decode(canonical(value),MAX_BYTES)
            if (set(value)!={'schema_version','kind','provenance','model','steps'}
                    or type(value['schema_version']) is not int or value['schema_version']!=1
                    or value['kind']!='planner_messages_recording'
                    or value['provenance']!='synthetic-test-fixture'):
                raise ValueError
            self.model=provider_id(value['model'])
            if type(value['steps']) is not list or not 1<=len(value['steps'])<=MAX_TURNS:raise ValueError
            for index,step in enumerate(value['steps']):
                finite(step['delay_s'],0.,60.)
                if set(step)=={'delay_s','repeat'}:
                    if type(step['repeat']) is not int or not 0<=step['repeat']<index:raise ValueError
                elif set(step)=={'delay_s','response','bind'}:
                    if type(step['response']) is not dict or type(step['bind']) is not bool:
                        raise ValueError
                    if len(canonical(step['response']).encode())>MAX_REPLY_BYTES:raise ValueError
                else:raise ValueError
        except (ValueError,KeyError,TypeError,OverflowError,RecursionError):
            raise ProviderError('invalid_recording') from None
        self.source_sha256=source_sha256 or hashlib.sha256(canonical(value).encode()).hexdigest()
        self.steps=copy.deepcopy(value['steps'])
        self.index=0;self.active=None;self.closed=False;self.responses=[];self.audit=[]

    def submit(self,raw,now):
        if self.closed or self.active is not None or self.index>=len(self.steps):raise ProviderError('unavailable')
        value=decode(raw,MAX_REQUEST_BYTES)
        if value['model']!=self.model:raise ProviderError('unavailable')
        step=self.steps[self.index]
        if 'repeat' in step:reply=self.responses[step['repeat']]
        else:
            item=copy.deepcopy(step['response'])
            if step['bind']:
                context=json.loads(value['messages'][-1]['content'][-1]['text'])
                if set(context)!={'kind','frame'} or context['kind']!='planning_context':
                    raise ProviderError('invalid_recording')
                basis=context['frame']
                blocks=item.get('content',[])
                for block in blocks if type(blocks) is list else []:
                    if type(block) is not dict or block.get('type')!='tool_use':continue
                    data=block.get('input',{})
                    if type(data) is not dict:continue
                    if data.get('basis_id')=='$current_basis':data['basis_id']=basis['basis_id']
                    args=data.get('arguments',{})
                    if type(args) is dict and args.get('evidence_ids')==['$known_evidence']:
                        args['evidence_ids']=copy.deepcopy(basis['executive']['evidence_ids'])
            reply=canonical(item)
        if len(reply.encode())>MAX_REPLY_BYTES:raise ProviderError('invalid_recording')
        self.responses.append(reply)
        self.audit.append({'request_sha256':hashlib.sha256(raw.encode()).hexdigest(),
                           'response_sha256':hashlib.sha256(reply.encode()).hexdigest()})
        self.active=(finite(now)+step['delay_s'],reply);self.index+=1

    def poll(self,now):
        if self.closed or self.active is None:return None
        if finite(now)<self.active[0]:return None
        _,reply=self.active;self.active=None
        return reply

    def cancel(self):self.active=None

    def close(self):self.cancel();self.closed=True


def load(path):
    """Read one bounded regular operator-selected file with stable descriptor identity."""
    fd=None
    try:
        fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC|os.O_NONBLOCK)
        before=os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or not 0<before.st_size<=MAX_BYTES:raise ValueError
        raw=os.read(fd,MAX_BYTES+1);after=os.fstat(fd)
        fields=('st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns')
        if len(raw)!=before.st_size or any(getattr(before,k)!=getattr(after,k) for k in fields):raise ValueError
        return RecordedProvider(decode(raw.decode('utf-8'),MAX_BYTES),source_sha256=hashlib.sha256(raw).hexdigest())
    except (ValueError,OSError,UnicodeError):
        raise ProviderError('invalid_recording') from None
    finally:
        if fd is not None:os.close(fd)
