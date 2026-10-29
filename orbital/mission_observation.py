"""Validate raw gateway observations against independent retained station data."""
from orbital import inspection_protocol, maintenance_protocol
from orbital.inspection_client import InspectionClient
from orbital.inspection_protocol import canonical, decode
from orbital.navigation_client import NavigationClient
from orbital.navigation_command import CommandGate
from orbital.station_truth_protocol import digest
from orbital.telemetry_observation import decode as telemetry

PROTOCOLS={'inspection':inspection_protocol,'maintenance':maintenance_protocol}
MUTATIONS=frozenset(('isolate','reset','restore','cancel_isolate'))


def observe(events, station, private_samples):
    registry=station['registry'];claim=station['scenario']['claim']
    states={'navigation':[],'inspection':[],'maintenance':[]}
    packets={};results={};replies=[]
    nav=NavigationClient(lambda:0.)
    def result(family,value,record):
        value=PROTOCOLS[family].result(value)
        key=(family,value['action_id'])
        if key in results and results[key]['value']!=value:raise ValueError('mission_conflicting_result')
        results.setdefault(key,{'value':value,'record':record})
    for event in events:
        if event['kind']!='received':continue
        data=event['data'];topic,text=data['topic'],data['text']
        if topic=='/station/telemetry':
            packet=telemetry(text,registry)
            if (packet['run_id']!=claim['station_run_id'] or packet['sequence'] in packets
                    or packet['sequence'] not in private_samples
                    or digest(packet)!=private_samples[packet['sequence']]['public_sha256']):
                raise ValueError('mission_station_packet_binding')
            if packets and packet['sequence']<=next(reversed(packets)):raise ValueError('mission_station_packet_order')
            packets[packet['sequence']]={'value':packet,'record':event}
        else:
            _,family,kind=topic.split('/')
            if family=='navigation':
                nav.now=lambda at=data['received_wall_s']:at
                nav.observe(text)
                if nav.failure:raise ValueError('mission_navigation_observation')
                states[family].append({'value':nav.state.copy(),'record':event})
            elif kind=='state':
                value=PROTOCOLS[family].state(text)
                if states[family]:
                    old=states[family][-1]['value']
                    if value['run_id']!=old['run_id'] or value['sequence']<=old['sequence']:
                        raise ValueError('mission_owner_generation_or_order')
                if family=='maintenance' and (value['run_id']!=claim['run_id'] or value['station_run_id']!=claim['station_run_id']):
                    raise ValueError('mission_maintenance_generation')
                states[family].append({'value':value,'record':event})
                if value['last_result'] is not None:result(family,value['last_result'],event)
            elif kind=='result':result(family,decode(text,32768),event)
            else:replies.append({'family':family,'text':text,'record':event})
    if not packets or any(not values for values in states.values()):raise ValueError('mission_missing_owner_observations')
    owners={family:values[0]['value']['run_id'] for family,values in states.items()}
    for (family,_),row in results.items():
        if row['value']['run_id']!=owners[family]:raise ValueError('mission_result_generation')
    return {'states':states,'packets':packets,'results':results,'replies':replies,'owners':owners}


def command(family,text,observation,registry,mission_id):
    value=decode(text)
    if family=='navigation':
        gate=CommandGate(observation['owners']['navigation'],registry.public()['machines'])
        if type(value.get('sequence')) is not int:raise ValueError('mission_navigation_command')
        gate.sequence=value['sequence']-1
        return gate.decode(canonical(value))
    value=PROTOCOLS[family].command(text)
    if value['mission_id']!=mission_id or value['run_id']!=observation['owners'][family]:
        raise ValueError('mission_command_generation')
    return value


def publications(events, records, check_pin, observation, registry, mission_id):
    attempts=[];intents=[];used=set()
    for record in records:
        if record['kind'] not in ('command','cancel_intent'):continue
        value=record['data']
        if not isinstance(value,dict) or set(value)!={'family','command'} or value['family'] not in ('navigation',*PROTOCOLS):
            raise ValueError('mission_command_intent')
        parsed=command(value['family'],canonical(value['command']),observation,registry,mission_id)
        intents.append((record['sequence'],value['family'],parsed))
    for event in events:
        if event['kind']!='published':continue
        value=event['data'];check_pin(value['journal'])
        if not records[0]['sequence']<=value['journal']['sequence']<records[-1]['sequence']:
            raise ValueError('mission_publication_journal_window')
        parsed=command(value['family'],value['text'],observation,registry,mission_id)
        matching=[seq for seq,f,c in intents if seq<=value['journal']['sequence'] and f==value['family'] and c==parsed]
        if not matching:raise ValueError('mission_publication_without_prior_intent')
        used.add(max(matching))
        attempts.append({'raw':value,'command':parsed,'family':value['family'],'reply':None,
                         'result':None,'intent_sequence':max(matching)})
    for reply in observation['replies']:
        value=decode(reply['text'],32768)
        matches=[a for a in attempts if a['family']==reply['family']
                 and a['command'].get('command_id')==value.get('command_id')]
        if not matches:
            if value.get('mission_id')==mission_id:raise ValueError('mission_reply_without_publication')
            continue  # Old, separately identified startup results are not this mission.
        for attempt in matches:
            request=attempt['command'];family=attempt['family']
            primary=request['op'] if request['op'] not in ('cancel','query') else ('inspect' if family=='inspection' else 'isolate')
            client=InspectionClient(lambda _:None,lambda _:None,lambda:None,
                protocol=PROTOCOLS[family],family=family,operation=primary)
            client.pending=request
            client.observe_reply(reply['text'])
            if client.failure or client.ack is None:raise ValueError('mission_invalid_observed_reply')
            if attempt['reply'] is not None and attempt['reply']!=client.ack:raise ValueError('mission_conflicting_reply')
            attempt['reply']=client.ack
    for attempt in attempts:
        family,request=attempt['family'],attempt['command']
        if family=='navigation':continue
        row=observation['results'].get((family,request['action_id']))
        if row is not None:
            value=row['value']
            if value['mission_id']!=mission_id:raise ValueError('mission_result_identity')
            if request['op'] not in ('cancel','query') and value['request_sha256']!=PROTOCOLS[family].request_digest(request):
                raise ValueError('mission_result_request')
            attempt['result']=value
    for (family,action_id),row in observation['results'].items():
        if row['value']['mission_id']==mission_id and not any(
                a['family']==family and a['command'].get('action_id')==action_id
                and a['command']['op'] not in ('query','cancel') for a in attempts):
            raise ValueError('mission_result_without_publication')
    return attempts,len(intents)-len(used)


def exact_callback(row,value,received):
    event=row['record'];data=event['data']
    return (row['value']==value and data['received_wall_s']<=received<=event['recorded_wall_s'])


def boundaries(records,observation,station,attempts):
    start,end=records[0]['data'],records[-1]['data']
    if start['registry_identity']!=station['registry'].identity or start['station_run_id']!=station['report']['station_run_id']:
        raise ValueError('mission_start_station_binding')
    baseline=start['baseline'];row=observation['packets'].get(baseline['sequence'])
    if (row is None or row['value']!=baseline or row['record']['recorded_wall_s']>start['started_wall_s']
            or baseline['control']['revision']!=0):
        raise ValueError('mission_requires_observed_fresh_trial_baseline')
    quiet=end['quiescence']
    for family,rows in observation['states'].items():
        if not any(exact_callback(row,quiet[family],quiet['received_wall_s'][family]) for row in rows):
            raise ValueError('mission_terminal_owner_observation_missing')
        if not any(row['record']['recorded_wall_s']<=start['started_wall_s'] for row in rows):
            raise ValueError('mission_owner_start_observation_missing')
    for row in observation['results'].values():
        if row['value']['mission_id']!=records[0]['mission_id']:continue
        data=row['record']['data']
        if (data['received_wall_s']>quiet['observed_wall_s']
                or data['received_sim_ns']>quiet['observed_sim_ns']
                or row['value']['finished_sim_stamp_ns']>quiet['observed_sim_ns']):
            raise ValueError('mission_result_after_terminal')
    for attempt in attempts:
        value=attempt['raw']
        if (not start['started_wall_s']<=value['entered_wall_s']<=value['finished_wall_s']<=quiet['observed_wall_s']
                or not start['started_sim_ns']<=value['entered_sim_ns']<=value['finished_sim_ns']<=quiet['observed_sim_ns']):
            raise ValueError('mission_publication_outside_window')
    return start,end
