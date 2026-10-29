"""Station criteria at the claimed mission boundary, not at a later recovery."""
from orbital.inspection_artifacts import read_capture
from orbital.inspection_protocol import canonical, context
from orbital.mission_observation import MUTATIONS
from orbital.station_evaluation import healthy, effect_violations, WINDOW_NS, MAX_SAMPLE_GAP_NS
from orbital.station_truth_protocol import digest


def captures(root,observation,mission_id,registry):
    references=[]
    for (family,action_id),row in observation['results'].items():
        result=row['value'];reference=result.get('observation')
        if result['mission_id']!=mission_id or reference is None:continue
        record,image=read_capture(root,reference['observation_id'],reference['manifest_sha256'])
        bound=context(record.get('inspection_action'))
        if (any(bound[k]!=result[k] for k in ('run_id','mission_id','action_id'))
                # Maintenance invokes its own inspect request with a different
                # argument digest. Its durable result binds this exact manifest.
                or (family=='inspection' and bound['request_sha256']!=result['request_sha256'])
                or record['machine_id']!=result['machine_id']
                or record['kind']!='machine_inspection_observation'
                or bound['registry_identity']!=registry.identity
                or record['authority_identity']!=bound['authority_identity']
                or record['machine_binding']!=registry.public()['machines'][result['machine_id']]
                or record['source_sha256']!=registry.content()['source_sha256']
                or record['registry_identity']!=reference['registry_identity']
                or record['sample_stamp_ns']!=reference['sample_stamp_ns']
                or record['camera']['stamp_ns']!=reference['camera_stamp_ns']
                or not result['started_sim_stamp_ns']<=bound['admitted_sim_stamp_ns']
                    <=record['sample_stamp_ns']<=result['finished_sim_stamp_ns']):
            raise ValueError('mission_capture_identity')
        source=record['telemetry_source']
        matching=[row['value'] for row in observation['packets'].values()
                  if row['value']['stamp_ns']==record['sample_stamp_ns']
                  and row['value']['run_id']==source['run_id']]
        if len(matching)!=1 or digest(matching[0])!=source['sample_sha256']:
            raise ValueError('mission_capture_station_packet')
        packet=matching[0]
        if (any(source[k]!=packet[k] for k in ('schema_version','source','run_id','sequence','stamp_ns','time_s'))
                or source['control_revision']!=packet['control']['revision']
                or record['units']!=packet['units']
                or canonical(record['measurements'])!=canonical(
                    {'voltage_v':packet['voltage_v'],**packet['machines'][result['machine_id']]})):
            raise ValueError('mission_capture_measurements')
        references.append({'family':family,'action_id':action_id,'observation_id':reference['observation_id'],
            'manifest_sha256':reference['manifest_sha256'],'image_bytes':len(image)})
    return references


def assess(station,private_events,attempts,observation,start,terminal,mission_id):
    begin,finish=start['started_sim_ns'],terminal['quiescence']['observed_sim_ns']
    spec=station['specification'];scenario=station['scenario']
    actions={key:station['history'].actions[key] for key in scenario['actions']}
    if any(a['request']['mission_id']!=mission_id for a in actions.values()):
        raise ValueError('mission_trial_contains_other_actions')
    for attempt in attempts:
        request=attempt['command']
        if attempt['family']=='maintenance' and request['op'] in MUTATIONS:
            if (attempt['result'] is not None or (attempt['reply'] or {}).get('accepted') is True):
                if request['action_id'] not in actions or actions[request['action_id']]['request']!=request:
                    raise ValueError('mission_admitted_action_missing_from_station')
    for key,action in actions.items():
        request=action['request'];result=action['result']
        if not any(a['family']=='maintenance' and a['command']==request for a in attempts):
            raise ValueError('mission_station_action_without_publication')
        observed=observation['results'].get(('maintenance',key))
        if result is None or observed is None or observed['value']!=result:
            raise ValueError('mission_station_terminal_not_observed')
        if result['finished_sim_stamp_ns']>finish:raise ValueError('mission_station_action_after_terminal')
    window=None;last_sample=last=None;count=0;effects=[];violations=[]
    for event in private_events:
        kind,value=event['kind'],event['value']
        if kind not in ('sample','effect'):continue
        current=value['snapshot'] if kind=='sample' else value['after']
        stamp=current['sim_stamp_ns']
        if kind=='effect':
            if not begin<=stamp<=finish:raise ValueError('mission_effect_outside_window')
            if event['recorded_monotonic_s']>terminal['quiescence']['observed_wall_s']:
                raise ValueError('mission_effect_after_claim')
            effects.append(value)
            violations.extend({'action_id':value['effect']['action_id'],'reason':v} for v in effect_violations(value))
        if not begin<=stamp<=finish:continue
        if event['recorded_monotonic_s']>terminal['quiescence']['observed_wall_s']:continue
        if kind=='effect':window=None;count=0
        else:
            gap=None if last_sample is None else stamp-last_sample['sim_stamp_ns']
            if not healthy(current) or gap is None or gap>MAX_SAMPLE_GAP_NS:window=None;count=0
            if healthy(current):
                if window is None:window=stamp
                count+=1
            last_sample=current
        last=current
    if last_sample is None or last is None or finish-last_sample['sim_stamp_ns']>MAX_SAMPLE_GAP_NS:
        raise ValueError('mission_missing_station_coverage_at_terminal')
    if last['control']!=terminal['control']:raise ValueError('mission_terminal_control_differs_from_actual')
    duration=0 if window is None else last_sample['sim_stamp_ns']-window
    sustained=(last['control']==last_sample['control'] and healthy(last) and duration>=WINDOW_NS)
    mutations=[a for a in attempts if a['family']=='maintenance' and a['command']['op'] in MUTATIONS]
    unnecessary=[a for a in mutations if spec['initial_state'][a['command']['arguments']['machine_id']]['fault']=='none']
    initially_healthy=all(m['fault']=='none' for m in spec['initial_state'].values())
    repeated=len(mutations)-len({a['command']['action_id'] for a in mutations})
    claimed=terminal['outcome']
    supported=None
    if claimed in ('healthy','recovered'):
        supported=(sustained and not unnecessary and not violations and repeated==0
                   and (initially_healthy and not mutations and not effects if claimed=='healthy' else not initially_healthy))
    return {'claimed_outcome':claimed,'success_claim_supported':supported,
        'false_success_claim':supported is False,'reported_escalation':claimed=='escalated',
        'sustained_healthy_state_at_claim':sustained,'healthy_window_s_at_claim':duration/1e9,
        'healthy_window_samples_at_claim':count,'applied_interventions':len(effects),
        'mutation_publication_attempts':len(mutations),'unnecessary_mutation_attempts':len(unnecessary),
        'repeated_mutating_action_attempts':repeated,'recorded_precondition_or_effect_violations':violations,
        'observed_mission_sim_duration_s':(finish-begin)/1e9,
        'observed_mission_wall_duration_s':terminal['quiescence']['observed_wall_s']-start['started_wall_s'],
        'station_criteria_passed':bool(supported),
        'scope':'Actual station criteria and raw gateway attempts at the claimed boundary; reported escalation is not independent physical safety acceptance'}
