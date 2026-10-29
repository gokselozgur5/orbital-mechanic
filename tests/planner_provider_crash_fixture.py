"""Owned process cuts at durable proposal/effect/receipt boundaries."""
import json
import signal
import sys

from planner_provider_fixture import ProviderFixture


def run(mode):
    fixture=ProviderFixture();fixture.setUp();fixture.make_provider()
    append=fixture.mission_journal.append
    def checkpoint(tool_id):
        state=fixture.driver.dialog();call=state['calls'][tool_id]
        print(json.dumps({'mode':mode,'root':str(fixture.f.root),'mission_id':fixture.driver.mission_id,
            'provider_id':tool_id,'action_id':call['request']['action_id'],
            'control_revision':fixture.f.model.control_revision,
            'powered':fixture.f.model.station.machines['pump'].powered,
            'published_commands':fixture.bus.sent}),flush=True)
        signal.pause()
        raise RuntimeError('checkpoint unexpectedly resumed')
    def record(mission,kind,data):
        selected=kind=='provider_receipt' and (data.get('reply') or {}).get('result',{}).get('tool')=='isolate_power'
        if selected and mode=='after_effect':checkpoint('call_2')
        answer=append(mission,kind,data)
        if mode=='before_send' and kind=='provider_binding':checkpoint('call_0')
        if selected and mode=='after_receipt':checkpoint('call_2')
        return answer
    fixture.mission_journal.append=record
    fixture.driver.run()
    raise RuntimeError('checkpoint was not reached')


if __name__=='__main__':run(sys.argv[1])
