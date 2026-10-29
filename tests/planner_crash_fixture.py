"""Owned child checkpoints around actual tool execution; never a native robot."""
import json
import signal
import sys

from orbital.inspection_protocol import canonical
from planner_gateway_fixture import GatewayFixture


def run(mode):
    fixture=GatewayFixture();fixture.setUp()
    def checkpoint(value):
        print(json.dumps({'mode':mode,'root':str(fixture.f.root),
            'mission_id':fixture.driver.mission_id,'action_id':value['action_id'],
            'control_revision':fixture.f.model.control_revision,
            'powered':fixture.f.model.station.machines['pump'].powered,
            'published_commands':fixture.bus.sent}),flush=True)
        signal.pause()
        raise RuntimeError('checkpoint unexpectedly resumed')
    if mode=='before_send':
        fixture.driver.start()
        value=fixture.command('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        bind=fixture.driver.gateway.bind
        def admitted(family,command):
            bind(family,command)
            checkpoint(value)
        fixture.driver.gateway.bind=admitted
    else:
        fixture.diagnose()
        value=fixture.command('isolate_power',{'machine_id':'pump','timeout_s':5.})
        append=fixture.mission_journal.append
        def completed(mission,kind,data):
            selected=kind=='tool_result' and data['result']['tool']=='isolate_power'
            if selected and mode=='after_effect':checkpoint(value)
            answer=append(mission,kind,data)
            if selected and mode=='after_result':checkpoint(value)
            return answer
        fixture.mission_journal.append=completed
    fixture.driver.gateway.execute(canonical(value))
    raise RuntimeError('checkpoint was not reached')


if __name__=='__main__':run(sys.argv[1])
