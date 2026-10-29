import copy
import json
import os
from pathlib import Path
import tempfile
import unittest

from orbital.inspection_protocol import canonical
from orbital.planner_messages import ProviderError, response, action_id, message_request, tools
from orbital.planner_recording import RecordedProvider, load
from orbital.planner_schema import TOOLS,FRESH_TOOLS
from planner_fixture import call
from planner_provider_fixture import MODEL,message,recording,step


class PlannerMessagesTests(unittest.TestCase):
    def literal(self,tool='navigate_to'):
        value=message(tool,call(tool)['arguments'])
        value['content'][0]['input']['basis_id']='c'*32 if tool in FRESH_TOOLS else None
        return value

    def test_all_tools_preserve_only_bounded_proposals_and_host_namespaced_identity(self):
        for tool in TOOLS:
            with self.subTest(tool=tool):
                value=self.literal(tool);parsed=response(canonical(value),model=MODEL)
                self.assertEqual(parsed['proposal']['name'],tool)
                self.assertEqual(parsed['proposal']['input']['arguments'],value['content'][0]['input']['arguments'])
        first=action_id('a'*32,'tool_1')
        self.assertEqual(first,action_id('a'*32,'tool_1'))
        self.assertNotEqual(first,action_id('b'*32,'tool_1'))

    def test_parallel_tools_are_refused_before_either_is_exposed(self):
        value=self.literal();value['content'].append(copy.deepcopy(value['content'][0]))
        value['content'][1]['id']='other'
        with self.assertRaisesRegex(ProviderError,'^invalid_reply$'):response(canonical(value),model=MODEL)

    def test_refusal_truncation_and_unsupported_stop_never_expose_complete_looking_tools(self):
        for reason,code in [('refusal','refused'),('max_tokens','truncated'),
                ('model_context_window_exceeded','truncated'),('pause_turn','unsupported_stop'),
                ('stop_sequence','unsupported_stop')]:
            value=self.literal();value['stop_reason']=reason
            with self.subTest(reason=reason),self.assertRaisesRegex(ProviderError,'^'+code+'$'):
                response(canonical(value),model=MODEL)

    def test_unknown_model_fields_usage_and_authority_arguments_have_fixed_errors(self):
        changes=[lambda v:v.update(model='unexpected'),lambda v:v.update(extra='canary'),
            lambda v:v['usage'].update(output_tokens=True),lambda v:v.update(content=[{'type':'server_tool_use'}]),
            lambda v:v['content'][0]['input'].update(run_id='a'*32),
            lambda v:v['content'][0]['input']['arguments'].update(path='/private/canary'),
            lambda v:v['content'][0].update(id='../file')]
        for change in changes:
            value=self.literal();change(value)
            with self.subTest(value=value),self.assertRaisesRegex(ProviderError,'^invalid_reply$'):
                response(canonical(value),model=MODEL)

    def test_duplicate_json_nonfinite_large_and_malformed_responses_are_refused(self):
        valid=canonical(self.literal())
        nonfinite=json.loads(valid)
        nonfinite['content'][0]['input']['arguments']['timeout_s']=float('nan')
        cases=[None,{},'[]','{',valid+'{}',json.dumps(nonfinite),
               valid.replace('"type":"message"','"type":"message","type":"message"'),' '*16385]
        for value in cases:
            with self.subTest(value=str(value)[:50]),self.assertRaisesRegex(ProviderError,'^invalid_reply$'):
                response(value,model=MODEL)

    def test_request_disables_parallel_calls_and_does_not_accept_tool_extensions(self):
        raw=message_request(MODEL,2048,[{'role':'user','content':[{'type':'text','text':'mission'}]}])
        value=json.loads(raw)
        self.assertEqual(value['tool_choice'],{'type':'auto','disable_parallel_tool_use':True})
        self.assertEqual(value['thinking'],{'type':'disabled'})
        self.assertEqual({v['name'] for v in value['tools']},set(TOOLS))
        self.assertNotIn('api_key',value);self.assertNotIn('endpoint',value)
        tools()[0]['input_schema']['properties']['path']={}
        self.assertNotIn('path',tools()[0]['input_schema']['properties'])

    def test_recording_loader_reads_actual_regular_file_and_refuses_links_and_fifo(self):
        value=recording([step('stop',{'reason':'operator_request'})])
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);path=root/'recording.json';path.write_text(canonical(value))
            self.assertEqual(load(path).model,MODEL)
            alias=root/'alias';alias.symlink_to(path)
            fifo=root/'fifo';os.mkfifo(fifo)
            for wrong in (alias,fifo,root):
                with self.subTest(path=wrong),self.assertRaisesRegex(ProviderError,'^invalid_recording$'):load(wrong)

    def test_recording_rejects_invalid_provenance_forward_replay_and_nonfinite_delay(self):
        for changes in ({'provenance':'claimed-real-provider'},
                        {'steps':[{'delay_s':0.,'repeat':0}]},
                        {'steps':[{**step('stop',{'reason':'uncertain'}),'delay_s':float('nan')}]},
                        {'steps':[]}):
            with self.subTest(changes=changes),self.assertRaisesRegex(ProviderError,'^invalid_recording$'):
                RecordedProvider({**recording([step('stop',{'reason':'uncertain'})]),**changes})
