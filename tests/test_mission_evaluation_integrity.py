"""Semantic identity, full-window and final-read controls on actual retained data."""
from contextlib import redirect_stdout,redirect_stderr
import importlib.util
from io import StringIO
import json
from pathlib import Path
from unittest.mock import patch

from orbital import mission_evaluation, mission_outcome
from orbital.inspection_protocol import canonical
from mission_evidence_corpus import MissionCorpus


class MissionEvaluationIntegrityTests(MissionCorpus):
    def test_altered_public_packet_refuses_even_with_rehashed_witness(self):
        def change(rows):
            row=next(r for r in rows if r['kind']=='received' and r['data']['topic']=='/station/telemetry')
            value=json.loads(row['data']['text']);value['voltage_v']+=.01
            row['data']['text']=canonical(value)
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError,'packet_binding'):self.score()

    def test_unrelated_journal_prefix_refuses_even_with_valid_payload_hash(self):
        def change(rows):
            row=next(r for r in rows if r['kind']=='published')
            row['data']['journal']['sha256']='f'*64
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError,'journal_prefix'):self.score()

    def test_missing_real_publication_cannot_be_replaced_by_intent_or_result(self):
        def change(rows):
            row=next(r for r in rows if r['kind']=='published' and r['data']['family']=='inspection')
            rows.remove(row)
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError,'without_publication'):self.score()

    def test_missing_raw_terminal_owner_state_refuses_claimed_journal_copy(self):
        journal=[json.loads(line) for line in (self.root/'runtime/mission-service/events.jsonl').read_text().splitlines()]
        last=journal[-1]['data']['quiescence']['inspection']
        def change(rows):
            rows[:]=[r for r in rows if not (r['kind']=='received' and r['data']['topic']=='/inspection/state'
                                            and json.loads(r['data']['text'])==last)]
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError,'terminal_owner_observation_missing'):self.score()

    def test_navigation_generation_change_refuses_coherent_file_rehash(self):
        def change(rows):
            row=next(r for r in reversed(rows) if r['kind']=='received' and r['data']['topic']=='/navigation/state')
            value=json.loads(row['data']['text']);value['run_id']='other-run'
            row['data']['text']=canonical(value)
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError,'navigation_observation'):self.score()

    def test_actual_capture_image_corruption_refuses_evaluation(self):
        image=next((self.root/'runtime/inspection').rglob('image.ppm'))
        raw=image.read_bytes();image.write_bytes(bytes([raw[0]^1])+raw[1:])
        with self.assertRaises(ValueError):self.score()

    def test_final_witness_and_station_reads_detect_changes_during_analysis(self):
        real=mission_outcome.assess
        for relative in ('runtime/mission-service/mission-observations/'+self.mission+'/events.jsonl',
                         'runtime/station-service/station-observations/'+self.trial+'.jsonl'):
            with self.subTest(relative=relative):
                path=self.root/relative;raw=path.read_bytes()
                def changing(*args,**kwargs):
                    result=real(*args,**kwargs);path.write_bytes(raw+b'\n');return result
                try:
                    with patch.object(mission_outcome,'assess',changing):
                        with self.assertRaisesRegex(ValueError,'changed'):self.score()
                finally:path.write_bytes(raw)

    def test_changed_evaluator_source_refuses_after_valid_analysis(self):
        real=mission_evaluation.sources;calls=0
        def changing():
            nonlocal calls
            value=real();calls+=1
            if calls>1:value['orbital/mission_outcome.py']='f'*64
            return value
        with patch.object(mission_evaluation,'sources',changing):
            with self.assertRaisesRegex(ValueError,'source_changed'):self.score()

    def test_cli_reports_valid_evidence_and_bounded_errors_without_mutating_history(self):
        spec=importlib.util.spec_from_file_location('mission_evaluation_cli',
            Path(__file__).resolve().parents[1]/'scripts/evaluate-mission.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        args=['--trial-id',self.trial,'--mission-id',self.mission]
        before={str(p):p.read_bytes() for p in (self.root/'runtime').rglob('*.jsonl')}
        with patch.object(cli,'ROOT',self.root),redirect_stdout(StringIO()) as out:
            self.assertEqual(cli.main(args),0)
            self.assertIsNone(json.loads(out.getvalue())['overall_acceptance'])
        self.assertEqual(before,{str(p):p.read_bytes() for p in (self.root/'runtime').rglob('*.jsonl')})
        journal=self.root/'runtime/mission-service/events.jsonl';journal.unlink()
        with patch.object(cli,'ROOT',self.root),redirect_stderr(StringIO()) as err,redirect_stdout(StringIO()) as out:
            self.assertEqual(cli.main(args),2)
            self.assertEqual(json.loads(err.getvalue()),{'error':'mission_evaluation_unavailable'})
            self.assertEqual(out.getvalue(),'')
        self.assertFalse(journal.exists())
