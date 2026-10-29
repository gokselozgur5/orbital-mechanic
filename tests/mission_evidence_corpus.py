"""One actual closed gateway run; each integrity test gets an isolated copy."""
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from orbital.inspection_protocol import canonical
from orbital.mission_evaluation import evaluate
from mission_evaluation_fixture import MissionEvaluationFixture


class MissionCorpus(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.case=MissionEvaluationFixture();cls.case.setUp()
        cls.addClassCleanup(cls.case.doCleanups)
        cls.trial=cls.case.configure()
        cls.report=cls.case.composed()
        if not cls.report['complete']:raise AssertionError(cls.report)
        cls.case.close_station()
        cls.mission=cls.report['mission_id']

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)/'evidence'
        shutil.copytree(self.case.f.root,self.root)
        self.witness=self.root/'runtime/mission-service/mission-observations'/self.mission

    def score(self,**kwargs):return evaluate(self.root,self.trial,self.mission,**kwargs)

    def events(self):return [json.loads(line) for line in (self.witness/'events.jsonl').read_text().splitlines()]

    def rewrite(self,change):
        rows=self.events();change(rows)
        previous='0'*64;publications=0
        for sequence,row in enumerate(rows,1):
            row.update(sequence=sequence,previous=previous)
            if row['kind']=='published':
                publications+=1;row['data']['publication']=publications
            if row['kind']=='end':row['data']['publications']=publications
            row['sha256']=hashlib.sha256(canonical({k:v for k,v in row.items() if k!='sha256'}).encode()).hexdigest()
            previous=row['sha256']
        raw=''.join(canonical(v)+'\n' for v in rows).encode()
        (self.witness/'events.jsonl').write_bytes(raw)
        summary=json.loads((self.witness/'summary.json').read_text())
        summary.update(records=len(rows),bytes=len(raw),chain_sha256=previous,payload_sha256=hashlib.sha256(raw).hexdigest())
        (self.witness/'summary.json').write_text(canonical(summary)+'\n')
