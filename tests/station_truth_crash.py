"""Owned process crash cuts in the actual model and private filesystem producer."""
import json
from pathlib import Path
import sys

from station_evaluation_fixture import StationFixture

root,cut=Path(sys.argv[1]),sys.argv[2]
fixture=StationFixture();fixture.setUp()
fixture.root=root
fixture.tmp.cleanup()  # The parent owns the retained root; do not orphan the unused setup directory.
fixture.attach()


def stopped():
    print(json.dumps({'cut':cut,'trial_id':fixture.assignment['trial_id'],
                      'control_revision':fixture.model.control_revision}),flush=True)
    sys.stdin.read(1)
    raise AssertionError('Expected the parent to kill this owned child at the cut')


fixture.sample(.05);fixture.sample()
if cut=='sample':stopped()
real=fixture.truth.effect
def effect(*args):
    if cut=='before_effect_record':stopped()
    real(*args)
    stopped()
fixture.truth.effect=effect
fixture.start(fixture.request);fixture.apply()
raise AssertionError('Crash cut was not reached')
