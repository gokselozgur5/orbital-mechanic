from pathlib import Path
import runpy
import unittest
from unittest.mock import PropertyMock, patch

from orbital.machines import Fault, Station

ROOT = Path(__file__).resolve().parents[1]


class PublicBaselineTests(unittest.TestCase):
    def test_decision_works_without_hidden_recovery_oracle(self):
        run = runpy.run_path(str(ROOT/'scripts/run-machine-baseline.py'))['run']
        with patch.object(Station, 'recovered', new_callable=PropertyMock,
                          side_effect=AssertionError('policy read hidden recovery oracle')):
            results = [run(fault) for fault in Fault]
        self.assertTrue(all(result['passed'] for result in results))
        self.assertEqual([result['outcome'] for result in results], ['healthy', 'repaired', 'escalated'])
        self.assertEqual([result['interventions'] for result in results], [0, 3, 0])


if __name__ == '__main__':
    unittest.main()
