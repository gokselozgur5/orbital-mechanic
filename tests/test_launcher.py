"""Exercise transient-unit collection races without a real systemd instance."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


class LauncherTests(unittest.TestCase):
    def exercise(self, mode):
        runtime=ROOT/'runtime';runtime.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=runtime) as directory:
            base=Path(directory);scripts=base/'scripts';scripts.mkdir();bin_dir=base/'bin';bin_dir.mkdir()
            launch=scripts/'launch-thor.sh';launch.write_bytes((ROOT/'scripts/launch-thor.sh').read_bytes())
            ctl=bin_dir/'systemctl'
            ctl.write_text('''#!/usr/bin/env bash
case "$*" in
  *--property=LoadState*)
    if [[ -f "$TEST_STATE/queried" ]]; then echo not-found; else touch "$TEST_STATE/queried"; echo loaded; fi;;
  *--property=ActiveState*) echo "$TEST_MODE";;
  *reset-failed*) exit 1;;
  *) exit 90;;
esac
''')
            run=bin_dir/'systemd-run';run.write_text('#!/usr/bin/env bash\nprintf "started" > "$TEST_STATE/started"\n')
            ctl.chmod(0o755);run.chmod(0o755)
            env=os.environ.copy();env.update(PATH=str(bin_dir)+os.pathsep+env['PATH'],TEST_STATE=str(base),TEST_MODE=mode)
            result=subprocess.run(['bash',str(launch)],env=env,capture_output=True,text=True)
            return result,(base/'started').exists()

    def test_collected_between_state_read_and_reset_does_not_block_launch(self):
        result,started=self.exercise('failed')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertTrue(started)

    def test_active_world_is_not_replaced(self):
        result,started=self.exercise('active')
        self.assertNotEqual(result.returncode,0)
        self.assertFalse(started)


if __name__=='__main__':unittest.main()
