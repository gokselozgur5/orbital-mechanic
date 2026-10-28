import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from orbital.inspection_worker import CaptureProcess, OwnerLock

ROOT = Path(__file__).resolve().parents[1]


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.lock = OwnerLock(self.root); self.addCleanup(self.lock.close)
        self.workers = []
        self.addCleanup(self.stop_workers)

    def stop_workers(self):
        for worker in self.workers:
            if not worker.done:
                worker.stop(time.monotonic())
                self.drain(worker)

    def start(self, source):
        real = subprocess.Popen
        def launch(args, **kwargs):
            return real([sys.executable, '-c', source, *args[2:]], **kwargs)
        with patch('orbital.inspection_worker.subprocess.Popen', side_effect=launch):
            worker = CaptureProcess(self.root, self.lock.fd, {}, time.monotonic())
        self.workers.append(worker)
        return worker

    def drain(self, worker):
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            if worker.poll(time.monotonic()): return
            time.sleep(.005)
        self.fail('exact child was not reaped')

    def test_bounded_output_exit_and_json_contract(self):
        cases = [('print("x"*5000)', 'worker_output_oversized'),
                 ('import sys;sys.stderr.write("x"*9000)', 'worker_output_oversized'),
                 ('print("not JSON")', 'invalid_worker_result'),
                 ('print("{}")', 'invalid_worker_result'),
                 ('import json;print(json.dumps({"status":"failed","error_code":"capture_unavailable"}))', 'invalid_worker_result')]
        for source, expected in cases:
            worker = self.start(source); self.drain(worker)
            with self.subTest(source=source):
                self.assertEqual(worker.error, expected)
                self.assertIsNotNone(worker.process.returncode)
                self.assertLessEqual(len(worker.output), 4096)

    def test_valid_failure_and_success_references_are_distinct(self):
        worker = self.start('import sys;print(\'{"status":"failed","error_code":"capture_unavailable"}\');sys.exit(2)')
        self.drain(worker); self.assertIsNone(worker.error)
        self.assertEqual(worker.reply['status'], 'failed')
        value = {'status': 'succeeded', 'observation_id': '1'*32, 'manifest_sha256': '2'*64}
        worker = self.start('print('+repr(json.dumps(value))+')')
        self.drain(worker); self.assertEqual(worker.reply, value)

    def test_ignoring_interrupt_and_termination_requires_kill_and_reap(self):
        worker = self.start('import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);print("ready",flush=True);time.sleep(30)')
        deadline = time.monotonic()+2.
        while b'ready' not in worker.output and time.monotonic() < deadline:
            worker.poll(time.monotonic()); time.sleep(.005)
        self.assertIn(b'ready', worker.output)
        worker.stop(time.monotonic())
        self.assertFalse(worker.done)
        self.drain(worker)
        self.assertTrue(worker.term_sent and worker.kill_sent)
        self.assertEqual(worker.process.returncode, -signal.SIGKILL)
        self.assertIsNone(worker.reply)

    def test_inherited_lock_survives_parent_descriptor_close(self):
        worker = self.start('import time;print("ready",flush=True);time.sleep(30)')
        self.lock.close()
        with self.assertRaises(BlockingIOError): OwnerLock(self.root)
        worker.stop(time.monotonic()); self.drain(worker)
        replacement = OwnerLock(self.root); replacement.close()

    def test_lock_symlink_and_replacement_are_refused(self):
        file = self.root/'runtime/inspection-service/owner.lock'
        file.rename(file.with_suffix('.old'))
        file.symlink_to(file.with_suffix('.old'))
        with self.assertRaises(ValueError): self.lock.verify()
        with self.assertRaises(OSError): OwnerLock(self.root)

    def test_lifeline_eof_is_delivered_to_the_owned_process(self):
        # Isolate pipe EOF from signals and from the production capture reader.
        source = ('import os,sys,time;fd=int(sys.argv[sys.argv.index("--lifeline-fd")+1]);'
                  'os.read(fd,1);sys.exit(0)')
        worker = self.start(source)
        os.close(worker.lifeline); worker.lifeline = None
        self.drain(worker)
        self.assertEqual(worker.process.returncode, 0)
        self.assertEqual(worker.error, 'invalid_worker_result')

    def test_second_pipe_failure_closes_the_first_pipe_without_a_child(self):
        real_pipe = os.pipe
        opened = []
        def pipe():
            if opened: raise OSError('controlled descriptor exhaustion')
            opened.extend(real_pipe()); return tuple(opened)
        with patch('orbital.inspection_worker.os.pipe', side_effect=pipe), self.assertRaises(OSError):
            CaptureProcess(self.root, self.lock.fd, {}, time.monotonic())
        for fd in opened:
            with self.assertRaises(OSError): os.fstat(fd)

    def test_started_child_is_retained_when_nonblocking_setup_fails(self):
        with patch('orbital.inspection_worker.os.set_blocking', side_effect=OSError('controlled setup failure')):
            worker = self.start('import time;time.sleep(30)')
        self.assertEqual(worker.error, 'worker_setup_failed')
        self.drain(worker)
        self.assertIsNotNone(worker.process.returncode)


if __name__ == '__main__': unittest.main()
