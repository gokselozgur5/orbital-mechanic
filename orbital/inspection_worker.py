"""Exact-child ownership and bounded pipes for the read-only capture process."""
import fcntl
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys

from orbital.inspection_artifacts import DIRECTORY_FLAGS, FILE_FLAGS, directory
from orbital.inspection_protocol import canonical, worker_reply


class OwnerLock:
    """Inherited open-file-description lock outlives an unexpectedly lost parent."""
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.fds = []
        try:
            parent = os.open(self.root, DIRECTORY_FLAGS); self.fds.append(parent)
            for name in ('runtime', 'inspection-service'):
                parent = directory(parent, name, True); self.fds.append(parent)
            self.fd = os.open('owner.lock', os.O_RDWR | os.O_CREAT | FILE_FLAGS, 0o600, dir_fd=parent)
            self.fds.append(self.fd)
            info = os.fstat(self.fd)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise ValueError('invalid_owner_lock')
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.identity = (info.st_dev, info.st_ino)
        except BaseException:
            self.close()
            raise

    def verify(self):
        opened = []
        try:
            parent = os.open(self.root, DIRECTORY_FLAGS); opened.append(parent)
            for name in ('runtime', 'inspection-service'):
                parent = directory(parent, name); opened.append(parent)
            info = os.stat('owner.lock', dir_fd=parent, follow_symlinks=False)
            if (info.st_dev, info.st_ino) != self.identity or info.st_nlink != 1:
                raise ValueError('owner_lock_changed')
        finally:
            for fd in reversed(opened): os.close(fd)

    def close(self):
        # LOCK_UN would release the child's inherited lock as well.
        for fd in reversed(self.fds): os.close(fd)
        self.fds.clear()


class CaptureProcess:
    """Only the returned Popen child may be signalled; retain it until reaped."""
    def __init__(self, root, lock_fd, payload, now):
        self.process = None
        self.lifeline = None
        self.streams = {}
        self.output = bytearray()
        self.error = None
        self.reply = None
        self.done = False
        self.stop_at = None
        self.term_sent = self.kill_sent = False
        raw = (canonical(payload) + '\n').encode()
        if len(raw) > 4096:
            raise ValueError('worker_input_oversized')
        input_read = input_write = life_read = None
        try:
            input_read, input_write = os.pipe()
            life_read, self.lifeline = os.pipe()
            # The bounded request fits PIPE_BUF before the child exists.
            if os.write(input_write, raw) != len(raw):
                raise OSError('worker_input_write_failed')
            os.close(input_write); input_write = None
            self.process = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve().parents[1] / 'scripts/inspection-worker.py'),
                 '--lifeline-fd', str(life_read), '--lock-fd', str(lock_fd)],
                stdin=input_read, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                pass_fds=(life_read, lock_fd), cwd=Path(root),
                env={**os.environ, 'RCUTILS_LOGGING_USE_STDOUT': '0'}, bufsize=0)
            for name in ('stdout', 'stderr'):
                stream = getattr(self.process, name)
                self.streams[name] = [stream, 0]
                os.set_blocking(stream.fileno(), False)
        except BaseException:
            if self.process is None:
                self.close()
                raise
            # Never throw away a started child after pipe setup fails.
            for stream in (self.process.stdout, self.process.stderr):
                if stream is not None: stream.close()
            self.streams.clear()
            self.error = 'worker_setup_failed'
            self.stop(now)
        finally:
            for fd in (input_read, input_write, life_read):
                if fd is not None: os.close(fd)

    def stop(self, now):
        if self.stop_at is not None:
            return
        self.stop_at = now
        if self.lifeline is not None:
            os.close(self.lifeline); self.lifeline = None
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)

    def poll(self, now):
        if self.done:
            return True
        for name, (stream, count) in list(self.streams.items()):
            limit = 4096 if name == 'stdout' else 8192
            try:
                part = os.read(stream.fileno(), min(4096, max(1, limit + 1 - count)))
            except BlockingIOError:
                continue
            except OSError:
                self.error = self.error or 'worker_pipe_failed'
                part = b''
            if not part:
                stream.close(); del self.streams[name]
                continue
            count += len(part)
            self.streams[name][1] = count
            if count > limit:
                self.error = self.error or 'worker_output_oversized'
                stream.close(); del self.streams[name]
            elif name == 'stdout':
                self.output.extend(part)
        if self.error and self.stop_at is None:
            self.stop(now)
        if self.stop_at is not None and self.process.poll() is None:
            elapsed = now - self.stop_at
            if elapsed >= .5 and not self.kill_sent:
                self.process.kill(); self.kill_sent = True
            elif elapsed >= .25 and not self.term_sent:
                self.process.terminate(); self.term_sent = True
        if self.process.poll() is None:
            return False
        if self.streams:
            # Reap is necessary but not sufficient: consume the bounded tail/EOF.
            return False
        self.done = True
        if self.stop_at is None and not self.error:
            try:
                self.reply = worker_reply(bytes(self.output).decode('utf-8'))
                expected_exit = 0 if self.reply['status'] == 'succeeded' else 2
                if self.process.returncode != expected_exit:
                    raise ValueError('worker_exit_mismatch')
            except (ValueError, UnicodeError):
                self.error = 'invalid_worker_result'
        self.close()
        return True

    def close(self):
        if self.lifeline is not None:
            os.close(self.lifeline); self.lifeline = None
        for stream, _ in self.streams.values(): stream.close()
        self.streams.clear()
