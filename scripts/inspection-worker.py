#!/usr/bin/env python3
"""Private capture subprocess entry; the inspection server supplies inherited FDs."""
import argparse
import contextlib
import _thread
import os
from pathlib import Path
import stat
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.inspection_protocol import canonical, context, decode, finite
from orbital.machines import MACHINE_IDS
from orbital.ros_proximity import capture_machine_observation


def run(payload, cancelled, root=ROOT):
    if set(payload) != {'machine_id', 'timeout_s', 'context'} or payload['machine_id'] not in MACHINE_IDS:
        raise ValueError('invalid_worker_input')
    owned = context(payload['context'])
    timeout = finite(payload['timeout_s'], .1, 10.)
    # ROS and Python diagnostics belong on stderr, separate from the single reply.
    with contextlib.redirect_stdout(sys.stderr):
        result = capture_machine_observation(root, payload['machine_id'], timeout=timeout,
                                             action_context=owned, cancelled=cancelled)
    if cancelled():
        raise ValueError('inspection_cancelled')
    return {'status': 'succeeded', 'observation_id': result['observation_id'],
            'manifest_sha256': result['capture_artifact']['manifest_sha256']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lifeline-fd', required=True, type=int)
    parser.add_argument('--lock-fd', required=True, type=int)
    args = parser.parse_args(argv)
    interrupted = threading.Event()
    try:
        if not stat.S_ISFIFO(os.fstat(args.lifeline_fd).st_mode) or not stat.S_ISREG(os.fstat(args.lock_fd).st_mode):
            raise ValueError('invalid_owner_descriptors')
        for fd in (args.lifeline_fd, args.lock_fd): os.set_inheritable(fd, False)

        def watch():
            try: os.read(args.lifeline_fd, 1)
            except OSError: pass
            interrupted.set()
            _thread.interrupt_main()

        threading.Thread(target=watch, daemon=True).start()
        raw = sys.stdin.buffer.read(4097)
        if len(raw) > 4096:
            raise ValueError('worker_input_oversized')
        reply = run(decode(raw.decode('utf-8')), interrupted.is_set, root=Path.cwd())
        code = 0
    except (KeyboardInterrupt, Exception):
        reply, code = {'status': 'failed', 'error_code': 'capture_unavailable'}, 2
    print(canonical(reply), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
