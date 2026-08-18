"""Stdlib-only qualification worker for the witness-continuity probe."""
import argparse
import json
import os
import time
from pathlib import Path


def phase(path, name):
    with path.open('a', encoding='utf-8') as handle:
        handle.write(json.dumps({'phase': name, 'monotonic_ns': time.monotonic_ns(),
                                 'pid': os.getpid(), 'ppid': os.getppid(), 'pgid': os.getpgrp()}) + '\n')
        handle.flush(); os.fsync(handle.fileno())


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output-dir', required=True)
    parser.add_argument('--hold-sec', type=float, default=.1); args = parser.parse_args()
    output = Path(args.output_dir); output.mkdir(parents=True, exist_ok=True); journal = output/'continuity_worker_phase.jsonl'
    phase(journal, 'PROCESS_START'); phase(journal, 'CONTINUITY_WORKER_READY')
    time.sleep(args.hold_sec); phase(journal, 'PROCESS_EXIT')


if __name__ == '__main__': main()
