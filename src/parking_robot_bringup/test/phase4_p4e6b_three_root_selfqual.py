"""Non-ROS exact-token three-root ownership selfqualification helpers.

This module intentionally uses only stdlib dummy processes.  It is not a ROS
launcher and is safe to execute as an offline ownership proof.
"""
import os
import signal
import subprocess
import time
import uuid


TOKEN_KEY = 'P4E6B_EPISODE_TOKEN'


def exact_token_match(environ_bytes, token):
    """Match exactly one NUL-delimited environment assignment, never a substring."""
    expected = (TOKEN_KEY + '=' + token).encode()
    return expected in environ_bytes.split(b'\0')


def _proc_identity(pid):
    proc = f'/proc/{pid}'
    try:
        stat = open(f'{proc}/stat').read().split()
        return {
            'pid': pid,
            'ppid': int(stat[3]),
            'pgid': os.getpgid(pid),
            'sid': os.getsid(pid),
            'cmdline': open(f'{proc}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace'),
            'starttime': stat[21],
        }
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None


def token_processes(token):
    """Inventory exact-token processes using /proc, including escaped descendants."""
    found = []
    for item in os.scandir('/proc'):
        if not item.name.isdigit():
            continue
        try:
            raw = open(f'/proc/{item.name}/environ', 'rb').read()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if exact_token_match(raw, token):
            identity = _proc_identity(int(item.name))
            if identity is not None:
                found.append(identity)
    return sorted(found, key=lambda row: row['pid'])


def _wait_absent(token, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if not token_processes(token):
            return True
        time.sleep(.03)
    return not token_processes(token)


def _term_pgid(root):
    if root.poll() is None:
        os.killpg(os.getpgid(root.pid), signal.SIGTERM)
        try:
            root.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(root.pid), signal.SIGKILL)
            root.wait(timeout=2)


def _term_exact(pid):
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return False
    return True


def _spawn(token, command='exec sleep 30'):
    env = os.environ.copy()
    env[TOKEN_KEY] = token
    return subprocess.Popen(['/bin/sh', '-c', command], env=env, start_new_session=True)


def run_three_root_selfqualification():
    """Execute the three required harmless, non-ROS ownership scenarios."""
    if TOKEN_KEY in os.environ:
        raise RuntimeError('selfqualification parent unexpectedly carries episode token')
    results = []

    # 1. Separate matrix, worker and witness roots are all inventoried.
    token = 'p4e6b-selfqual-' + uuid.uuid4().hex
    roots = [_spawn(token), _spawn(token), _spawn(token)]
    try:
        time.sleep(.08)
        inventory = token_processes(token)
        root_ids = [_proc_identity(root.pid) for root in roots]
        assert all(root_ids) and {row['pid'] for row in inventory}.issuperset({root.pid for root in roots})
        for root in roots:
            _term_pgid(root)
        assert _wait_absent(token)
        results.append({'scenario': 'NORMAL_THREE_ROOT_OWNERSHIP', 'pass': True,
                        'token': token, 'roots': root_ids, 'inventory_count': len(inventory),
                        'escaped_descendants': 0, 'token_survivor_term_count': 0,
                        'token_survivor_kill_count': 0, 'final_token_count': 0})
    finally:
        for root in roots:
            _term_pgid(root)

    # 2. Worker natural exit is independent from matrix/witness ownership.
    token = 'p4e6b-selfqual-' + uuid.uuid4().hex
    matrix, worker, witness = _spawn(token), _spawn(token, 'exit 0'), _spawn(token)
    try:
        worker.wait(timeout=2)
        time.sleep(.05)
        inventory = token_processes(token)
        pids = {row['pid'] for row in inventory}
        assert worker.pid not in pids and matrix.pid in pids and witness.pid in pids
        matrix_id, worker_id, witness_id = _proc_identity(matrix.pid), _proc_identity(worker.pid), _proc_identity(witness.pid)
        for root in (matrix, witness):
            _term_pgid(root)
        assert _wait_absent(token)
        results.append({'scenario': 'WORKER_EXITS_WITNESS_SURVIVES', 'pass': True, 'token': token,
                        'roots': [matrix_id, worker_id, witness_id], 'worker_absent': True,
                        'matrix_alive_before_cleanup': True, 'witness_alive_before_cleanup': True,
                        'next_episode_blocked_before_cleanup': True, 'final_token_count': 0,
                        'token_survivor_term_count': 0, 'token_survivor_kill_count': 0})
    finally:
        for root in (matrix, worker, witness):
            _term_pgid(root)

    # 3. A setsid descendant escapes its root PGID; exact /proc token discovery
    # must find it after root-PGID cleanup.
    token = 'p4e6b-selfqual-' + uuid.uuid4().hex
    root = _spawn(token, 'setsid /bin/sh -c "exec sleep 30" & wait')
    survivor_term_count = 0
    try:
        time.sleep(.12)
        root_id = _proc_identity(root.pid)
        assert root_id is not None
        _term_pgid(root)
        time.sleep(.08)
        survivors = token_processes(token)
        escaped = [row for row in survivors if row['pgid'] != root_id['pgid']]
        assert escaped, 'root-PGID cleanup did not leave a demonstrable escaped descendant'
        for row in escaped:
            survivor_term_count += int(_term_exact(row['pid']))
        assert _wait_absent(token)
        results.append({'scenario': 'ESCAPED_DESCENDANT_RECOVERY', 'pass': True, 'token': token,
                        'root': root_id, 'escaped_descendants': escaped,
                        'token_survivor_term_count': survivor_term_count,
                        'token_survivor_kill_count': 0, 'final_token_count': 0})
    finally:
        _term_pgid(root)
        for row in token_processes(token):
            _term_exact(row['pid'])
        _wait_absent(token)

    return {'parent_has_token': TOKEN_KEY in os.environ, 'scenarios': results,
            'root_pgid_term_count': 6,
            'token_survivor_term_count': sum(r['token_survivor_term_count'] for r in results),
            'token_survivor_kill_count': 0,
            'three_of_three_pass': len(results) == 3 and all(r['pass'] for r in results)}
