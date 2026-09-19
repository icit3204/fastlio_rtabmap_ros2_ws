"""Process-lifetime singleton guard for the physical R3H composition.

The lock is intentionally owned by a child process, not only by ``ros2
launch``.  If the launch frontend is killed while its children survive, the
guard survives with them and prevents a second sensor/Nav2/CAN authority from
starting over the orphaned composition.
"""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import signal
import sys
import time


class InstanceAlreadyRunning(RuntimeError):
    """Raised when another R3H physical composition owns the lease."""


class R3hInstanceLock:
    def __init__(self, path: str | os.PathLike[str] | None = None) -> None:
        runtime_dir = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp"))
        default = runtime_dir / f"mkmini-r3h-physical-{os.getuid()}.lock"
        self.path = Path(path or os.environ.get("R3H_PHYSICAL_LOCK_PATH", default))
        self._fd: int | None = None

    def acquire(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(fd)
            raise InstanceAlreadyRunning(
                f"R3H physical composition already owns {self.path}"
            ) from exc
        os.ftruncate(fd, 0)
        os.write(fd, f"pid={os.getpid()}\n".encode())
        self._fd = fd

    def close(self) -> None:
        if self._fd is not None:
            fcntl.flock(self._fd, fcntl.LOCK_UN)
            os.close(self._fd)
            self._fd = None


def main() -> int:
    lease = R3hInstanceLock()
    try:
        lease.acquire()
    except InstanceAlreadyRunning as exc:
        print(f"R3H_SINGLETON_CONFLICT: {exc}", file=sys.stderr, flush=True)
        return 73

    stop = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(f"R3H_SINGLETON_ACQUIRED: {lease.path}", flush=True)
    try:
        while not stop:
            time.sleep(0.2)
    finally:
        lease.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
