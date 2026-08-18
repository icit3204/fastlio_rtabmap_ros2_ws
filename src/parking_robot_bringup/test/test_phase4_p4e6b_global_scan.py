import os
import subprocess
import time

from parking_robot_bringup.phase4_p4e6b_live_evidence import global_target_scan


def test_global_scan_detects_unowned_target_process():
    proc = subprocess.Popen(["bash", "-c", "exec -a phase4_p4e6b_terminal_closure_witness sleep 30"],
                            start_new_session=True)
    try:
        rows = global_target_scan()
        assert any(row["pid"] == proc.pid for row in rows)
    finally:
        os.killpg(proc.pid, 15)
        proc.wait(timeout=3)
    time.sleep(.05)
    assert not any(row["pid"] == proc.pid for row in global_target_scan())
