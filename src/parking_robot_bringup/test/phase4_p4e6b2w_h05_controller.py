"""B2W current-stack H05 controller; qualification-only, no production changes."""
import json, os, signal, subprocess, time, uuid
from pathlib import Path

MATRIX=["ros2","launch","parking_robot_bringup","phase4_p4e6b_health_matrix.launch.py",
        "enable_health_runner:=false","case_id:=B-H05"]
RUNNER=["ros2","run","parking_robot_bringup","phase4_p4e6b_health_failure_runner",
        "--case-id","B-H05","--execute-authorized-case"]

def token_rows(token):
    rows=[]
    for p in Path('/proc').glob('[0-9]*'):
        try:
            if ('P4E6B_EPISODE_TOKEN='+token).encode() not in (p/'environ').read_bytes().split(b'\0'): continue
            st=(p/'stat').read_text().split(); cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            rows.append({'pid':int(p.name),'ppid':int(st[3]),'pgid':int(st[4]),'state':st[2],'argv':cmd})
        except (FileNotFoundError,PermissionError,IndexError,ValueError): pass
    return rows

def stop_group(p):
    if p is None or p.poll() is not None: return
    try: os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=5)
    except (ProcessLookupError,subprocess.TimeoutExpired):
        if p.poll() is None:
            try: os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError: pass
            p.wait(timeout=5)

def rows(path):
    try: return [json.loads(x) for x in Path(path).read_text().splitlines() if x]
    except FileNotFoundError: return []

def healthy_snapshot(out):
    now=time.monotonic_ns(); gs=rows(out/'witness'/'graph_observations.jsonl'); bs=rows(out/'witness'/'bool_observations.jsonl'); ds=rows(out/'witness'/'diagnostic_observations.jsonl')
    g=next((x for x in reversed(gs) if x.get('canonical_bool_publisher_count')==1 and x.get('canonical_bool_node_identity')=='collision_monitor_validity_monitor'),None)
    b=next((x for x in reversed(bs) if x.get('value') is True and x.get('post_epoch_bool_count',0)>=2),None)
    d=next((x for x in reversed(ds) if x.get('state')=='VALID' and x.get('reason_code')=='VALID' and (x.get('healthy_stable_sec') or 0)>=1.0),None)
    if not (g and b and d): return {'healthy':False,'reason':'MISSING_VALID_PROOF','graph':g,'bool':b,'diagnostic':d}
    source=(d.get('source_age_sec') or 99)+(d.get('receipt_ros_ns',0)-d.get('header_ros_ns',0))/1e9
    current=now-d.get('receipt_monotonic_ns',0)<250_000_000
    ok=current and source<.5 and d.get('diagnostic_graph_gid') and b.get('authority_token')
    return {'healthy':bool(ok),'reason':None if ok else 'STALE_OR_UNAUTHORIZED_PROOF','graph':g,'bool':b,'diagnostic':d,'source_age_upper_bound_sec':source,'observer_now_ns':now}

def main():
    out=Path(os.environ.get('B2W_H05_OUTPUT','/home/dog/phase4_reports/p4e6b2c1f3e1b2w_work/h05_requal_001')); out.mkdir(parents=True,exist_ok=True)
    domain=int(os.environ.get('B2W_H05_DOMAIN','230')); token='b2w-h05-'+uuid.uuid4().hex; episode_uuid=str(uuid.uuid4())
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',P4E6B_EPISODE_TOKEN=token,P4E6B_EPISODE_UUID=episode_uuid,PYTHONUNBUFFERED='1')
    (out/'identity.json').write_text(json.dumps({'identity':'B2W_H05_REQUAL_001','domain':domain,'token':token,'uuid':episode_uuid,'historical_h05':'PASS','new_b2m_samples':0,'formal_samples':0,'h02_injection_count':0},indent=2)+'\n')
    witness=subprocess.Popen(["/usr/bin/python3","-c","import runpy,sys; output_dir=sys.argv[1]; script_path=sys.argv[2]; sys.argv=['witness','--output-dir',output_dir]; runpy.run_path(script_path,run_name='__main__')",str(out/'witness'),str(Path(__file__).with_name('phase4_p4e6b_passive_readiness_witness.py'))],env=env,start_new_session=True,stdout=(out/'witness.out').open('w'),stderr=(out/'witness.err').open('w'))
    deadline=time.monotonic()+10
    while time.monotonic()<deadline and not (out/'witness'/'witness_phase.jsonl').exists(): time.sleep(.05)
    matrix=None; runner=None; fault_committed=False; initial=None
    try:
        matrix=subprocess.Popen(MATRIX,env=env,start_new_session=True,stdout=(out/'matrix.out').open('w'),stderr=(out/'matrix.err').open('w'))
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            initial=healthy_snapshot(out)
            if initial['healthy']: break
            time.sleep(.05)
        (out/'initial_health_gate.json').write_text(json.dumps(initial,indent=2,sort_keys=True)+'\n')
        if not initial.get('healthy'): raise RuntimeError('INITIAL_HEALTH_GATE_FAILED')
        (out/'H05_FAULT_COMMIT.json').write_text(json.dumps({'marker':'B2W_H05_FAULT_COMMITTED','fault':'scan_silent','parameter':'mode=SILENT','monotonic_ns':time.monotonic_ns(),'injection_count':1},indent=2)+'\n'); fault_committed=True
        runner_env=dict(env,B2W_H05_OUTPUT=str(out))
        runner=subprocess.Popen([*RUNNER,'--output-dir',str(out/'runner')],env=runner_env,start_new_session=True,stdout=(out/'runner.out').open('w'),stderr=(out/'runner.err').open('w'))
        try: runner.wait(timeout=45)
        except subprocess.TimeoutExpired: stop_group(runner); raise RuntimeError('H05_RUNNER_TIMEOUT')
        if runner.returncode!=0: raise RuntimeError(f'H05_RUNNER_EXIT_{runner.returncode}')
        metrics=json.loads((out/'runner'/'terminal_metrics.json').read_text())
        if not metrics.get('pass'): raise RuntimeError('H05_TERMINAL_METRICS_FAIL')
    finally:
        stop_group(runner); stop_group(matrix); stop_group(witness)
        time.sleep(2)
        cleanup={'token_processes_after':token_rows(token),'global_zero':not token_rows(token)}
        (out/'cleanup.json').write_text(json.dumps(cleanup,indent=2)+'\n')
        result={'identity':'B2W_H05_REQUAL_001','fault_commit':fault_committed,'initial_health':initial,'runner_returncode':None if runner is None else runner.returncode,'matrix_returncode':None if matrix is None else matrix.returncode,'cleanup':cleanup,'pass':fault_committed and runner is not None and runner.returncode==0 and cleanup['global_zero']}
        (out/'summary.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    raise SystemExit(0 if result['pass'] else 1)

if __name__=='__main__': main()
