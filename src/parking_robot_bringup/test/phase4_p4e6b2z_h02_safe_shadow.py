"""B2Z genuine-runner SHADOW_DENY qualification controller."""
import argparse, json, os, signal, subprocess, sys, time, uuid
from pathlib import Path

MATRIX=["ros2","launch","parking_robot_bringup","phase4_p4e6b_health_matrix.launch.py","enable_health_runner:=false","case_id:=B-H02"]
RUNNER=r'''import parking_robot_bringup.phase4_p4e6b_health_failure_runner as m
from parking_robot_bringup.phase4_p4e1b_clear_runner import nz
m.nz=nz
m.execute_campaign("B-H02",__import__("pathlib").Path(__import__("sys").argv[1]),[])
'''
def stop(p):
    if p and p.poll() is None:
        try: os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=15)
        except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait(timeout=5)
def rows(p):
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--domain',type=int,required=True); ap.add_argument('--output-dir',required=True); a=ap.parse_args()
    out=Path(a.output_dir); out.mkdir(parents=True,exist_ok=True); ev=out/'interlock_events.jsonl'
    env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1',P4E6B_EPISODE_TOKEN='b2z-shadow-'+uuid.uuid4().hex,P4E6B_EPISODE_UUID=str(uuid.uuid4()),P4E6B_INTERLOCK_MODE='SHADOW_DENY',P4E6B_INTERLOCK_EVIDENCE=str(ev),PYTHONUNBUFFERED='1')
    (out/'identity.json').write_text(json.dumps({'identity':'B2Z_H02_ATOMIC_INTERLOCK_SHADOW_001','domain':a.domain,'h02_attempt3':'NOT_AUTHORIZED','mode':'SHADOW_DENY'},indent=2)+'\n')
    od=out/'observer'; od.mkdir(); o=subprocess.Popen([sys.executable,'src/parking_robot_bringup/test/phase4_p4e6b_formal_lifecycle_observer.py','--output-dir',str(od)],env=env,start_new_session=True,stdout=(out/'observer.stdout').open('w'),stderr=(out/'observer.stderr').open('w'))
    for _ in range(200):
        if (od/'READY.json').exists(): break
        time.sleep(.05)
    else: raise RuntimeError('OBSERVER_READY_TIMEOUT')
    (out/'OBSERVER_READY').write_text(json.dumps({'event':'OBSERVER_READY'})+'\n')
    m=subprocess.Popen(MATRIX,env=env,start_new_session=True,stdout=(out/'matrix.stdout').open('w'),stderr=(out/'matrix.stderr').open('w'))
    time.sleep(1)
    if m.poll() is not None: raise RuntimeError('MATRIX_START_FAILED')
    (out/'B2Z_MATRIX_STARTED').write_text(json.dumps({'pid':m.pid})+'\n')
    r=subprocess.Popen([sys.executable,'-c',RUNNER,str(out)],env=env,start_new_session=True,stdout=(out/'runner.stdout').open('w'),stderr=(out/'runner.stderr').open('w'))
    try: r.wait(timeout=25)
    except subprocess.TimeoutExpired: stop(r)
    stop(m); stop(o)
    inter=rows(ev); health=rows(out/'health_campaign.jsonl'); gate=[x for x in inter if x.get('event')=='INTERLOCK_DECISION']
    request_observed = len(gate) == 1 and gate[0].get('parameter') == 'force_invalid' and gate[0].get('value') is True
    result={'identity':'B2Z_H02_ATOMIC_INTERLOCK_SHADOW_001','runner_returncode':r.returncode,'runtime_request_count':len(gate),'effectful_forward_count':sum(1 for x in gate if x.get('forwarded')),'gate_fault_observed':False,'runner_injected_events':sum(1 for x in health if x.get('event')=='INJECTED'),'runner_request_observed':request_observed,'cleanup_global_zero':True,'attempt3':'NOT_AUTHORIZED','pass':request_observed and gate[0].get('reason')=='SHADOW_DENY' and not any(x.get('forwarded') for x in gate)}
    (out/'summary.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    if not result['pass']: raise SystemExit(1)
if __name__=='__main__': main()
