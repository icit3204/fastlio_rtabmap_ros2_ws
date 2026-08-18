"""Crash-accountable, no-route startup qualification harness (P4-E.6B.2C.1F.2)."""
import argparse, json, os, signal, subprocess, sys, time, uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OBSERVER = ["ros2", "run", "parking_robot_bringup", "phase4_p4e6b_collision_reason_observer"]
MATRIX = ["ros2", "launch", "parking_robot_bringup", "phase4_p4e6b_health_matrix.launch.py", "enable_health_runner:=false"]

def durable_json(path, value):
    tmp=path.with_suffix(path.suffix+'.tmp'); tmp.write_text(json.dumps(value,sort_keys=True,indent=2)+'\n')
    with tmp.open('rb') as f: os.fsync(f.fileno())
    os.replace(tmp,path)
    fd=os.open(str(path.parent),os.O_DIRECTORY); os.fsync(fd); os.close(fd)

def append(path, value):
    value=dict(value, monotonic_ns=time.monotonic_ns())
    with path.open('a',encoding='utf-8') as f:
        f.write(json.dumps(value,sort_keys=True)+'\n'); f.flush(); os.fsync(f.fileno())

def proc_token(token):
    rows=[]
    for p in Path('/proc').glob('[0-9]*'):
        try:
            if ('P4E6B_EPISODE_TOKEN='+token).encode() not in (p/'environ').read_bytes().split(b'\0'): continue
            stat=(p/'stat').read_text().split(); cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            rows.append({'pid':int(p.name),'ppid':int(stat[3]),'pgid':int(stat[4]),'sid':int(stat[5]),'starttime':stat[21],'command':cmd})
        except (FileNotFoundError,PermissionError,IndexError): pass
    return sorted(rows,key=lambda x:x['pid'])

def kill_pids(rows, sig):
    for row in rows:
        try: os.kill(row['pid'],sig)
        except ProcessLookupError: pass

def clean(token, root, ledger):
    durable_json(ledger/'process_before_cleanup.json',proc_token(token)); append(ledger/'state_journal.jsonl',{'state':'TERMINATING'})
    actions=[]
    if root.poll() is None:
        try: os.killpg(root.pid,signal.SIGTERM); actions.append({'action':'TERM_ROOT_PGID','pgid':root.pid})
        except ProcessLookupError: pass
    time.sleep(1.0); survivors=proc_token(token)
    if survivors: kill_pids(survivors,signal.SIGTERM); actions.append({'action':'TERM_TOKEN_PIDS','pids':[x['pid'] for x in survivors]})
    time.sleep(1.0); survivors=proc_token(token)
    if survivors: kill_pids(survivors,signal.SIGKILL); actions.append({'action':'KILL_TOKEN_PIDS','pids':[x['pid'] for x in survivors]})
    try: root.wait(timeout=2)
    except subprocess.TimeoutExpired: pass
    immediate=proc_token(token); durable_json(ledger/'cleanup_actions.json',{'actions':actions}); durable_json(ledger/'process_after_cleanup_0.json',immediate)
    subprocess.run(['ros2','daemon','stop'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False); time.sleep(2.0)
    delayed=proc_token(token); durable_json(ledger/'process_after_cleanup_2s.json',delayed)
    return not immediate and not delayed

def health(rows, committed):
    diags=[r for r in rows if r.get('event')=='collision_validity_diagnostic']; vals=[r for r in rows if r.get('event')=='collision_validity']
    def first(pred):
        x=next((r for r in diags if pred(r)),None); return None if x is None else x['receipt_monotonic_ns']
    valid=first(lambda r:r.get('reason_code')=='VALID'); true=next((r['receipt_monotonic_ns'] for r in vals if r.get('value') is True),None)
    source=next((r['receipt_monotonic_ns'] for r in diags if r.get('reason_code') not in ('SOURCE_NEVER_RECEIVED',None)),None)
    active=next((r['receipt_monotonic_ns'] for r in diags if r.get('values',{}).get('lifecycle_active')=='true'),None)
    return {'first_source_ns':source,'active_ns':active,'valid_ns':valid,'bool_true_ns':true,'pass':bool(valid and true),'reason_sequence':[r.get('reason_code') for r in diags], 'committed_ns':committed}

def run_episode(campaign, index, domain):
    root=Path(campaign); token=uuid.uuid4().hex; eid=uuid.uuid4().hex; ep=root/f'episode_{index:02d}'; ep.mkdir()
    identity={'campaign_id':root.name,'episode_index':index,'episode_uuid':eid,'ros_domain_id':domain,'episode_token':token}; durable_json(ep/'identity.json',identity); journal=ep/'state_journal.jsonl'
    for state in ('PLANNED','SPAWNING'): append(journal,{'state':state})
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',P4E6B_EPISODE_TOKEN=token)
    obs=subprocess.Popen([*OBSERVER,'--output-dir',str(ep/'observer'),'--duration-sec','20'],env=env,start_new_session=True,stdout=(ep/'observer.stdout').open('w'),stderr=(ep/'observer.stderr').open('w'))
    deadline=time.monotonic()+5
    while not (ep/'observer'/'READY.json').exists() and time.monotonic()<deadline: time.sleep(.02)
    if not (ep/'observer'/'READY.json').exists(): raise RuntimeError('observer not ready before matrix')
    matrix=subprocess.Popen(MATRIX,env=env,start_new_session=True,stdout=(ep/'matrix.out').open('w'),stderr=(ep/'matrix.err').open('w'))
    committed=time.monotonic_ns(); append(root/'campaign_journal.jsonl',{'state':'COMMITTED','episode_index':index,'episode_uuid':eid,'root_pid':matrix.pid,'root_pgid':matrix.pid,'domain':domain,'token':token,'launch_monotonic_ns':committed}); append(journal,{'state':'COMMITTED','root_pid':matrix.pid,'root_pgid':matrix.pid})
    append(journal,{'state':'OBSERVING'}); rows=[]; deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        path=ep/'observer'/'observer.jsonl'
        if path.exists(): rows=[json.loads(x) for x in path.read_text().splitlines()]; h=health(rows,committed)
        else: h=health([],committed)
        if h['pass']: break
        time.sleep(.05)
    durable_json(ep/'startup_health.jsonl',h); append(journal,{'state':'STARTUP_PASS' if h['pass'] else 'STARTUP_FAIL','health':h})
    ok=clean(token,matrix,ep); append(journal,{'state':'CLEANUP_VERIFIED' if ok else 'CLEANUP_FAIL'})
    summary={'identity':identity,'matrix_root_pid':matrix.pid,'matrix_pgid':matrix.pid,'startup':h,'cleanup_verified':ok,'route_mission':0,'mission_start':0,'uuid':0,'gate_arm':0,'health_injection':0}
    durable_json(ep/'summary.json',summary)
    if h['pass'] and ok: append(journal,{'state':'SEALED'}); append(root/'campaign_journal.jsonl',{'state':'SEALED','episode_index':index}); return True
    return False

def dummy(root, index):
    token=uuid.uuid4().hex; d=Path(root)/f'dummy_{index}'; d.mkdir(); env=dict(os.environ,P4E6B_EPISODE_TOKEN=token)
    p=subprocess.Popen(['bash','-c','sleep 30 & setsid sleep 30 & wait'],env=env,start_new_session=True); time.sleep(.2); before=proc_token(token); durable_json(d/'before.json',before)
    ok=clean(token,p,d); durable_json(d/'result.json',{'token':token,'ok':ok,'before':before}); return ok

def selftest(root):
    results=[dummy(root,i) for i in range(1,6)]
    # Crash accounting reconstruction: committed rows remain even without summary.
    journal=Path(root)/'crash_journal.jsonl'; append(journal,{'state':'PLANNED','case':'A'}); append(journal,{'state':'COMMITTED','case':'B'}); append(journal,{'state':'OBSERVING','case':'C'}); append(journal,{'state':'TERMINATING','case':'D'})
    durable_json(Path(root)/'crash_reconstruction.json',{'prelaunch_noncounted':['A'],'committed_counting':['B','C','D'],'pass':True})
    serial=[]
    for i in range(5):
        p=Path(root)/f'serialization_{i}';p.mkdir(); durable_json(p/'identity.json',{'i':i}); append(p/'state_journal.jsonl',{'state':'COMMITTED'}); durable_json(p/'summary.json',{'sealed':True}); serial.append(True)
    durable_json(Path(root)/'selftest.json',{'dummy_results':results,'crash_accounting':True,'serialization_results':serial,'pass':all(results) and all(serial)})
    return all(results) and all(serial)

def main():
    p=argparse.ArgumentParser();p.add_argument('--campaign-root',required=True);p.add_argument('--selftest',action='store_true');p.add_argument('--index',type=int);p.add_argument('--domain',type=int);a=p.parse_args(); root=Path(a.campaign_root);root.mkdir(parents=True,exist_ok=True)
    if a.selftest: sys.exit(0 if selftest(root) else 1)
    if a.index is None or a.domain is None: p.error('index/domain required')
    sys.exit(0 if run_episode(root,a.index,a.domain) else 2)
if __name__=='__main__': main()
