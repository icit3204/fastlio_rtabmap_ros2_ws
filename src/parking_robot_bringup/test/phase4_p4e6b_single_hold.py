"""One owned, no-mission VALID/TRUE timing hold; never dispatches a route."""
import argparse, json, os, signal, subprocess, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
PROBE=ROOT/'src/parking_robot_bringup/test/phase4_p4e6b_cadence_probe.py'

def descendants(pid):
    result=[]; frontier=[pid]
    while frontier:
        parent=frontier.pop()
        for path in Path('/proc').glob('[0-9]*'):
            try:
                stat=(path/'stat').read_text().split(); child=int(path.name)
                if int(stat[3])==parent: result.append(child);frontier.append(child)
            except (OSError,IndexError,ValueError): pass
    return sorted(result)

def stop(proc, ledger, label):
    ledger[label+'_descendants_before']=descendants(proc.pid)
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGTERM); ledger[label+'_term']=True
        try:proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL);ledger[label+'_kill']=True;proc.wait(timeout=3)
    ledger[label+'_descendants_after']=descendants(proc.pid)

def gaps(rows): return [b[0]-a[0] for a,b in zip(rows,rows[1:])]
def stat(values):
    values=sorted(values);n=len(values)
    return {'count':n,'median_ns':values[n//2] if n else 0,'p95_ns':values[min(n-1,int(.95*n))] if n else 0,'max_ns':max(values,default=0)}

def main():
    a=argparse.ArgumentParser();a.add_argument('--output-dir',required=True);a.add_argument('--domain',required=True);ns=a.parse_args()
    out=Path(ns.output_dir);out.mkdir(parents=True,exist_ok=True);env=dict(os.environ,ROS_DOMAIN_ID=str(ns.domain),ROS_LOCALHOST_ONLY='1'); ledger={'domain':ns.domain,'route_mission':0,'mission_start':0,'gate_arm':0,'health_injection':0}
    probe=subprocess.Popen(['/usr/bin/python3',str(PROBE),'--output',str(out/'probe.json'),'--duration','14'],env=env,start_new_session=True);ledger['probe_pid']=probe.pid;ledger['probe_pgid']=probe.pid
    time.sleep(.7); matrix=subprocess.Popen(['ros2','launch','parking_robot_bringup','phase4_p4e6b_health_matrix.launch.py','enable_health_runner:=false'],env=env,start_new_session=True,stdout=(out/'matrix.out').open('w'),stderr=(out/'matrix.err').open('w'));ledger['matrix_pid']=matrix.pid;ledger['matrix_pgid']=matrix.pid
    try:
        probe.wait(timeout=15); x=json.load(open(out/'probe.json'));ds=x['streams']['diagnostic']['rows'];bs=x['streams']['bool']['rows'];v=next((r[0] for r in ds if r[1]=='VALID'),None)
        if v is None:raise RuntimeError('no VALID')
        end=v+5_000_000_000
        if max(r[0] for r in ds)<end:raise RuntimeError('healthy hold <5s')
        hold={k:[r for r in x['streams'][k]['rows'] if v<=r[0]<=end] for k in x['streams']}
        if any(not r[1] for r in hold['bool']) or any(r[1]!='VALID' for r in hold['diagnostic']):raise RuntimeError('HEALTHY_STATE_LOSS')
        summary={'pass':True,'valid_ns':v,'hold_end_ns':end,'ledger':ledger,'healthy':{k:{'receipt':stat(gaps(r)),'header':stat([b[2]-a[2] for a,b in zip(r,r[1:]) if a[2] is not None and b[2] is not None])} for k,r in hold.items()},'startup':{k:x['streams'][k]['receipt'] for k in x['streams']}}
        (out/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
    finally:
        stop(matrix,ledger,'matrix');stop(probe,ledger,'probe');subprocess.run(['ros2','daemon','stop'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False);(out/'cleanup.json').write_text(json.dumps(ledger,indent=2,sort_keys=True)+'\n')

if __name__=='__main__':main()
