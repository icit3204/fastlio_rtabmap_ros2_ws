"""No-mission process-group harness for P4-E.6B startup qualification."""
import argparse, json, os, signal, subprocess, time
from pathlib import Path


def stop_group(process, grace=3.0):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try: process.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3.0)


def main():
    p=argparse.ArgumentParser(); p.add_argument("--output-dir",required=True); p.add_argument("--domain",required=True,type=int); p.add_argument("--minimum-records",type=int,default=0)
    ns=p.parse_args(); out=Path(ns.output_dir); out.mkdir(parents=True,exist_ok=True); obs=out/"observer"; env=dict(os.environ,ROS_DOMAIN_ID=str(ns.domain),ROS_LOCALHOST_ONLY="1")
    observer=subprocess.Popen(["ros2","run","parking_robot_bringup","phase4_p4e6b_collision_reason_observer","--output-dir",str(obs),"--duration-sec","20"],env=env,stdout=(out/"observer.stdout").open("w"),stderr=(out/"observer.stderr").open("w"),start_new_session=True)
    matrix=None
    try:
        deadline=time.monotonic()+5
        while not (obs/"READY.json").exists() and time.monotonic()<deadline: time.sleep(.02)
        if not (obs/"READY.json").exists(): raise RuntimeError("observer READY timeout")
        matrix=subprocess.Popen(["ros2","launch","parking_robot_bringup","phase4_p4e6b_health_matrix.launch.py","enable_health_runner:=false"],env=env,stdout=(out/"matrix.stdout").open("w"),stderr=(out/"matrix.stderr").open("w"),start_new_session=True)
        deadline=time.monotonic()+15; stable=None
        while time.monotonic()<deadline:
            if (obs/"observer.jsonl").exists():
                rows=[json.loads(line) for line in (obs/"observer.jsonl").read_text().splitlines()]
                diags=[x for x in rows if x["event"]=="collision_validity_diagnostic"]
                vals=[x for x in rows if x["event"]=="collision_validity"]
                scans=[x for x in rows if x["event"]=="synthetic_scan"]
                auth=[x for x in rows if x["event"]=="publisher_authority"]
                valid=[x for x in diags if x["reason_code"]=="VALID"]
                if valid and vals and scans and len(auth)>=4:
                    start=valid[0]["receipt_monotonic_ns"]
                    subsequent=[x for x in vals if x["receipt_monotonic_ns"]>=start]
                    enough=all(sum(x["event"]==kind for x in rows)>=ns.minimum_records for kind in ("collision_validity","collision_validity_diagnostic","synthetic_scan"))
                    if subsequent and all(x["value"] for x in subsequent) and subsequent[-1]["receipt_monotonic_ns"]-start>=1_000_000_000 and enough:
                        stable=rows; break
            time.sleep(.1)
        if stable is None: raise RuntimeError("startup did not reach 1s VALID/TRUE")
        counts={kind:sum(x["event"]==kind for x in stable) for kind in ("collision_validity","collision_validity_diagnostic","synthetic_scan","publisher_authority")}
        if any(counts[x]<ns.minimum_records for x in ("collision_validity","collision_validity_diagnostic","synthetic_scan")): raise RuntimeError("evidence count")
        authority={x["topic"]:x["publishers"] for x in stable if x["event"]=="publisher_authority"}
        if len(authority.get("/system/collision_monitor_valid",[]))!=1 or authority["/system/collision_monitor_valid"][0]["node"]!="/collision_monitor_validity_monitor": raise RuntimeError("collision authority")
        gaps=lambda kind:max([b["receipt_monotonic_ns"]-a["receipt_monotonic_ns"] for a,b in zip([x for x in stable if x["event"]==kind],[x for x in stable if x["event"]==kind][1:])] or [0])
        summary={"pass":True,"domain":ns.domain,"counts":counts,"authority":authority,"max_bool_gap_ns":gaps("collision_validity"),"max_diag_gap_ns":gaps("collision_validity_diagnostic"),"max_scan_gap_ns":gaps("synthetic_scan"),"route_mission":0,"mission_start":0,"gate_arm":0,"health_injection":0}
        (out/"summary.json").write_text(json.dumps(summary,indent=2,sort_keys=True)+"\n")
    finally:
        if matrix: stop_group(matrix)
        stop_group(observer)
        subprocess.run(["ros2","daemon","stop"],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)


if __name__=="__main__": main()
