"""Actual no-route executor-blindness proof using the real readiness node."""
import argparse, json, os, signal, subprocess, time
from pathlib import Path
import rclpy
from parking_robot_bringup.phase4_p4e6b_health_failure_runner import HealthRuntimeRunner, PremissionCollisionReadiness


def stop_group(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGTERM)
        try: p.wait(timeout=3)
        except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL); p.wait(timeout=3)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output-dir',required=True);ap.add_argument('--domain',type=int,required=True);a=ap.parse_args()
    out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1')
    matrix=subprocess.Popen(['ros2','launch','parking_robot_bringup','phase4_p4e6b_health_matrix.launch.py','enable_health_runner:=false'],env=env,start_new_session=True,stdout=(out/'matrix.out').open('w'),stderr=(out/'matrix.err').open('w'))
    os.environ.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1');rclpy.init();node=HealthRuntimeRunner(out,'B-H01');model=PremissionCollisionReadiness();events=[]
    try:
        deadline=time.monotonic()+15; baseline=None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01); snap=node._premission_collision_snapshot(time.monotonic_ns()); result=model.observe(**snap)
            events.append({'phase':'baseline_poll','snapshot':snap,'result':result})
            if result['ready']: baseline=(snap,result);break
        if baseline is None: raise RuntimeError('baseline READY timeout')
        blind_start=time.monotonic_ns();time.sleep(.400);blind_end=time.monotonic_ns()
        # No spin occurred above: cached runner evidence is intentionally used.
        stale_snap=node._premission_collision_snapshot(time.monotonic_ns());stale=model.observe(**stale_snap)
        events.append({'phase':'stale_without_spin','snapshot':stale_snap,'result':stale})
        if stale['ready']: raise RuntimeError('FALSE_READY_WHILE_STALE')
        recovered=None;deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01);snap=node._premission_collision_snapshot(time.monotonic_ns());result=model.observe(**snap)
            events.append({'phase':'resume_poll','snapshot':snap,'result':result})
            if result['ready']: recovered=(snap,result);break
        if recovered is None: raise RuntimeError('RECOVERY_TIMEOUT')
        payload={'pass':True,'domain':a.domain,'baseline':{'snapshot':baseline[0],'result':baseline[1]},'blind_start_ns':blind_start,'blind_end_ns':blind_end,'blind_duration_ns':blind_end-blind_start,'stale':{'snapshot':stale_snap,'result':stale},'recovered':{'snapshot':recovered[0],'result':recovered[1]},'recovery_ns':time.monotonic_ns()-blind_end,'events':events,'route_mission':0,'mission_start':0,'uuid':0,'gate_arm':0,'health_injection':0}
        (out/'blindness.json').write_text(json.dumps(payload,indent=2,sort_keys=True,default=list)+'\n')
    finally:
        node.writer.finalize();node.destroy_node();rclpy.shutdown();stop_group(matrix);subprocess.run(['ros2','daemon','stop'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)
if __name__=='__main__':main()
