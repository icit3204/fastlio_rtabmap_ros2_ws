"""One owned no-route invocation of the real premission helper."""
import argparse, json, os, signal, subprocess, time
from pathlib import Path
import rclpy


def stop_group(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try: process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output-dir',required=True); p.add_argument('--domain',type=int,required=True); p.add_argument('--qos',choices=('reliable','best_effort'),required=True); a=p.parse_args()
    out=Path(a.output_dir); out.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1')
    matrix=subprocess.Popen(['ros2','launch','parking_robot_bringup','phase4_p4e6b_health_matrix.launch.py','enable_health_runner:=false'],env=env,start_new_session=True,stdout=(out/'matrix.out').open('w'),stderr=(out/'matrix.err').open('w'))
    started=time.monotonic_ns()
    try:
        # A countable helper episode begins only once the actual canonical
        # graph authority exists, not merely when the launch parent exists.
        os.environ.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1')
        rclpy.init(); graph=rclpy.create_node('p4e6b_point_currentness_graph')
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            endpoints=graph.get_publishers_info_by_topic('/system/collision_monitor_valid')
            targets=[ep for ep in graph.get_publishers_info_by_topic('/diagnostics') if ep.node_name=='collision_monitor_validity_monitor' and ep.node_namespace=='/']
            if len(endpoints)==1 and len(targets)==1: break
            rclpy.spin_once(graph,timeout_sec=.05)
        graph_ready=(len(endpoints)==1 and len(targets)==1)
        graph.destroy_node(); rclpy.shutdown()
        if not graph_ready: raise RuntimeError('pre-VALID setup graph timeout')
        seam=subprocess.run(['/usr/bin/python3',str(Path(__file__).with_name('phase4_p4e6b_premission_seam.py')),'--output-dir',str(out),'--diagnostic-qos',a.qos],env=env,stdout=(out/'seam.out').open('w'),stderr=(out/'seam.err').open('w'),timeout=15)
        row=json.load(open(out/'premission_seam.json'))['event']
        assert row['ready'] and row['bool_age_ns'] < 250_000_000 and row['diagnostic_age_ns'] < 250_000_000 and row['diagnostic_transport_age_ns'] < 250_000_000 and row['source_age_upper_bound_sec'] < .5
        (out/'episode.json').write_text(json.dumps({'pass':True,'qos':a.qos,'domain':a.domain,'started_ns':started,'ready_ns':row['monotonic_ns'],'ready_latency_ns':row['monotonic_ns']-started,'row':row,'route_mission':0,'mission_start':0,'uuid':0,'gate_arm':0,'health_injection':0},indent=2,sort_keys=True)+'\n')
    finally:
        stop_group(matrix); subprocess.run(['ros2','daemon','stop'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)

if __name__=='__main__': main()
