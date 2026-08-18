"""Subscription-only, durable qualification witness; imports ROS only after bootstrap."""
import argparse, hashlib, json, os, sys, time, traceback
from pathlib import Path

BOOL_TOPIC='/system/collision_monitor_valid'; DIAGNOSTICS_TOPIC='/diagnostics'
EXPECTED_NODE='collision_monitor_validity_monitor'; TARGET_STATUS='vehicle_cmd_safety/collision_monitor_validity_monitor'

def canonical_endpoint_gid(endpoint_gid):
    """Return one strict lowercase hex identity for an rclpy endpoint GID."""
    if isinstance(endpoint_gid, (bytes, bytearray, memoryview)):
        values=list(bytes(endpoint_gid))
    elif isinstance(endpoint_gid, (list, tuple)):
        values=list(endpoint_gid)
    else:
        raise TypeError(f'unsupported endpoint_gid type: {type(endpoint_gid).__name__}')
    if not values:
        raise ValueError('endpoint_gid must not be empty')
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0 or value > 255 for value in values):
        raise ValueError('endpoint_gid elements must be integer bytes in range 0..255')
    return ''.join(f'{value:02x}' for value in values)

def append(path,row):
    row=dict(row, monotonic_ns=row.get('monotonic_ns',time.monotonic_ns()))
    with Path(path).open('a',encoding='utf-8') as f: f.write(json.dumps(row,sort_keys=True)+'\n'); f.flush(); os.fsync(f.fileno())

def read_jsonl(path):
    rows=[]; lines=Path(path).read_text().splitlines(); meta={'truncated_final_record':False,'corrupt_line_number':None}
    for n,line in enumerate(lines,1):
        try: rows.append(json.loads(line))
        except json.JSONDecodeError:
            if n==len(lines): meta['truncated_final_record']=True; break
            meta['corrupt_line_number']=n; raise ValueError(f'corrupt JSONL line {n}')
    return rows,meta

class Epoch:
    def __init__(self): self.token=None; self.start_ns=None; self.start_ros_ns=None; self.count=0
    def observe(self,token,now_ns,now_ros_ns):
        old=self.token
        if token==old: return 'NONE'
        self.token=token; self.count=0
        if token is None: self.start_ns=self.start_ros_ns=None; return 'LOST' if old else 'NONE'
        self.start_ns,self.start_ros_ns=now_ns,now_ros_ns
        return 'REBOUND' if old else 'ESTABLISHED'
    def bool_callback(self):
        if self.token is not None: self.count+=1

def witness_contract():
    return {'subscriptions':(BOOL_TOPIC,DIAGNOSTICS_TOPIC),'publishers':(), 'services':(), 'passive':True,
            'bool_qos':'RELIABLE/VOLATILE/KEEP_LAST1','diagnostic_qos':'BEST_EFFORT/VOLATILE/KEEP_LAST1','graph_poll_sec':.05}

def _environment():
    keys=('PATH','PYTHONPATH','AMENT_PREFIX_PATH','ROS_DISTRO','ROS_VERSION','ROS_PYTHON_VERSION')
    values={k:os.environ.get(k) for k in keys}; return values,hashlib.sha256(json.dumps(values,sort_keys=True).encode()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);a=p.parse_args(); out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True); phase=out/'witness_phase.jsonl'
    append(phase,{'phase':'PROCESS_START','pid':os.getpid(),'ppid':os.getppid(),'pgid':os.getpgrp()}); append(phase,{'phase':'ROS_IMPORT_BEGIN'})
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
        from std_msgs.msg import Bool
        from diagnostic_msgs.msg import DiagnosticArray
    except Exception as e:
        env,fp=_environment(); append(phase,{'phase':'ROS_IMPORT_EXCEPTION','exception_class':type(e).__name__,'message':str(e)})
        with (out/'witness_import_exception.json').open('w') as f: json.dump({'exception_class':type(e).__name__,'message':str(e),'traceback':traceback.format_exc(),'sys_executable':sys.executable,'sys_path':sys.path,'environment':env,'environment_sha256':fp,'pid':os.getpid(),'ppid':os.getppid(),'pgid':os.getpgrp()},f);f.flush();os.fsync(f.fileno())
        raise SystemExit(1)
    append(phase,{'phase':'ROS_IMPORT_END'}); append(phase,{'phase':'RCLPY_INIT_BEGIN'});rclpy.init();append(phase,{'phase':'RCLPY_INIT_END'});append(phase,{'phase':'NODE_CONSTRUCT_BEGIN'})
    class Witness(Node):
        def __init__(self):
            super().__init__('phase4_p4e6b_passive_readiness_witness',enable_rosout=False); self.epoch=Epoch();self.phase=phase;self.out=out;self.b=out/'bool_observations.jsonl';self.d=out/'diagnostic_observations.jsonl';self.g=out/'graph_observations.jsonl'
            bq=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE);dq=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Bool,BOOL_TOPIC,self.on_bool,bq);self.create_subscription(DiagnosticArray,DIAGNOSTICS_TOPIC,self.on_diag,dq);self.create_timer(.05,self.graph);append(self.phase,{'phase':'NODE_CONSTRUCT_END'});append(self.phase,{'phase':'SUBSCRIPTIONS_READY',**witness_contract()});append(self.phase,{'phase':'GRAPH_MONITOR_READY'});append(self.phase,{'phase':'OBSERVATION_ACTIVE'})
        def graph(self):
            now=time.monotonic_ns();ros=self.get_clock().now().nanoseconds; pubs=self.get_publishers_info_by_topic(BOOL_TOPIC);b=[x for x in pubs if x.node_name==EXPECTED_NODE and x.node_namespace=='/'];ds=[x for x in self.get_publishers_info_by_topic(DIAGNOSTICS_TOPIC) if x.node_name==EXPECTED_NODE and x.node_namespace=='/'];bg=canonical_endpoint_gid(b[0].endpoint_gid) if len(b)==1 else None;dg=canonical_endpoint_gid(ds[0].endpoint_gid) if len(ds)==1 else None; token=(EXPECTED_NODE,bg,EXPECTED_NODE,dg) if bg and dg else None;transition=self.epoch.observe(token,now,ros);append(self.g,{'observation_monotonic_ns':now,'observation_ros_ns':ros,'total_bool_publisher_count':len(pubs),'canonical_bool_publisher_count':len(b),'canonical_bool_node_identity':EXPECTED_NODE if bg else None,'canonical_bool_gid':bg,'target_diagnostic_publisher_count':len(ds),'target_diagnostic_node_identity':EXPECTED_NODE if dg else None,'target_diagnostic_gid':dg,'authority_token':token,'authority_valid':token is not None,'epoch_start_monotonic_ns':self.epoch.start_ns,'epoch_start_ros_ns':self.epoch.start_ros_ns,'transition':transition})
        def on_bool(self,m):
            now=time.monotonic_ns();ros=self.get_clock().now().nanoseconds;self.epoch.bool_callback();append(self.b,{'receipt_monotonic_ns':now,'receipt_ros_ns':ros,'value':m.data,'authority_token':self.epoch.token,'generation':None if self.epoch.token is None else [EXPECTED_NODE,self.epoch.token[1]],'epoch_start_monotonic_ns':self.epoch.start_ns,'epoch_start_ros_ns':self.epoch.start_ros_ns,'post_epoch_bool_count':self.epoch.count if self.epoch.token else 0})
        def on_diag(self,m):
            now=time.monotonic_ns();ros=self.get_clock().now().nanoseconds
            for s in m.status:
                if s.name!=TARGET_STATUS: continue
                v={x.key:x.value for x in s.values}
                def num(k):
                    try:return float(v[k])
                    except (KeyError,ValueError):return None
                append(self.d,{'receipt_monotonic_ns':now,'receipt_ros_ns':ros,'header_ros_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'name':s.name,'state':v.get('state'),'reason_code':v.get('reason_code'),'healthy_stable_sec':num('healthy_stable_sec'),'source_age_sec':num('source_age_sec'),'authority_token':self.epoch.token,'diagnostic_graph_gid':None if self.epoch.token is None else self.epoch.token[3],'epoch_start_monotonic_ns':self.epoch.start_ns,'epoch_start_ros_ns':self.epoch.start_ros_ns})
    w=Witness()
    try:rclpy.spin(w)
    except ExternalShutdownException:
        pass
    except Exception as exc:
        if 'context is not valid' not in str(exc) and 'rcl_shutdown already called' not in str(exc):
            raise
    finally:
        append(phase,{'phase':'OBSERVATION_STOP_REQUESTED'});append(phase,{'phase':'WRITER_FINALIZE_BEGIN'});append(phase,{'phase':'WRITER_FINALIZE_END'});append(phase,{'phase':'DESTROY_NODE_BEGIN'});w.destroy_node();append(phase,{'phase':'DESTROY_NODE_END'});append(phase,{'phase':'RCLPY_SHUTDOWN_BEGIN'})
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception:
            pass
        append(phase,{'phase':'RCLPY_SHUTDOWN_END'});append(phase,{'phase':'PROCESS_EXIT'})
if __name__=='__main__':main()
