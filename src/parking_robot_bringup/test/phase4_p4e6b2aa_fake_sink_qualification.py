"""Real-ROS fake-sink qualification for the B2Z LIVE_SINGLE_SHOT interlock."""
import argparse, json, os, subprocess, sys, time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter as RclParameter
from std_msgs.msg import String

from parking_robot_bringup.phase4_p4e6b_injection_interlock import InjectionInterlock


class Sink(Node):
    def __init__(self, path):
        super().__init__('b2aa_fake_downstream_sink')
        self.path = Path(path); self.count = 0
        self.sub = self.create_subscription(String, '/b2aa_fake_sink/forward', self.cb, 10)

    def cb(self, msg):
        self.count += 1
        item = {'event': 'FAKE_SINK_RECEIPT', 'monotonic_ns': time.monotonic_ns(),
                'count': self.count, 'payload': msg.data}
        with self.path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(item, sort_keys=True) + '\n'); f.flush(); os.fsync(f.fileno())


class Gateway(Node):
    def __init__(self, mode, authority, expected, evidence, sink_path):
        super().__init__('b2aa_interlock_gateway')
        self.interlock = InjectionInterlock(mode=mode, authority_token=authority,
                                            expected_token=expected, evidence_path=evidence)
        self.forward_pub = self.create_publisher(String, '/b2aa_fake_sink/forward', 10)
        self.srv = self.create_service(SetParameters, '/b2aa_interlock/set_parameters', self.cb)
        self.sink_path = Path(sink_path)

    def cb(self, request, response):
        p = request.parameters[0] if request.parameters else None
        name = p.name if p else ''
        value = bool(p.value.bool_value) if p and p.value.type == ParameterType.PARAMETER_BOOL else None
        decision = self.interlock.request(parameter=name, value=value)
        item = SetParametersResult(); item.successful = decision.accepted; item.reason = decision.reason
        response.results = [item]
        if decision.forwarded:
            msg = String(); msg.data = json.dumps({'parameter': name, 'value': value,
                                                    'monotonic_ns': time.monotonic_ns()})
            self.forward_pub.publish(msg)
        return response


def param(name, value=None, valid=True):
    p = Parameter(); p.name = name
    p.value.type = ParameterType.PARAMETER_BOOL if valid else ParameterType.PARAMETER_NOT_SET
    if valid: p.value.bool_value = value
    return p


def worker(args):
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    ev, sink_ev = out/'interlock.jsonl', out/'sink.jsonl'
    rclpy.init()
    sink = Sink(sink_ev)
    gw = Gateway(args.mode, args.authority, args.expected, ev, sink_ev)
    client = Node('b2aa_client'); c = client.create_client(SetParameters, '/b2aa_interlock/set_parameters')
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(); [ex.add_node(n) for n in (sink, gw, client)]
    import threading; t = threading.Thread(target=ex.spin, daemon=True); t.start()
    while not c.wait_for_service(timeout_sec=0.1): pass
    def call(parameters):
        req = SetParameters.Request(); req.parameters = parameters
        f = c.call_async(req)
        while not f.done(): time.sleep(.01)
        return f.result()
    results = []
    for p in args.requests.split(','):
        if p == 'true': results.append(call([param('force_invalid', True)]))
        elif p == 'false': results.append(call([param('force_invalid', False)]))
        else: results.append(call([param(p, True)]))
    time.sleep(.2)
    result = {'mode': args.mode or 'SHADOW_DENY', 'authority': args.authority or None,
              'expected': args.expected or None, 'request_count': gw.interlock.request_count,
              'forward_count': gw.interlock.forward_count, 'fake_sink_count': sink.count,
              'decisions': [r.results[0].reason for r in results]}
    (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    ex.shutdown(); [n.destroy_node() for n in (client, gw, sink)]; rclpy.shutdown(); t.join(timeout=2)
    return result


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--worker', action='store_true'); ap.add_argument('--output', required=True)
    ap.add_argument('--mode'); ap.add_argument('--authority'); ap.add_argument('--expected'); ap.add_argument('--requests', default='true')
    a=ap.parse_args()
    if a.worker: print(json.dumps(worker(a))); return
    root=Path(a.output); root.mkdir(parents=True, exist_ok=True); token='B2AA_FAKE_LIVE_AUTHORITY_'+os.urandom(8).hex()
    cases=[('missing',None,token,'true'),('wrong','WRONG_'+token,token,'true'),('authorized',token,token,'true,true'),('restart',None,None,'true')]
    results={}
    for name, auth, expected, reqs in cases:
        od=root/name; cmd=[sys.executable,__file__,'--worker','--output',str(od),'--requests',reqs]
        if name != 'restart': cmd += ['--mode','LIVE_SINGLE_SHOT','--expected',expected]
        if auth: cmd += ['--authority',auth]
        env=dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='232')
        p=subprocess.run(cmd, env=env, check=True, text=True, capture_output=True)
        results[name]=json.loads(p.stdout.strip().splitlines()[-1])
    (root/'summary.json').write_text(json.dumps(results, indent=2, sort_keys=True)+'\n')
    assert results['missing']['fake_sink_count']==0 and results['missing']['decisions']==['AUTHORITY_DENIED']
    assert results['wrong']['fake_sink_count']==0
    assert results['authorized']['forward_count']==1 and results['authorized']['fake_sink_count']==1
    assert results['authorized']['decisions']==['LIVE_SINGLE_SHOT_FORWARD','SINGLE_SHOT_ALREADY_USED']
    assert results['restart']['forward_count']==0 and results['restart']['fake_sink_count']==0

if __name__ == '__main__': main()
