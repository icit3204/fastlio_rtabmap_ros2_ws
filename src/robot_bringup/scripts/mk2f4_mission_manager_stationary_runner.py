#!/usr/bin/env python3
"""Passive recorder and typed Mission Manager client for MK2F4."""

import argparse, csv, json, math, statistics, time
from pathlib import Path

from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from parking_robot_interfaces.msg import MissionState, RouteMission
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class CommandPathReadiness:
    """Common-clock proof that MPPI -> CM is continuously ready to arm.

    The Gate's safe-input watchdog is 0.25 s.  A 0.30 s observation window
    therefore proves more than one complete watchdog interval, while the
    0.15 s maximum gap leaves 0.10 s margin to that unchanged watchdog.
    """

    def __init__(self, stability_sec=.30, max_gap_sec=.15, max_age_sec=.10):
        self.stability_ns=int(stability_sec*1e9)
        self.max_gap_ns=int(max_gap_sec*1e9)
        self.max_age_ns=int(max_age_sec*1e9)
        self.start_epoch(0)

    def start_epoch(self, now_ns):
        self.epoch_ns=int(now_ns); self.mppi=[]; self.cm=[]
        self.gate_ns=0; self.gate_inputs_healthy=False

    def observe_command(self, stream, now_ns, linear_x, angular_z):
        samples=self.mppi if stream=='mppi' else self.cm
        if samples and int(now_ns)-samples[-1][0]>self.max_gap_ns:
            samples.clear()
        samples.append((int(now_ns),float(linear_x),float(angular_z)))

    def observe_gate(self, now_ns, values):
        self.gate_ns=int(now_ns)
        self.gate_inputs_healthy=all(values.get(k)=='true' for k in (
            'localization_valid','controller_valid','collision_monitor_valid'))

    def _stream_ready(self, samples, now_ns):
        if len(samples)<3 or samples[-1][0]-samples[0][0]<self.stability_ns:
            return False
        if int(now_ns)-samples[-1][0]>self.max_age_ns:
            return False
        if max(b[0]-a[0] for a,b in zip(samples,samples[1:]))>self.max_gap_ns:
            return False
        return samples[-1][1]>.002

    def ready(self, now_ns):
        return (self._stream_ready(self.mppi,now_ns)
                and self._stream_ready(self.cm,now_ns)
                and self.gate_inputs_healthy
                and int(now_ns)-self.gate_ns<=self.max_age_ns)

    @staticmethod
    def _stats(samples):
        gaps=[(b[0]-a[0])/1e9 for a,b in zip(samples,samples[1:])]
        return {'count':len(samples),
                'duration_sec':0.0 if len(samples)<2 else (samples[-1][0]-samples[0][0])/1e9,
                'max_gap_sec':0.0 if not gaps else max(gaps),
                'first_ns':None if not samples else samples[0][0],
                'last_ns':None if not samples else samples[-1][0]}

    def evidence(self):
        return {'criterion':{'stability_sec':self.stability_ns/1e9,
                             'max_gap_sec':self.max_gap_ns/1e9,
                             'max_age_sec':self.max_age_ns/1e9},
                'mppi':self._stats(self.mppi),'cm':self._stats(self.cm),
                'gate_inputs_healthy':self.gate_inputs_healthy,
                'gate_last_ns':self.gate_ns}


class Runner(Node):
    def __init__(self):
        super().__init__('mk2f4_mission_manager_stationary_runner')
        self.events=[]; self.latest={}; self.map=None; self.states=[]
        self.readiness=CommandPathReadiness(); self.readiness_epochs=[]
        self.tf=Buffer(); self.tfl=TransformListener(self.tf, self)
        q=QoSProfile(depth=1); q.reliability=ReliabilityPolicy.RELIABLE; q.durability=DurabilityPolicy.TRANSIENT_LOCAL
        self.route=self.create_publisher(RouteMission,'/mission/route',q)
        self.start=self.create_client(Trigger,'/mission/start')
        self.cancel=self.create_client(Trigger,'/mission/cancel')
        self.pause=self.create_client(SetBool,'/mission/pause')
        self.arm=self.create_client(SetBool,'/phase5/gate_test/arm')
        self.create_subscription(MissionState,'/mission/state',self.state_cb,q)
        self.create_subscription(OccupancyGrid,'/map',self.map_cb,q)
        self.create_subscription(PathMsg,'/plan',lambda m:self.rec('path',[m.header.frame_id,len(m.poses)]),10)
        self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.command_cb('mppi',m),100)
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.command_cb('cm',m),100)
        self.create_subscription(TwistStamped,'/vehicle_cmd_safe',lambda m:self.rec('gate',[m.twist.linear.x,m.twist.angular.z]),100)
        self.create_subscription(Float32MultiArray,'/wheelchair_control_command',lambda m:self.rec('bridge',list(m.data)),100)
        self.create_subscription(Float32MultiArray,'/phase5/mk2e4/mock_can_decoded',lambda m:self.rec('backend',list(m.data)),100)
        self.create_subscription(DiagnosticStatus,'/phase5/gate_test/state',self.gate_state_cb,100)
        self.create_subscription(DiagnosticStatus,'/mission/status',self.mission_status_cb,q)
        for name in ('navigate_to_pose','compute_path_to_pose','follow_path'):
            self.create_subscription(GoalStatusArray,f'/{name}/_action/status',lambda m,n=name:self.rec('action:'+n,[int(x.status) for x in m.status_list]),20)

    def rec(self,k,v):
        e={'monotonic_ns':time.monotonic_ns(),'topic':k,'value':v}; self.events.append(e); self.latest[k]=e
    @staticmethod
    def diagnostic_values(msg): return {v.key:v.value for v in msg.values}
    def command_cb(self,k,m):
        now=time.monotonic_ns(); value=[m.linear.x,m.angular.z]
        e={'monotonic_ns':now,'topic':k,'value':value}; self.events.append(e); self.latest[k]=e
        self.readiness.observe_command(k,now,*value)
    def gate_state_cb(self,m):
        now=time.monotonic_ns(); values=self.diagnostic_values(m)
        value={'reason':m.message,'values':values}
        e={'monotonic_ns':now,'topic':'gate_state','value':value}; self.events.append(e); self.latest['gate_state']=e
        self.readiness.observe_gate(now,values)
    def mission_status_cb(self,m):
        self.rec('mission_status',{'state':m.message,'values':self.diagnostic_values(m)})
    def state_cb(self,m):
        v={'state':int(m.state),'mission_id':m.mission_id,'index':int(m.current_waypoint_index),'completed':int(m.completed_waypoint_count),'total':int(m.total_waypoint_count),'goal_uuid':m.active_goal_uuid,'reason':m.reason_code}
        self.states.append(v); self.rec('mission_state',v)
    def map_cb(self,m): self.map=m; self.rec('map',[m.header.frame_id,m.info.width,m.info.height])
    def until(self,p,t,label):
        end=time.monotonic()+t
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.02)
            if p(): return
        raise RuntimeError('timeout waiting for '+label)
    def spin(self,t):
        end=time.monotonic()+t
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
    def call(self,c,req,label):
        self.until(c.service_is_ready,20,label+' service'); self.rec('service_request:'+label,True); f=c.call_async(req); self.until(f.done,10,label+' response'); r=f.result()
        if r is None or not r.success: raise RuntimeError(label+' failed: '+('none' if r is None else r.message))
        self.rec('service:'+label,r.message); return r.message
    def gate_arm(self,on):
        label='gate_'+('arm' if on else 'disarm'); self.until(self.arm.service_is_ready,20,label+' service')
        deadline=time.monotonic()+(12 if on else 3)
        while True:
            r=SetBool.Request();r.data=on;self.rec('service_request:'+label,True);f=self.arm.call_async(r);self.until(f.done,5,label+' response');reply=f.result()
            if reply is not None and reply.success:
                self.rec('service:'+label,reply.message);return reply.message
            if not on or time.monotonic()>=deadline:
                raise RuntimeError(label+' failed: '+('none' if reply is None else reply.message))
            self.spin(.25)
    def mission_pause(self,on):
        r=SetBool.Request();r.data=on;return self.call(self.pause,r,'mission_'+('pause' if on else 'resume'))
    def trigger(self,c,label): return self.call(c,Trigger.Request(),label)
    def current_pose(self):
        self.until(lambda:self.tf.can_transform('map','base_footprint',Time(),timeout=Duration()),45,'map TF')
        return self.tf.lookup_transform('map','base_footprint',Time()).transform
    def cell(self,x,y):
        g=self.map;o=g.info.origin; a=yaw(o.orientation);dx=x-o.position.x;dy=y-o.position.y
        ix=int(math.floor((math.cos(a)*dx+math.sin(a)*dy)/g.info.resolution));iy=int(math.floor((-math.sin(a)*dx+math.cos(a)*dy)/g.info.resolution))
        return -1 if ix<0 or iy<0 or ix>=g.info.width or iy>=g.info.height else int(g.data[iy*g.info.width+ix])
    def goal(self):
        self.until(lambda:self.map is not None and self.map.header.frame_id=='map',45,'map')
        t=self.current_pose(); a=yaw(t.rotation)
        for d in (2.0,1.5,1.0):
            if all(0<=self.cell(t.translation.x+s*math.cos(a)-l*math.sin(a),t.translation.y+s*math.sin(a)+l*math.cos(a))<50 for s in [d*i/20 for i in range(21)] for l in (-.45,-.3,0,.3,.45)):
                return [t.translation.x+d*math.cos(a),t.translation.y+d*math.sin(a),a]
        raise RuntimeError('no current known-free forward mission goal')
    def publish_mission(self,mid,goal):
        m=RouteMission();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.mission_id=mid;m.route_id='mk2f4-route';m.topology_version='v1';m.node_ids=['goal-0']
        from geometry_msgs.msg import PoseStamped
        p=PoseStamped();p.header=m.header;p.pose.position.x=goal[0];p.pose.position.y=goal[1];p.pose.orientation.z=math.sin(goal[2]/2);p.pose.orientation.w=math.cos(goal[2]/2);m.poses=[p]
        # RouteMission is one-shot authority. Wait for the actual manager
        # subscriber before publishing rather than retrying/replaying routes.
        self.until(lambda:self.count_subscribers('/mission/route') == 1,10,'Mission Manager route subscriber')
        self.spin(.2)
        self.route.publish(m);self.rec('route',[mid,goal]);self.until(lambda:self.states and self.states[-1]['mission_id']==mid and self.states[-1]['state']==MissionState.RECEIVED,5,'mission received')
    def moving(self): return self.latest.get('backend',{}).get('value',[0])[0] in (4.0,4)
    def zero(self): return self.latest.get('backend',{}).get('value',[99])[0] in (3.0,3) and abs(self.latest['backend']['value'][1])<1e-6
    def begin_readiness_epoch(self,label):
        now=time.monotonic_ns();self.readiness.start_epoch(now);self.rec('readiness_epoch',label)
    def await_readiness(self,label,timeout=20):
        self.until(lambda:self.readiness.ready(time.monotonic_ns()),timeout,label+' continuous MPPI+CM readiness')
        evidence={'label':label,**self.readiness.evidence()};self.readiness_epochs.append(evidence);self.rec('readiness_proven',evidence)
    def await_armed(self):
        self.until(lambda:self.latest.get('gate_state',{}).get('value',{}).get('reason')=='ARMED_COMMAND',5,'Gate ARMED_COMMAND')
    def summary(self,start):
        ev=self.events[start:]; vals=lambda k:[e['value'] for e in ev if e['topic']==k]
        return {'samples':{k:len(vals(k)) for k in ('mppi','cm','gate','bridge','backend')},'mission_states':vals('mission_state'),'backend_gears':sorted({int(v[0]) for v in vals('backend')}),'backend_speed_range':([min(v[1] for v in vals('backend')),statistics.median(v[1] for v in vals('backend')),max(v[1] for v in vals('backend'))] if vals('backend') else []),'services':[(e['topic'],e['value']) for e in ev if e['topic'].startswith('service:')]}
    def run(self):
        self.until(lambda:all(c.service_is_ready() for c in (self.start,self.cancel,self.pause,self.arm)),90,'services')
        g=self.goal(); self.gate_arm(False)
        self.publish_mission('mk2f4-live-a',g); base=len(self.events);self.begin_readiness_epoch('initial'); self.trigger(self.start,'mission_start')
        self.until(lambda:any(s['state']==MissionState.NAVIGATING for s in self.states[-20:]),15,'mission navigating')
        self.await_readiness('initial');self.gate_arm(True);self.await_armed()
        self.until(self.moving,25,'active backend D'); active=self.summary(base)
        p0=len(self.events); self.mission_pause(True); self.until(lambda:self.states[-1]['state']==MissionState.PAUSED,10,'mission paused');self.until(self.zero,3,'pause backend zero');paused=self.summary(p0)
        self.gate_arm(False);r0=len(self.events);self.begin_readiness_epoch('resume');self.mission_pause(False);self.until(lambda:self.states[-1]['state']==MissionState.NAVIGATING,15,'mission resumed');self.await_readiness('resume');self.gate_arm(True);self.await_armed();self.until(self.moving,25,'resumed backend D');resumed=self.summary(r0)
        s0=len(self.events);safety_index=self.states[-1]['index'];self.gate_arm(False);self.until(self.zero,3,'safety backend zero');self.spin(.25);safety=self.summary(s0);safety_terminal=self.states[-1]
        if safety_terminal['completed'] or safety_terminal['index']!=safety_index or safety_terminal['state']==MissionState.SUCCEEDED:
            raise RuntimeError('Gate interruption caused false mission completion or waypoint advance')
        # Use a fresh mission for the independent user-stop case if the safety
        # observer terminated the first one, preserving truthful live policy.
        self.until(lambda:self.states[-1]['state'] not in (MissionState.CANCELLING,),8,'safety cancellation disposition')
        self.gate_arm(False)
        if self.states[-1]['state'] not in (MissionState.NAVIGATING, MissionState.TEMPORARILY_BLOCKED):
            # Gate disarm is an intentional health-termination policy, so a
            # fresh mission is genuinely required to exercise user cancel.
            self.publish_mission('mk2f4-live-cancel',g);self.begin_readiness_epoch('cancel_case');self.trigger(self.start,'mission_start_cancel_case');self.until(lambda:self.states[-1]['state']==MissionState.NAVIGATING,15,'cancel case navigating');self.await_readiness('cancel_case');self.gate_arm(True);self.await_armed();self.until(self.moving,25,'cancel case backend D')
        c0=len(self.events);self.trigger(self.cancel,'mission_cancel');self.until(lambda:self.states[-1]['state']==MissionState.CANCELLED,12,'mission cancelled');self.until(self.zero,3,'cancel backend zero');cancelled=self.summary(c0)
        return {'status':'pass','goal_pose':g,'readiness_epochs':self.readiness_epochs,'active':active,'pause':paused,'resume':resumed,'safety':safety,'safety_final_state':safety_terminal,'cancel':cancelled,'final_state':self.states[-1],'events':self.events}


def main():
    p=argparse.ArgumentParser();p.add_argument('--output-json',required=True);p.add_argument('--timeline-csv',required=True);p.add_argument('--common-clock-csv',required=True);a=p.parse_args();rclpy.init();n=Runner();result=None
    try:
        result=n.run()
    except Exception as exc:
        result={'status':'failed','error':f'{type(exc).__name__}: {exc}','readiness_epochs':n.readiness_epochs,
                'latest':n.latest,'states':n.states,'events':n.events}
        raise
    finally:
        Path(a.output_json).write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
        with Path(a.timeline_csv).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=['monotonic_ns','topic','value']);w.writeheader()
            for e in result['events']:w.writerow({**e,'value':json.dumps(e['value'],sort_keys=True)})
        with Path(a.common_clock_csv).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=['monotonic_ns','topic','value']);w.writeheader()
            for e in result['events']:
                if e['topic'] in ('mission_state','mission_status','action:navigate_to_pose','mppi','cm','gate_state','gate','bridge','backend','readiness_epoch','readiness_proven') or e['topic'].startswith('service'):
                    w.writerow({**e,'value':json.dumps(e['value'],sort_keys=True)})
        print('MK2F4_RESULT '+json.dumps({k:v for k,v in result.items() if k!='events'},sort_keys=True),flush=True)
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
