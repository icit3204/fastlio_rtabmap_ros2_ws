"""Minimal passive timing probe; one JSON result is written only at shutdown."""
import argparse, json, time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

TARGET="vehicle_cmd_safety/collision_monitor_validity_monitor"


def stamp_ns(stamp): return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
def stats(values):
    values=sorted(values)
    return {"count":len(values),"max_ns":max(values,default=0),"p95_ns":values[min(len(values)-1,int(.95*len(values)))] if values else 0}


class Probe(Node):
    def __init__(self):
        super().__init__("p4e6b_cadence_probe"); self.rows={"bool":[],"diagnostic":[],"scan":[]}
        self.create_subscription(Bool,"/system/collision_monitor_valid",self.boolean,100)
        self.create_subscription(DiagnosticArray,"/diagnostics",self.diagnostic,100)
        self.create_subscription(LaserScan,"/phase4/synthetic_scan",self.scan,100)
    def boolean(self,msg): self.rows["bool"].append((time.monotonic_ns(),bool(msg.data),None))
    def diagnostic(self,msg):
        for status in msg.status:
            if status.name==TARGET:self.rows["diagnostic"].append((time.monotonic_ns(),status.message,stamp_ns(msg.header.stamp)))
    def scan(self,msg): self.rows["scan"].append((time.monotonic_ns(),len(msg.ranges),stamp_ns(msg.header.stamp)))


def main():
    p=argparse.ArgumentParser();p.add_argument("--output",required=True);p.add_argument("--duration",type=float,default=12);ns=p.parse_args()
    rclpy.init(); node=Probe(); end=time.monotonic()+ns.duration
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    result={"streams":{},"valid_true":0}
    for name,rows in node.rows.items():
        receipt=[b[0]-a[0] for a,b in zip(rows,rows[1:])]
        header=[b[2]-a[2] for a,b in zip(rows,rows[1:]) if a[2] is not None and b[2] is not None]
        result["streams"][name]={"records":len(rows),"receipt":stats(receipt),"header":stats(header),"rows":rows}
    result["valid_true"]=sum(1 for _,v,_ in node.rows["bool"] if v)
    Path(ns.output).write_text(json.dumps(result,sort_keys=True)+"\n")
    node.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
