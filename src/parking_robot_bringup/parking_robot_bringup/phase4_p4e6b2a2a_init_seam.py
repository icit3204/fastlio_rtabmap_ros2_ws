"""Qualification-only execute-argv/rclpy-init seam; creates no campaign object."""
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from .phase4_p4e6b_health_failure_runner import parse_args


def run(raw=None):
    raw=list(sys.argv[1:] if raw is None else raw)
    parsed=parse_args(raw)
    if not parsed.execute_authorized_case:
        raise RuntimeError("execute-mode seam requires explicit execute mode")
    rclpy.init(args=raw)
    node=Node("p4e6b2a2a_unremapped")
    result={"pass":True,"case_id":parsed.case_id,"execute":parsed.execute_authorized_case,
            "output_dir":parsed.output_dir,"raw":raw,"application_argv":remove_ros_args(raw),
            "effective_node_name":node.get_name(),"campaign_object_created":False,
            "route_mission":0,"mission_start":0,"gate_arm":0,"health_injection":0}
    node.destroy_node();rclpy.shutdown()
    return result


def main(args=None):
    result=run(args);out=Path(result["output_dir"]);out.mkdir(parents=True,exist_ok=True)
    (out/"execute_init_seam.json").write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
    print(json.dumps(result,sort_keys=True))
