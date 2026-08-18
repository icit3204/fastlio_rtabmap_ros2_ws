"""Qualification-only single-owner observation relays."""
import os
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool
from .phase4_p4e6b_injection_interlock import InjectionInterlock

KINDS={"odom":(Odometry,"/Odometry","/phase4_qualification/p4e6b/mm_odometry"),"safe":(Twist,"/cmd_vel_nav_safe","/phase4_qualification/p4e6b/mm_cmd_vel_nav_safe"),"adapter":(DiagnosticArray,"/wheelchair_cmd_adapter/diagnostics","/phase4_qualification/p4e6b/mm_adapter_diagnostics"),"gate_localization":(Bool,"/system/localization_valid","/phase4_qualification/p4e6b/gate_localization_valid")}
class ObservationRelay(Node):
    def __init__(self):
        super().__init__("phase4_p4e6b_observation_relay")
        self.declare_parameter("kind","odom"); self.declare_parameter("suppress",False); self.declare_parameter("force_invalid",False)
        self.kind=str(self.get_parameter("kind").value); typ,src,dst=KINDS[self.kind]
        self.suppress=bool(self.get_parameter("suppress").value); self.force_invalid=bool(self.get_parameter("force_invalid").value)
        self.interlock = InjectionInterlock(
            mode=os.environ.get("P4E6B_INTERLOCK_MODE"),
            authority_token=os.environ.get("H02_ATTEMPT3_AUTHORIZATION_TOKEN"),
            expected_token=os.environ.get("P4E6B_EXPECTED_AUTHORITY_TOKEN"),
            evidence_path=os.environ.get("P4E6B_INTERLOCK_EVIDENCE")) if self.kind == "gate_localization" else None
        self.pub=self.create_publisher(typ,dst,10); self.create_subscription(typ,src,self.cb,10); self.add_on_set_parameters_callback(self.params)
    def params(self,ps):
        for p in ps:
            if p.name=="suppress": self.suppress=bool(p.value)
            if p.name=="force_invalid" and self.kind=="gate_localization":
                decision=self.interlock.request(parameter=p.name,value=bool(p.value))
                if not decision.accepted:
                    self.get_logger().warn(f"B2Z_INTERLOCK_{decision.reason}")
                    return SetParametersResult(successful=False,reason=decision.reason)
                self.force_invalid=bool(p.value)
            elif p.name=="force_invalid": self.force_invalid=bool(p.value)
        return SetParametersResult(successful=True)
    def cb(self,msg):
        if self.suppress:return
        if self.kind=="adapter" and self.force_invalid:
            for s in msg.status:
                if s.name=="mock_wheelchair_cmd_adapter": s.level=DiagnosticStatus.WARN; s.message="P4E6B_QUALIFICATION_INVALID"
        if self.kind=="gate_localization" and self.force_invalid: msg.data=False
        self.pub.publish(msg)
def main(args=None):
    rclpy.init(args=args); n=ObservationRelay()
    try:rclpy.spin(n)
    finally:n.destroy_node(); rclpy.shutdown()
