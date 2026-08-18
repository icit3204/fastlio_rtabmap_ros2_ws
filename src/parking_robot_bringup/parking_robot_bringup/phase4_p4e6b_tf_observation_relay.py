"""Qualification-only /tf relay for a Mission-Manager-remapped TF stream."""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
class TfRelay(Node):
    def __init__(self):
        super().__init__("phase4_p4e6b_tf_observation_relay"); self.declare_parameter("suppress_dynamic",False); self.suppress=False
        self.pub=self.create_publisher(TFMessage,"/phase4_qualification/p4e6b/mm_tf",100); self.create_subscription(TFMessage,"/tf",self.cb,100); self.add_on_set_parameters_callback(self.params)
    def params(self,ps):
        for p in ps:
            if p.name=="suppress_dynamic":self.suppress=bool(p.value)
        return SetParametersResult(successful=True)
    def cb(self,msg):
        if not self.suppress:self.pub.publish(msg)
def main(args=None):
    rclpy.init(args=args); n=TfRelay()
    try:rclpy.spin(n)
    finally:n.destroy_node(); rclpy.shutdown()
