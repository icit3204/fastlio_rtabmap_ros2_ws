"""Offline/demo status fixture for Operator GUI review."""

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TwistStamped
from parking_robot_interfaces.msg import MissionState, RouteMission
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class MockFixture(Node):
    def __init__(self) -> None:
        super().__init__('operator_gui_mock_fixture')
        self.declare_parameter('initial_state', 'RECEIVED')
        self.declare_parameter('gate_state', 'ARMED')
        self.declare_parameter('gate_reason', 'ARMED_COMMAND')
        self.declare_parameter('controller_valid', True)
        self.declare_parameter('perception_valid', True)
        self.declare_parameter('localization_valid', True)
        self.declare_parameter('safe_speed', 0.25)
        self.state_name = str(self.get_parameter('initial_state').value)
        self.gate_state = str(self.get_parameter('gate_state').value)
        self.gate_reason = str(self.get_parameter('gate_reason').value)
        self.q = QoSProfile(depth=1)
        self.q.reliability = ReliabilityPolicy.RELIABLE
        self.q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(MissionState, '/mission/state', self.q)
        self.route_pub = self.create_publisher(RouteMission, '/mission/route', self.q)
        self.controller_pub = self.create_publisher(Bool, '/system/controller_valid', 10)
        self.perception_pub = self.create_publisher(Bool, '/system/collision_monitor_valid', 10)
        self.localization_pub = self.create_publisher(Bool, '/system/localization_valid', 10)
        self.speed_pub = self.create_publisher(TwistStamped, '/vehicle_cmd_safe', 10)
        self.gate_pub = self.create_publisher(DiagnosticStatus, '/vehicle_cmd_safety/state', 10)
        self.create_service(Trigger, '/mission/start', self.start_cb)
        self.create_service(SetBool, '/mission/pause', self.pause_cb)
        self.create_service(Trigger, '/mission/cancel', self.cancel_cb)
        self.timer = self.create_timer(0.1, self.publish_all)

    def pose(self, x: float, y: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.w = 1.0
        return p

    def route(self) -> RouteMission:
        msg = RouteMission()
        msg.header.frame_id = 'map'
        msg.mission_id, msg.route_id, msg.topology_version = 'demo-mission', 'demo-route', 'v1'
        msg.node_ids = ['A', 'B', 'C']
        msg.edge_ids = ['A-B', 'B-C']
        msg.edge_directions = [0, 0]
        msg.poses = [self.pose(0.0, 0.0), self.pose(1.0, 0.0), self.pose(2.0, 0.0)]
        return msg

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        route = self.route()
        route.header.stamp = stamp
        state = MissionState()
        state.header.stamp, state.header.frame_id = stamp, 'map'
        state.mission_id, state.route_id = route.mission_id, route.route_id
        state.state = getattr(MissionState, self.state_name, MissionState.RECEIVED)
        state.current_waypoint_index = 1 if state.state in (MissionState.NAVIGATING, MissionState.PAUSED) else 0
        state.completed_waypoint_count = 1 if state.current_waypoint_index else 0
        state.total_waypoint_count, state.progress = 3, state.completed_waypoint_count / 3.0
        state.reason_code = self.gate_reason if state.state == MissionState.FAILED else ''
        self.route_pub.publish(route)
        self.state_pub.publish(state)
        self.controller_pub.publish(Bool(data=bool(self.get_parameter('controller_valid').value)))
        self.perception_pub.publish(Bool(data=bool(self.get_parameter('perception_valid').value)))
        self.localization_pub.publish(Bool(data=bool(self.get_parameter('localization_valid').value)))
        speed = TwistStamped(); speed.header.stamp = stamp; speed.header.frame_id = 'base_footprint'
        speed.twist.linear.x = float(self.get_parameter('safe_speed').value)
        self.speed_pub.publish(speed)
        gate = DiagnosticStatus(); gate.name = 'vehicle_cmd_safety/guarded_vehicle_cmd_gate'
        gate.message = self.gate_reason
        gate.values = [KeyValue(key='state', value=self.gate_state)]
        self.gate_pub.publish(gate)

    def start_cb(self, request, response):
        del request; self.state_name = 'NAVIGATING'; response.success = True; response.message = 'demo start accepted'; return response

    def pause_cb(self, request, response):
        self.state_name = 'PAUSED' if request.data else 'NAVIGATING'
        response.success = True; response.message = 'demo pause/resume accepted'; return response

    def cancel_cb(self, request, response):
        del request; self.state_name = 'CANCELLED'; response.success = True; response.message = 'demo cancel accepted'; return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockFixture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
