"""Single non-production fixture for the Operator GUI and canonical RViz demo.

It intentionally publishes *observational* status data, including synthetic
``/vehicle_cmd_safe`` solely for the GUI's Safe Commanded Speed display.  It
never creates a velocity command publisher, navigation action client, lower
chassis node, SocketCAN endpoint, or physical transport.
"""

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from parking_robot_interfaces.msg import MissionState, RouteMission
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


_MISSION_CODES = {
    'IDLE': MissionState.IDLE,
    'RECEIVED': MissionState.RECEIVED,
    'NAVIGATING': MissionState.NAVIGATING,
    'PAUSED': MissionState.PAUSED,
    'CANCELLING': MissionState.CANCELLING,
    'CANCELLED': MissionState.CANCELLED,
    'SUCCEEDED': MissionState.SUCCEEDED,
    'FAILED': MissionState.FAILED,
}
_CANCELLABLE = {'RECEIVED', 'NAVIGATING', 'PAUSED'}


class IntegratedDemoFixture(Node):
    """One coherent mission, spatial scene, safety display, and services."""

    def __init__(self) -> None:
        super().__init__('operator_visualization_integrated_demo_fixture')
        self.declare_parameter('initial_state', 'RECEIVED')
        requested = str(self.get_parameter('initial_state').value).upper()
        self._state_name = requested if requested in _MISSION_CODES else 'RECEIVED'
        self._normal_gate = ('ARMED_COMMAND', 'NONE')
        self._gate_state, self._gate_reason = self._normal_gate
        if self._state_name == 'FAILED':
            self._gate_state, self._gate_reason = 'DISARMED_ZERO', 'GATE_DISARMED_DEMO'

        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', retained)
        self._plan_pub = self.create_publisher(Path, '/plan', retained)
        self._route_pub = self.create_publisher(RouteMission, '/mission/route', retained)
        self._state_pub = self.create_publisher(MissionState, '/mission/state', retained)
        self._robot_description_pub = self.create_publisher(String, '/robot_description', retained)
        self._footprint_pub = self.create_publisher(PolygonStamped, '/local_costmap/published_footprint', retained)
        self._costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', retained)
        self._trajectory_pub = self.create_publisher(MarkerArray, '/trajectories', 10)
        self._controller_pub = self.create_publisher(Bool, '/system/controller_valid', 10)
        self._perception_pub = self.create_publisher(Bool, '/system/collision_monitor_valid', 10)
        self._localization_pub = self.create_publisher(Bool, '/system/localization_valid', 10)
        # Demo-status publication only; this is not a vehicle command path.
        self._safe_speed_pub = self.create_publisher(TwistStamped, '/vehicle_cmd_safe', 10)
        self._gate_pub = self.create_publisher(DiagnosticStatus, '/vehicle_cmd_safety/state', 10)
        self._tf = TransformBroadcaster(self)

        self.create_service(Trigger, '/mission/start', self._start)
        self.create_service(SetBool, '/mission/pause', self._pause)
        self.create_service(Trigger, '/mission/cancel', self._cancel)
        self.create_timer(0.1, self._publish)

    @staticmethod
    def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    @staticmethod
    def _robot_description() -> String:
        # A minimal synthetic RobotModel visual for RViz, rooted at the same
        # demo TF base frame. It is not a production chassis description.
        return String(data=(
            '<robot name="operator_visualization_demo">'
            '<link name="base_footprint"><visual><geometry><box size="0.8 0.5 0.25"/>'
            '</geometry><material name="blue"><color rgba="0.1 0.4 1.0 1.0"/>'
            '</material></visual></link></robot>'
        ))

    def _route(self, stamp) -> RouteMission:
        route = RouteMission()
        route.header.stamp, route.header.frame_id = stamp, 'map'
        route.mission_id, route.route_id, route.topology_version = 'demo-route-01', 'demo-route-01-path', 'demo-v1'
        route.node_ids = ['A', 'B', 'C', 'D']
        route.edge_ids = ['A->B', 'B->C', 'C->D']
        route.edge_directions = [0, 0, 0]
        route.poses = [
            self._pose(0.0, 0.0), self._pose(2.0, 0.2),
            self._pose(4.0, 1.0, 0.25), self._pose(6.0, 1.6, 0.25),
        ]
        return route

    def _state(self, stamp, route: RouteMission) -> MissionState:
        msg = MissionState()
        msg.header.stamp, msg.header.frame_id = stamp, 'map'
        msg.mission_id, msg.route_id = route.mission_id, route.route_id
        msg.state = _MISSION_CODES[self._state_name]
        msg.current_waypoint_index = 2
        msg.completed_waypoint_count = 2 if self._state_name in ('NAVIGATING', 'PAUSED', 'FAILED') else 0
        msg.total_waypoint_count = len(route.poses)
        msg.progress = float(msg.completed_waypoint_count) / float(msg.total_waypoint_count)
        msg.reason_code = self._gate_reason if self._state_name == 'FAILED' else ''
        msg.detail = 'Integrated mock/demo fixture; no vehicle command authority.'
        return msg

    @staticmethod
    def _grid(stamp, topic_frame: str = 'map') -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, topic_frame
        grid.info.resolution, grid.info.width, grid.info.height = 0.1, 120, 120
        grid.info.origin.position.x, grid.info.origin.position.y = -4.0, -4.0
        grid.info.origin.orientation.w = 1.0
        data = [0] * (grid.info.width * grid.info.height)
        for row in range(50, 60):
            for col in range(74, 82):
                data[row * grid.info.width + col] = 100
        grid.data = data
        return grid

    @staticmethod
    def _footprint(stamp) -> PolygonStamped:
        footprint = PolygonStamped()
        footprint.header.stamp, footprint.header.frame_id = stamp, 'map'
        footprint.polygon.points = [
            Point32(x=1.30, y=-0.05), Point32(x=2.10, y=-0.05),
            Point32(x=2.10, y=0.45), Point32(x=1.30, y=0.45),
        ]
        return footprint

    def _plan(self, stamp) -> Path:
        path = Path()
        path.header.stamp, path.header.frame_id = stamp, 'map'
        path.poses = [self._pose(i * 0.25, 0.03 * (i * 0.25) ** 2) for i in range(25)]
        return path

    def _trajectories(self, stamp) -> MarkerArray:
        markers = MarkerArray()
        for namespace, marker_id, offset, color in (
            ('Optimal Trajectory', 0, 0.00, (0.0, 0.9, 1.0)),
            ('Candidate Trajectories', 1, 0.28, (1.0, 0.55, 0.0)),
            ('Candidate Trajectories', 2, -0.25, (1.0, 0.55, 0.0)),
        ):
            marker = Marker()
            marker.header.stamp, marker.header.frame_id = stamp, 'map'
            marker.ns, marker.id = namespace, marker_id
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.08
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 0.95
            marker.points = [
                Point(x=1.7 + 0.35 * i, y=0.12 + offset + 0.035 * i * i, z=0.05)
                for i in range(8)
            ]
            markers.markers.append(marker)
        return markers

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        route = self._route(stamp)
        state = self._state(stamp, route)
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x, transform.transform.translation.y = 1.7, 0.12
        transform.transform.rotation.w = 1.0

        self._map_pub.publish(self._grid(stamp))
        self._costmap_pub.publish(self._grid(stamp))
        self._plan_pub.publish(self._plan(stamp))
        self._route_pub.publish(route)
        self._state_pub.publish(state)
        self._robot_description_pub.publish(self._robot_description())
        self._footprint_pub.publish(self._footprint(stamp))
        self._trajectory_pub.publish(self._trajectories(stamp))
        self._controller_pub.publish(Bool(data=True))
        self._perception_pub.publish(Bool(data=True))
        self._localization_pub.publish(Bool(data=True))
        speed = TwistStamped()
        speed.header.stamp, speed.header.frame_id = stamp, 'base_footprint'
        speed.twist.linear.x = 0.20 if self._state_name == 'NAVIGATING' else 0.0
        self._safe_speed_pub.publish(speed)
        gate = DiagnosticStatus()
        gate.name, gate.message = 'operator_visualization_demo/gate', self._gate_reason
        gate.values = [KeyValue(key='state', value=self._gate_state)]
        self._gate_pub.publish(gate)
        self._tf.sendTransform(transform)

    def _start(self, _request, response):
        if self._state_name != 'RECEIVED':
            response.success, response.message = False, 'START is legal only from RECEIVED'
            return response
        self._state_name, (self._gate_state, self._gate_reason) = 'NAVIGATING', self._normal_gate
        response.success, response.message = True, 'demo mission started'
        return response

    def _pause(self, request, response):
        if request.data:
            if self._state_name != 'NAVIGATING':
                response.success, response.message = False, 'PAUSE is legal only from NAVIGATING'
                return response
            self._state_name, (self._gate_state, self._gate_reason) = 'PAUSED', ('DISARMED_ZERO', 'MISSION_PAUSED_DEMO')
            response.success, response.message = True, 'demo mission paused'
            return response
        if self._state_name != 'PAUSED':
            response.success, response.message = False, 'RESUME is legal only from PAUSED'
            return response
        self._state_name, (self._gate_state, self._gate_reason) = 'NAVIGATING', self._normal_gate
        response.success, response.message = True, 'demo mission resumed'
        return response

    def _cancel(self, _request, response):
        if self._state_name not in _CANCELLABLE:
            response.success, response.message = False, 'CANCEL is not legal in the current demo state'
            return response
        self._state_name, (self._gate_state, self._gate_reason) = 'CANCELLED', ('DISARMED_ZERO', 'MISSION_CANCELLED_DEMO')
        response.success, response.message = True, 'demo mission cancelled'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegratedDemoFixture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
