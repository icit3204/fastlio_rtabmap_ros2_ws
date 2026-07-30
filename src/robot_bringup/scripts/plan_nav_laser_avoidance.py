#!/usr/bin/env python3
"""Generate an active path from /plan_nav with temporary 2D-lidar avoidance."""

import math
import time
import copy
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


class PlanNavLaserAvoidance(Node):
    def __init__(self):
        super().__init__('plan_nav_laser_avoidance')

        self.declare_parameter('input_path_topic', '/plan_nav')
        self.declare_parameter('output_path_topic', '/active_plan')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('scan_timeout', 0.7)

        self.declare_parameter('obstacle_enter_distance', 1.5)
        self.declare_parameter('obstacle_exit_distance', 1.6)
        self.declare_parameter('emergency_stop_distance', 1.0)
        self.declare_parameter('rear_self_filter_distance', 1.0)
        self.declare_parameter('front_sector_deg', 50.0)
        self.declare_parameter('front_channel_half_width', 0.5)
        self.declare_parameter('side_inner_deg', 25.0)
        self.declare_parameter('side_outer_deg', 50.0)
        self.declare_parameter('side_clearance_min', 0.8)
        self.declare_parameter('side_clearance_margin', 0.3)

        self.declare_parameter('emergency_stop_duration', 0.8)
        self.declare_parameter('avoidance_depart_angle_deg', 45.0)
        self.declare_parameter('avoidance_depart_distance', 1.414)
        self.declare_parameter('lateral_offset', 2.0)
        self.declare_parameter('min_lateral_offset', 1.4)
        self.declare_parameter('avoidance_control_length_ratio', 0.30)
        self.declare_parameter('avoidance_control_length_min', 0.8)
        self.declare_parameter('avoidance_control_length_max', 2.2)
        self.declare_parameter('return_margin', 2.0)
        self.declare_parameter('min_return_distance', 3.0)
        self.declare_parameter('max_return_distance', 7.0)
        self.declare_parameter('path_sample_step', 0.15)

        self.declare_parameter('clear_duration', 1.0)
        self.declare_parameter('return_tolerance', 1.0)
        self.declare_parameter('path_lateral_tolerance', 1.0)
        self.declare_parameter('heading_tolerance_deg', 60.0)
        self.declare_parameter('obstacle_confirm_frames', 3)

        self.input_path_topic = str(self.get_parameter('input_path_topic').value)
        self.output_path_topic = str(self.get_parameter('output_path_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.enter_distance = float(self.get_parameter('obstacle_enter_distance').value)
        self.exit_distance = float(self.get_parameter('obstacle_exit_distance').value)
        self.emergency_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.rear_self_filter_distance = max(0.0, float(self.get_parameter('rear_self_filter_distance').value))
        self.front_sector_deg = self._clamp_front_deg(float(self.get_parameter('front_sector_deg').value))
        self.front_channel_half_width = max(0.05, float(self.get_parameter('front_channel_half_width').value))
        self.side_inner_deg = self._clamp_front_deg(float(self.get_parameter('side_inner_deg').value))
        self.side_outer_deg = self._clamp_front_deg(float(self.get_parameter('side_outer_deg').value))
        if self.side_outer_deg < self.side_inner_deg:
            self.side_outer_deg = self.side_inner_deg
        self.side_clearance_min = float(self.get_parameter('side_clearance_min').value)
        self.side_clearance_margin = float(self.get_parameter('side_clearance_margin').value)
        self.stop_duration = max(0.0, float(self.get_parameter('emergency_stop_duration').value))
        self.depart_angle = math.radians(float(self.get_parameter('avoidance_depart_angle_deg').value))
        self.depart_distance = max(0.2, float(self.get_parameter('avoidance_depart_distance').value))
        self.lateral_offset = float(self.get_parameter('lateral_offset').value)
        self.min_lateral_offset = float(self.get_parameter('min_lateral_offset').value)
        self.control_length_ratio = float(self.get_parameter('avoidance_control_length_ratio').value)
        self.control_length_min = float(self.get_parameter('avoidance_control_length_min').value)
        self.control_length_max = float(self.get_parameter('avoidance_control_length_max').value)
        self.return_margin = float(self.get_parameter('return_margin').value)
        self.min_return_distance = float(self.get_parameter('min_return_distance').value)
        self.max_return_distance = float(self.get_parameter('max_return_distance').value)
        self.path_sample_step = float(self.get_parameter('path_sample_step').value)
        self.clear_duration = float(self.get_parameter('clear_duration').value)
        self.return_tolerance = float(self.get_parameter('return_tolerance').value)
        self.path_lateral_tolerance = float(self.get_parameter('path_lateral_tolerance').value)
        self.heading_tolerance = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.confirm_frames = int(self.get_parameter('obstacle_confirm_frames').value)

        self.path: Optional[Path] = None
        self.latest_scan_time = 0.0
        self.front_min = math.inf
        self.obstacle_angle = 0.0
        self.obstacle_lateral = 0.0
        self.left_clearance = math.inf
        self.right_clearance = math.inf
        self.front_hit_count = 0
        self.clear_since: Optional[float] = None

        self.mode = 'NORMAL'
        self.avoidance_path: Optional[Path] = None
        self.return_point: Optional[Pose2D] = None
        self.return_side = 0
        self.stop_until = 0.0
        self.latched_obstacle_angle = 0.0
        self.latched_obstacle_lateral = 0.0
        self.latched_obstacle_distance = math.inf
        self.latched_side = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.path_sub = self.create_subscription(Path, self.input_path_topic, self.path_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.path_pub = self.create_publisher(Path, self.output_path_topic, 10)
        self.state_pub = self.create_publisher(String, '/laser_avoidance_state', 10)
        self.return_marker_pub = self.create_publisher(Marker, '/laser_avoidance_return_point', 10)

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f'plan_nav laser avoidance: {self.input_path_topic} -> {self.output_path_topic}, '
            f'scan={self.scan_topic}, enabled={self.enabled}')

    def path_callback(self, msg: Path):
        self.path = msg

    def scan_callback(self, scan: LaserScan):
        now = time.monotonic()
        self.latest_scan_time = now
        self.front_min, self.obstacle_angle, self.obstacle_lateral = self._front_obstacle(scan)
        self.left_clearance = math.inf
        self.right_clearance = math.inf

        if self.front_min < self.enter_distance:
            self.front_hit_count = min(self.front_hit_count + 1, self.confirm_frames)
            self.clear_since = None
        else:
            self.front_hit_count = 0
            if self.front_min > self.exit_distance:
                if self.clear_since is None:
                    self.clear_since = now
            else:
                self.clear_since = None

    def timer_callback(self):
        if self.path is None or len(self.path.poses) < 2:
            self._publish_state('WAITING_PATH')
            return

        robot = self._lookup_robot_pose()
        if robot is None:
            self._publish_path(self.path)
            self._publish_state('NO_TF_PASS_THROUGH')
            return

        if not self.enabled:
            self.mode = 'NORMAL'
            self._publish_path(self.path)
            self._publish_state('DISABLED_PASS_THROUGH')
            return

        points = self._path_to_points(self.path)
        scan_recent = (time.monotonic() - self.latest_scan_time) <= self.scan_timeout
        now = time.monotonic()

        if self.mode == 'STOPPING':
            self._publish_path(self._hold_path(robot))
            if now < self.stop_until:
                self._publish_state(
                    f'STOPPING_EMERGENCY front={self._fmt_range(self.front_min)} '
                    f'latched_front={self._fmt_range(self.latched_obstacle_distance)} '
                    f'latched_y={self.latched_obstacle_lateral:.2f}m '
                    f'latched_angle={math.degrees(self.latched_obstacle_angle):.1f}deg')
                return
            if self._start_avoidance(points, robot):
                self._publish_path(self.avoidance_path)
                side_name = 'LEFT' if self.return_side > 0 else 'RIGHT'
                self._publish_state(
                    f'AVOIDING_{side_name} front={self._fmt_range(self.front_min)} '
                    f'latched_front={self._fmt_range(self.latched_obstacle_distance)} '
                    f'latched_y={self.latched_obstacle_lateral:.2f}m')
                self._publish_return_marker()
                return
            self.mode = 'WAITING'
            self._publish_state(
                f'WAITING_AVOIDANCE_BUILD_FAILED latched_front={self._fmt_range(self.latched_obstacle_distance)} '
                f'latched_angle={math.degrees(self.latched_obstacle_angle):.1f}deg')
            return

        if self.mode == 'AVOIDING':
            if self._avoidance_finished(points, robot, scan_recent):
                self.mode = 'NORMAL'
                self.avoidance_path = None
                self.return_point = None
                self.return_side = 0
                self.latched_side = 0
                self.latched_obstacle_distance = math.inf
                self._publish_path(self.path)
                self._publish_state('NORMAL_REJOINED_PLAN_NAV')
                return
            if self.avoidance_path is not None:
                self._publish_path(self.avoidance_path)
                side_name = 'LEFT' if self.return_side > 0 else 'RIGHT'
                self._publish_state(f'AVOIDING_{side_name} front={self._fmt_range(self.front_min)}')
                self._publish_return_marker()
                return

        if self.mode == 'WAITING':
            if scan_recent and self.front_min > self.exit_distance and self._clear_duration_ok():
                self.mode = 'NORMAL'
                self._publish_path(self.path)
                self._publish_state('NORMAL_FRONT_CLEAR')
                return
            if self._should_start_avoidance(scan_recent):
                if self._start_avoidance(points, robot):
                    self._publish_path(self.avoidance_path)
                    side_name = 'LEFT' if self.return_side > 0 else 'RIGHT'
                    self._publish_state(f'AVOIDING_{side_name} front={self._fmt_range(self.front_min)}')
                    self._publish_return_marker()
                    return
            self._publish_path(self._hold_path(robot))
            self._publish_state(
                f'WAITING_BLOCKED front={self._fmt_range(self.front_min)} '
                f'left={self._fmt_range(self.left_clearance)} right={self._fmt_range(self.right_clearance)}')
            return

        if self._should_stop_before_avoidance(scan_recent):
            self.mode = 'STOPPING'
            self.stop_until = now + self.stop_duration
            self.latched_obstacle_angle = self.obstacle_angle
            self.latched_obstacle_lateral = self.obstacle_lateral
            self.latched_obstacle_distance = self.front_min
            self.latched_side = self._choose_side()
            self._publish_path(self._hold_path(robot))
            self._publish_state(
                f'STOPPING_EMERGENCY front={self._fmt_range(self.front_min)} '
                f'obstacle_angle={math.degrees(self.obstacle_angle):.1f}deg')
            return

        if self._obstacle_detected(scan_recent):
            self._publish_path(self.path)
            self._publish_state(
                f'APPROACHING_OBSTACLE front={self._fmt_range(self.front_min)} '
                f'stop_distance={self.emergency_distance:.2f}m '
                f'obstacle_y={self.obstacle_lateral:.2f}m '
                f'obstacle_angle={math.degrees(self.obstacle_angle):.1f}deg')
            return

        self.mode = 'NORMAL'
        self._publish_path(self.path)
        self._publish_state(f'NORMAL front={self._fmt_range(self.front_min)}')

    def _obstacle_detected(self, scan_recent: bool) -> bool:
        return scan_recent and self.front_hit_count >= self.confirm_frames and self.front_min < self.enter_distance

    def _should_stop_before_avoidance(self, scan_recent: bool) -> bool:
        return self._obstacle_detected(scan_recent) and self.front_min <= self.emergency_distance

    def _should_start_avoidance(self, scan_recent: bool) -> bool:
        return self._obstacle_detected(scan_recent)

    def _start_avoidance(self, points: List[Pose2D], robot: Pose2D) -> bool:
        side = self.latched_side if self.latched_side != 0 else self._choose_side()
        if side == 0:
            return False

        nearest_idx = self._nearest_index(points, robot)
        if math.isfinite(self.latched_obstacle_distance):
            obstacle_distance = self.latched_obstacle_distance
        else:
            obstacle_distance = self.front_min if math.isfinite(self.front_min) else self.enter_distance
        return_distance = obstacle_distance + self.return_margin
        return_distance = max(self.min_return_distance, min(self.max_return_distance, return_distance))
        return_idx = self._index_at_distance(points, nearest_idx, return_distance)
        if return_idx <= nearest_idx + 2:
            return False

        self.avoidance_path = self._build_avoidance_path(points, robot, nearest_idx, return_idx, side)
        if self.avoidance_path is None:
            return False

        self.mode = 'AVOIDING'
        self.return_side = side
        self.return_point = points[return_idx]
        self.clear_since = None
        side_name = 'left' if side > 0 else 'right'
        self.get_logger().warn(
            f'Obstacle ahead {self.front_min:.2f} m, start {side_name} avoidance, '
            f'latched_angle={math.degrees(self.latched_obstacle_angle):.1f} deg, '
            f'depart_angle={math.degrees(self.depart_angle):.1f} deg, '
            f'depart_distance={self.depart_distance:.2f} m, return_idx={return_idx}, '
            f'return_distance={return_distance:.2f} m')
        return True

    def _choose_side(self) -> int:
        deadband = 0.05
        if self.obstacle_lateral > deadband:
            return -1
        if self.obstacle_lateral < -deadband:
            return 1
        return 1

    def _avoidance_finished(self, points: List[Pose2D], robot: Pose2D, scan_recent: bool) -> bool:
        if self.return_point is None:
            return False
        if distance(robot, self.return_point) > self.return_tolerance:
            return False
        nearest_idx = self._nearest_index(points, robot)
        if distance(robot, points[nearest_idx]) > self.path_lateral_tolerance:
            return False
        path_yaw = self._path_yaw(points, nearest_idx)
        if abs(normalize_angle(robot.yaw - path_yaw)) > self.heading_tolerance:
            return False
        return True

    def _clear_duration_ok(self) -> bool:
        return self.clear_since is not None and (time.monotonic() - self.clear_since) >= self.clear_duration

    def _lookup_robot_pose(self) -> Optional[Pose2D]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.05))
        except Exception as exc:
            self.get_logger().warn(f'Cannot lookup {self.map_frame}->{self.base_frame}: {exc}', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        yaw = yaw_from_quaternion(tf.transform.rotation)
        return Pose2D(float(t.x), float(t.y), yaw)

    def _path_to_points(self, path: Path) -> List[Pose2D]:
        points = []
        for pose_stamped in path.poses:
            p = pose_stamped.pose.position
            yaw = yaw_from_quaternion(pose_stamped.pose.orientation)
            points.append(Pose2D(float(p.x), float(p.y), yaw))
        return points

    def _build_avoidance_path(
        self,
        points: List[Pose2D],
        robot: Pose2D,
        nearest_idx: int,
        return_idx: int,
        side: int,
    ) -> Optional[Path]:
        return_pt = points[return_idx]
        path_yaw = self._path_yaw(points, nearest_idx)
        depart_yaw = normalize_angle(path_yaw + side * self.depart_angle)
        p0 = Pose2D(robot.x, robot.y, robot.yaw)
        p_avoid = Pose2D(
            robot.x + self.depart_distance * math.cos(depart_yaw),
            robot.y + self.depart_distance * math.sin(depart_yaw),
            depart_yaw)

        active_points: List[Pose2D] = []
        active_points.extend(self._line_samples(p0, p_avoid, include_start=True))
        active_points.extend(self._line_samples(p_avoid, return_pt, include_start=False))
        active_points.extend(points[return_idx + 1:])
        self._assign_yaws(active_points)
        if len(active_points) < 2:
            return None
        return self._points_to_path(active_points)

    def _hold_path(self, robot: Pose2D) -> Path:
        hold_points = [
            Pose2D(robot.x, robot.y, robot.yaw),
            Pose2D(robot.x + 0.01 * math.cos(robot.yaw), robot.y + 0.01 * math.sin(robot.yaw), robot.yaw),
        ]
        return self._points_to_path(hold_points)

    @staticmethod
    def _bezier(p0: Pose2D, p1: Pose2D, p2: Pose2D, p3: Pose2D, u: float) -> Pose2D:
        v = 1.0 - u
        x = v ** 3 * p0.x + 3.0 * v ** 2 * u * p1.x + 3.0 * v * u ** 2 * p2.x + u ** 3 * p3.x
        y = v ** 3 * p0.y + 3.0 * v ** 2 * u * p1.y + 3.0 * v * u ** 2 * p2.y + u ** 3 * p3.y
        return Pose2D(x, y)

    def _line_samples(self, start: Pose2D, end: Pose2D, include_start: bool) -> List[Pose2D]:
        length = distance(start, end)
        samples = max(1, int(math.ceil(length / max(0.05, self.path_sample_step))))
        points = []
        first = 0 if include_start else 1
        for i in range(first, samples + 1):
            u = i / float(samples)
            x = start.x + (end.x - start.x) * u
            y = start.y + (end.y - start.y) * u
            points.append(Pose2D(x, y))
        return points

    @staticmethod
    def _assign_yaws(points: List[Pose2D]):
        for i in range(len(points) - 1):
            dx = points[i + 1].x - points[i].x
            dy = points[i + 1].y - points[i].y
            if math.hypot(dx, dy) > 1e-4:
                points[i].yaw = math.atan2(dy, dx)
        if len(points) > 1:
            points[-1].yaw = points[-2].yaw

    def _points_to_path(self, points: List[Pose2D]) -> Path:
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point.x)
            pose.pose.position.y = float(point.y)
            pose.pose.position.z = 0.0
            qx, qy, qz, qw = quaternion_from_yaw(point.yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        return path

    def _publish_path(self, path: Path):
        stamped_path = copy.deepcopy(path)
        now_msg = self.get_clock().now().to_msg()
        if not stamped_path.header.frame_id:
            stamped_path.header.frame_id = self.map_frame
        stamped_path.header.stamp = now_msg
        for pose in stamped_path.poses:
            if not pose.header.frame_id:
                pose.header.frame_id = stamped_path.header.frame_id
            pose.header.stamp = now_msg
        self.path_pub.publish(stamped_path)

    def _publish_state(self, text: str):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def _publish_return_marker(self):
        if self.return_point is None:
            return
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'laser_avoidance_return'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.return_point.x
        marker.pose.position.y = self.return_point.y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 0.1
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.return_marker_pub.publish(marker)

    @staticmethod
    def _nearest_index(points: List[Pose2D], pose: Pose2D) -> int:
        best_idx = 0
        best_dist = float('inf')
        for i, point in enumerate(points):
            d = (point.x - pose.x) ** 2 + (point.y - pose.y) ** 2
            if d < best_dist:
                best_idx = i
                best_dist = d
        return best_idx

    @staticmethod
    def _index_at_distance(points: List[Pose2D], start_idx: int, target_distance: float) -> int:
        if start_idx >= len(points) - 1:
            return len(points) - 1
        acc = 0.0
        for i in range(start_idx + 1, len(points)):
            acc += distance(points[i - 1], points[i])
            if acc >= target_distance:
                return i
        return len(points) - 1

    @staticmethod
    def _path_yaw(points: List[Pose2D], idx: int) -> float:
        if idx < len(points) - 1:
            dx = points[idx + 1].x - points[idx].x
            dy = points[idx + 1].y - points[idx].y
        elif idx > 0:
            dx = points[idx].x - points[idx - 1].x
            dy = points[idx].y - points[idx - 1].y
        else:
            return points[idx].yaw
        if math.hypot(dx, dy) < 1e-4:
            return points[idx].yaw
        return math.atan2(dy, dx)

    def _sector_min(self, scan: LaserScan, min_deg: float, max_deg: float) -> float:
        values = self._sector_values(scan, min_deg, max_deg)
        return min(values) if values else math.inf

    def _sector_score(self, scan: LaserScan, min_deg: float, max_deg: float) -> float:
        values = self._sector_values(scan, min_deg, max_deg)
        if not values:
            return float(scan.range_max) if scan.range_max > 0.0 else self.enter_distance
        values.sort()
        idx = min(len(values) - 1, max(0, int(len(values) * 0.30)))
        return values[idx]

    def _sector_values(self, scan: LaserScan, min_deg: float, max_deg: float) -> List[float]:
        min_deg = max(-90.0, min(90.0, min_deg))
        max_deg = max(-90.0, min(90.0, max_deg))
        if min_deg > max_deg:
            min_deg, max_deg = max_deg, min_deg
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        upper_range = scan.range_max if scan.range_max > 0.0 else float('inf')
        values = []
        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value <= max(0.0, scan.range_min) or value >= upper_range:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if self._is_rear_self_point(angle, float(value)):
                continue
            if min_rad <= angle <= max_rad:
                values.append(float(value))
        return values

    def _front_obstacle(self, scan: LaserScan) -> Tuple[float, float, float]:
        upper_range = min(self.enter_distance, scan.range_max if scan.range_max > 0.0 else self.enter_distance)
        best_x = math.inf
        weighted_angle_sum = 0.0
        weighted_y_sum = 0.0
        weight_sum = 0.0
        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            value = float(value)
            if value <= max(0.0, scan.range_min) or value > upper_range:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            x = value * math.cos(angle)
            y = value * math.sin(angle)
            if x <= 0.0 or x > self.enter_distance:
                continue
            if abs(y) > self.front_channel_half_width:
                continue
            weight = 1.0 / max(0.05, x)
            weighted_angle_sum += math.atan2(y, x) * weight
            weighted_y_sum += y * weight
            weight_sum += weight
            if x < best_x:
                best_x = x
        if not math.isfinite(best_x):
            return math.inf, 0.0, 0.0
        if weight_sum > 0.0:
            return best_x, weighted_angle_sum / weight_sum, weighted_y_sum / weight_sum
        return best_x, 0.0, 0.0

    def _is_rear_self_point(self, angle_rad: float, range_m: float) -> bool:
        return abs(angle_rad) > (math.pi * 0.5) and range_m <= self.rear_self_filter_distance

    @staticmethod
    def _clamp_front_deg(value: float) -> float:
        return max(0.0, min(90.0, value))

    @staticmethod
    def _fmt_range(value: float) -> str:
        return 'inf' if not math.isfinite(value) else f'{value:.2f}m'


def main():
    rclpy.init()
    node = PlanNavLaserAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
