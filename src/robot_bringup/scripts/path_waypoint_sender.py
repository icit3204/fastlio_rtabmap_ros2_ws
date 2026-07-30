#!/usr/bin/python3.10
"""Send initial pose and path waypoints to Nav2 for offline navigation.

1. Publishes /initialpose at the first waypoint
2. Tracks the extracted path by sending lookahead NavigateToPose goals
3. Nav2 plans around obstacles → MPPI avoids them → cmd_vel moves robot
"""
import math
import os
import yaml
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw):
    from geometry_msgs.msg import Quaternion
    return Quaternion(z=float(math.sin(yaw / 2.0)), w=float(math.cos(yaw / 2.0)))


class PathWaypointSender(Node):
    def __init__(self):
        super().__init__('path_waypoint_sender')
        self.declare_parameter('input', '/data/maps/db/path_waypoints.yaml')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('max_waypoint_step', 8.0)
        self.declare_parameter('auto_goal_distance', 8.0)
        self.declare_parameter('auto_replan_on_clicked_obstacle', True)
        self.declare_parameter('clicked_obstacle_topic', '/clicked_point')
        self.declare_parameter('auto_replan_delay', 0.8)
        self.declare_parameter('clicked_obstacle_width', 1.6)
        self.declare_parameter('clicked_obstacle_height', 1.6)
        self.declare_parameter('dynamic_goal_clearance', 0.8)
        self.declare_parameter('dynamic_goal_block_radius', 0.0)

        self.input_path = self.get_parameter('input').value
        self.frame_id = self.get_parameter('frame_id').value
        self.map_yaml = self.get_parameter('map_yaml').value
        self.max_waypoint_step = float(self.get_parameter('max_waypoint_step').value)
        self.auto_goal_distance = float(self.get_parameter('auto_goal_distance').value)
        self.auto_replan_on_clicked_obstacle = bool(
            self.get_parameter('auto_replan_on_clicked_obstacle').value)
        self.clicked_obstacle_topic = self.get_parameter('clicked_obstacle_topic').value
        self.auto_replan_delay = float(self.get_parameter('auto_replan_delay').value)
        self.clicked_obstacle_width = float(self.get_parameter('clicked_obstacle_width').value)
        self.clicked_obstacle_height = float(self.get_parameter('clicked_obstacle_height').value)
        self.dynamic_goal_clearance = float(self.get_parameter('dynamic_goal_clearance').value)
        configured_block_radius = float(self.get_parameter('dynamic_goal_block_radius').value)
        obstacle_half_diagonal = math.hypot(
            self.clicked_obstacle_width / 2.0, self.clicked_obstacle_height / 2.0)
        self.dynamic_goal_block_radius = configured_block_radius
        if self.dynamic_goal_block_radius <= 0.0:
            self.dynamic_goal_block_radius = obstacle_half_diagonal + self.dynamic_goal_clearance

        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.init_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_cb, 10)
        self.clicked_obstacle_sub = None
        if self.auto_replan_on_clicked_obstacle:
            self.clicked_obstacle_sub = self.create_subscription(
                PointStamped, self.clicked_obstacle_topic, self.clicked_obstacle_cb, 10)

        self.timer = self.create_timer(2.0, self.run)  # Delay to let nodes start
        self.sent = False
        self.waypoints = []
        self.goal_index = 0
        self.mode = 'auto'
        self.current_goal_handle = None
        self.ignore_initialpose_until = None
        self.pending_manual_goal = None
        self.pending_manual_timer = None
        self.last_manual_goal = None
        self.manual_goal_retries = 0
        self.auto_route_timer = None
        self.auto_route_retries = 0
        self.current_auto_target_index = None
        self.auto_replan_timer = None
        self.auto_replan_pending = False
        self.auto_replan_target_index = None
        self.goal_sequence = 0
        self.active_goal_sequence = None
        self.dynamic_obstacles = []

    def run(self):
        if self.sent:
            return
        self.sent = True
        self.timer.cancel()

        # Load waypoints
        with open(self.input_path, 'r') as f:
            data = yaml.safe_load(f)
        waypoints = data['waypoints']
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints')

        if not waypoints:
            self.get_logger().error('No waypoints found!')
            return

        waypoints = self.keep_longest_continuous_segment(waypoints)
        waypoints = self.filter_waypoints_on_free_map(waypoints)
        if not waypoints:
            self.get_logger().error('No reachable waypoints after map filtering!')
            return

        # Publish initial pose (first waypoint)
        wp0 = waypoints[0]
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = self.frame_id
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.pose.position.x = float(wp0['x'])
        init_pose.pose.pose.position.y = float(wp0['y'])
        init_pose.pose.pose.orientation = yaw_to_quat(
            self.trajectory_yaw(waypoints, 0, fallback=float(wp0['yaw'])))
        # Set covariance
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] = 0.068

        # Publish a few times to make sure Nav2 receives it
        self.ignore_initialpose_until = self.get_clock().now() + Duration(seconds=3.0)
        for _ in range(5):
            self.init_pub.publish(init_pose)
        self.get_logger().info(f'Initial pose set at ({wp0["x"]:.2f}, {wp0["y"]:.2f})')

        # Wait for action server
        self.get_logger().info('Waiting for Nav2 action servers...')
        if not self.pose_action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().fatal('NavigateToPose action server not available!')
            return

        self.waypoints = waypoints
        self.goal_index = 1 if len(waypoints) > 1 else 0
        self.send_auto_route()

    def make_pose_stamped(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = yaw_to_quat(yaw)
        return ps

    def trajectory_yaw(self, waypoints, index, fallback=0.0):
        if not waypoints:
            return fallback
        index = max(0, min(index, len(waypoints) - 1))

        current = waypoints[index]
        for next_index in range(index + 1, len(waypoints)):
            dx = float(waypoints[next_index]['x']) - float(current['x'])
            dy = float(waypoints[next_index]['y']) - float(current['y'])
            if math.hypot(dx, dy) > 0.20:
                return math.atan2(dy, dx)

        for prev_index in range(index - 1, -1, -1):
            dx = float(current['x']) - float(waypoints[prev_index]['x'])
            dy = float(current['y']) - float(waypoints[prev_index]['y'])
            if math.hypot(dx, dy) > 0.20:
                return math.atan2(dy, dx)

        return fallback

    def send_auto_route(self):
        if self.mode != 'auto':
            return
        if self.goal_index >= len(self.waypoints):
            self.get_logger().info('Automatic path completed')
            return

        target_index = self.select_auto_target_index()
        self.send_auto_target(target_index)

    def send_auto_target(self, target_index):
        target_index = self.find_unblocked_target_index(target_index)
        wp = self.waypoints[target_index]
        self.current_auto_target_index = target_index
        label = f'auto path target {target_index + 1}/{len(self.waypoints)}'
        yaw = self.trajectory_yaw(self.waypoints, target_index, fallback=float(wp['yaw']))
        self.send_goal(float(wp['x']), float(wp['y']), yaw, auto=True, label=label)

    def select_auto_target_index(self):
        if self.goal_index >= len(self.waypoints) - 1:
            return len(self.waypoints) - 1

        distance = 0.0
        last = self.waypoints[max(0, self.goal_index - 1)]
        for idx in range(self.goal_index, len(self.waypoints)):
            wp = self.waypoints[idx]
            distance += math.hypot(float(wp['x']) - float(last['x']),
                                   float(wp['y']) - float(last['y']))
            if distance >= self.auto_goal_distance:
                return self.find_unblocked_target_index(idx)
            last = wp
        return self.find_unblocked_target_index(len(self.waypoints) - 1)

    def waypoint_blocked_by_dynamic_obstacle(self, wp):
        if not self.dynamic_obstacles:
            return False
        x = float(wp['x'])
        y = float(wp['y'])
        for obstacle in self.dynamic_obstacles:
            if math.hypot(x - obstacle[0], y - obstacle[1]) <= self.dynamic_goal_block_radius:
                return True
        return False

    def find_unblocked_target_index(self, preferred_index):
        if not self.waypoints:
            return preferred_index
        preferred_index = max(0, min(preferred_index, len(self.waypoints) - 1))
        if not self.dynamic_obstacles:
            return preferred_index

        for idx in range(preferred_index, len(self.waypoints)):
            if not self.waypoint_blocked_by_dynamic_obstacle(self.waypoints[idx]):
                if idx != preferred_index:
                    self.get_logger().info(
                        f'Auto target waypoint {preferred_index + 1}/{len(self.waypoints)} '
                        f'is inside a clicked obstacle safety radius '
                        f'({self.dynamic_goal_block_radius:.2f}m); using waypoint '
                        f'{idx + 1}/{len(self.waypoints)} instead')
                return idx

        for idx in range(preferred_index - 1, -1, -1):
            if not self.waypoint_blocked_by_dynamic_obstacle(self.waypoints[idx]):
                self.get_logger().warn(
                    f'No later unblocked waypoint found after {preferred_index + 1}; '
                    f'using earlier waypoint {idx + 1}/{len(self.waypoints)}')
                return idx

        self.get_logger().error(
            'All automatic waypoints are inside clicked obstacle safety radii; '
            'keeping the preferred target')
        return preferred_index

    def schedule_auto_route_retry(self):
        if self.auto_route_timer is not None:
            self.auto_route_timer.cancel()
        self.auto_route_timer = self.create_timer(2.0, self.retry_auto_route)

    def retry_auto_route(self):
        if self.auto_route_timer is not None:
            self.auto_route_timer.cancel()
            self.auto_route_timer = None
        self.send_auto_route()

    def schedule_auto_replan(self):
        if self.auto_replan_timer is not None:
            self.auto_replan_timer.cancel()
        self.auto_replan_timer = self.create_timer(
            max(0.1, self.auto_replan_delay), self.perform_auto_replan)

    def perform_auto_replan(self):
        if self.auto_replan_timer is not None:
            self.auto_replan_timer.cancel()
            self.auto_replan_timer = None
        if self.mode != 'auto' or not self.auto_replan_pending:
            self.auto_replan_pending = False
            return
        if self.auto_replan_target_index is None or not self.waypoints:
            self.auto_replan_pending = False
            self.send_auto_route()
            return

        target_index = min(self.auto_replan_target_index, len(self.waypoints) - 1)
        self.auto_replan_pending = False
        self.auto_replan_target_index = None
        self.get_logger().info(
            f'Replanning automatic path around clicked obstacle to waypoint '
            f'{target_index + 1}/{len(self.waypoints)}')
        self.send_auto_target(target_index)

    def send_goal(self, x, y, yaw, auto=False, label='manual goal'):
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose_stamped(x, y, yaw)
        self.goal_sequence += 1
        goal_sequence = self.goal_sequence

        if not self.pose_action_client.server_is_ready():
            self.get_logger().info('Waiting for NavigateToPose action server...')
            if not self.pose_action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error('Action server not available for requested goal')
                return

        self.get_logger().info(f'Sending {label} to Nav2: ({x:.2f}, {y:.2f})')
        self._send_goal_future = self.pose_action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(
            lambda future: self._goal_response_cb(future, auto=auto, goal_sequence=goal_sequence))

    def cancel_current_goal(self):
        if self.current_goal_handle is None:
            return False
        try:
            self.current_goal_handle.cancel_goal_async()
            self.get_logger().info('Canceled current Nav2 goal')
        except Exception as exc:
            self.get_logger().warn(f'Failed to cancel current Nav2 goal: {exc}')
        self.current_goal_handle = None
        return True

    def switch_to_manual(self, reason):
        if self.mode != 'manual':
            self.get_logger().info(f'Stopping automatic trajectory: {reason}')
        self.mode = 'manual'
        return self.cancel_current_goal()

    def initialpose_cb(self, msg):
        if self.ignore_initialpose_until is not None:
            if self.get_clock().now() < self.ignore_initialpose_until:
                return
            self.ignore_initialpose_until = None
        self.switch_to_manual('RViz 2D Pose Estimate received')

    def goal_pose_cb(self, msg):
        canceled = self.switch_to_manual('RViz 2D Goal Pose received')
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.pending_manual_goal = (float(msg.pose.position.x), float(msg.pose.position.y), yaw)
        self.last_manual_goal = self.pending_manual_goal
        self.manual_goal_retries = 0
        if canceled:
            self.schedule_pending_manual_goal(3.0)
        else:
            self.send_pending_manual_goal()

    def clicked_obstacle_cb(self, msg):
        if self.mode != 'auto':
            return
        self.dynamic_obstacles.append((float(msg.point.x), float(msg.point.y)))
        if self.auto_replan_pending:
            return
        if self.current_goal_handle is None or self.current_auto_target_index is None:
            return
        self.auto_replan_pending = True
        self.auto_replan_target_index = self.find_unblocked_target_index(
            self.current_auto_target_index)
        self.get_logger().info(
            'Clicked obstacle received during automatic path tracking; '
            'canceling the current segment and requesting a fresh Nav2 plan')
        self.cancel_current_goal()
        self.schedule_auto_replan()

    def schedule_pending_manual_goal(self, delay_sec):
        if self.pending_manual_timer is not None:
            self.pending_manual_timer.cancel()
        self.pending_manual_timer = self.create_timer(delay_sec, self.send_pending_manual_goal)

    def send_pending_manual_goal(self):
        if self.pending_manual_timer is not None:
            self.pending_manual_timer.cancel()
            self.pending_manual_timer = None
        if self.pending_manual_goal is None:
            return
        x, y, yaw = self.pending_manual_goal
        self.pending_manual_goal = None
        self.send_goal(x, y, yaw, auto=False, label='RViz goal')

    def keep_longest_continuous_segment(self, waypoints):
        if len(waypoints) < 2 or self.max_waypoint_step <= 0:
            return waypoints

        segments = []
        start = 0
        for i in range(1, len(waypoints)):
            dx = float(waypoints[i]['x']) - float(waypoints[i - 1]['x'])
            dy = float(waypoints[i]['y']) - float(waypoints[i - 1]['y'])
            if math.hypot(dx, dy) > self.max_waypoint_step:
                segments.append((start, i))
                start = i
        segments.append((start, len(waypoints)))

        best_start, best_end = max(segments, key=lambda item: item[1] - item[0])
        skipped = len(waypoints) - (best_end - best_start)
        if skipped:
            self.get_logger().info(
                f'Dropped {skipped} waypoints outside the longest continuous segment '
                f'(max step={self.max_waypoint_step:.1f}m)')
        return waypoints[best_start:best_end]

    def filter_waypoints_on_free_map(self, waypoints):
        if not self.map_yaml:
            return waypoints

        try:
            with open(self.map_yaml, 'r') as f:
                meta = yaml.safe_load(f)
            pgm_path = os.path.join(os.path.dirname(os.path.abspath(self.map_yaml)), meta['image'])
            grid = self.read_pgm(pgm_path)
        except Exception as exc:
            self.get_logger().warn(f'Could not load map for waypoint filtering: {exc}')
            return waypoints

        resolution = float(meta['resolution'])
        origin_x = float(meta['origin'][0])
        origin_y = float(meta['origin'][1])
        y_top = origin_y + grid.shape[0] * resolution
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        negate = int(meta.get('negate', 0))

        filtered = []
        skipped = 0
        adjusted = 0
        for idx, wp in enumerate(waypoints):
            x = float(wp['x'])
            y = float(wp['y'])
            col = int((x - origin_x) / resolution)
            row = int((y_top - y) / resolution)
            if self.is_traversable(grid, row, col, negate, occupied_thresh):
                filtered.append(wp)
                continue

            adjusted_wp = self.nearest_traversable_waypoint(
                wp, grid, row, col, origin_x, y_top, resolution, negate, occupied_thresh)
            if adjusted_wp is not None:
                filtered.append(adjusted_wp)
                adjusted += 1
            else:
                skipped += 1

        if adjusted:
            self.get_logger().info(
                f'Moved {adjusted} waypoints from occupied pixels to nearby traversable pixels')
        if skipped:
            self.get_logger().info(f'Skipped {skipped} waypoints that are occupied in the Nav2 map')
        return filtered

    def is_traversable(self, grid, row, col, negate, occupied_thresh):
        if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
            return False
        pixel = int(grid[row, col])
        occ_prob = (255 - pixel) / 255.0 if negate == 0 else pixel / 255.0
        return occ_prob < occupied_thresh

    def nearest_traversable_waypoint(
            self, wp, grid, row, col, origin_x, y_top, resolution, negate, occupied_thresh):
        max_radius_cells = int(math.ceil(2.0 / resolution))
        best = None
        best_dist2 = None
        rows, cols = grid.shape

        for radius in range(1, max_radius_cells + 1):
            r0 = max(0, row - radius)
            r1 = min(rows - 1, row + radius)
            c0 = max(0, col - radius)
            c1 = min(cols - 1, col + radius)

            for rr in range(r0, r1 + 1):
                for cc in (c0, c1):
                    if self.is_traversable(grid, rr, cc, negate, occupied_thresh):
                        dist2 = (rr - row) * (rr - row) + (cc - col) * (cc - col)
                        if best_dist2 is None or dist2 < best_dist2:
                            best = (rr, cc)
                            best_dist2 = dist2
            for cc in range(c0 + 1, c1):
                for rr in (r0, r1):
                    if self.is_traversable(grid, rr, cc, negate, occupied_thresh):
                        dist2 = (rr - row) * (rr - row) + (cc - col) * (cc - col)
                        if best_dist2 is None or dist2 < best_dist2:
                            best = (rr, cc)
                            best_dist2 = dist2

            if best is not None:
                rr, cc = best
                adjusted = dict(wp)
                adjusted['x'] = origin_x + (cc + 0.5) * resolution
                adjusted['y'] = y_top - (rr + 0.5) * resolution
                return adjusted

        return None

    def read_pgm(self, pgm_path):
        with open(pgm_path, 'rb') as f:
            magic = f.readline().strip()
            if magic not in (b'P5', b'P2'):
                raise ValueError(f'Unsupported PGM format: {magic}')
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            width, height = map(int, line.split())
            maxval = int(f.readline().strip())
            if magic == b'P5':
                data = np.frombuffer(f.read(), dtype=np.uint8 if maxval < 256 else np.uint16).copy()
            else:
                data = np.fromfile(f, dtype=np.uint8, sep=' ')
        return data.reshape(height, width)

    def _goal_response_cb(self, future, auto=False, goal_sequence=None):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if auto and self.mode == 'auto' and self.auto_route_retries < 20:
                self.auto_route_retries += 1
                self.get_logger().warn(
                    f'Automatic path target rejected before Nav2 is fully active; retrying '
                    f'({self.auto_route_retries}/20)')
                self.schedule_auto_route_retry()
            elif not auto and self.last_manual_goal is not None and self.manual_goal_retries < 10:
                self.manual_goal_retries += 1
                self.pending_manual_goal = self.last_manual_goal
                self.get_logger().warn(
                    f'RViz goal rejected, retrying after Nav2 finishes current navigator '
                    f'({self.manual_goal_retries}/10)')
                self.schedule_pending_manual_goal(1.0)
            else:
                self.get_logger().error('Goal rejected!')
            return
        self.current_goal_handle = goal_handle
        self.active_goal_sequence = goal_sequence
        if auto:
            self.auto_route_retries = 0
        if not auto:
            self.last_manual_goal = None
        if auto and self.mode != 'auto':
            goal_handle.cancel_goal_async()
            return
        self.get_logger().info('Goal accepted — Nav2 is navigating with obstacle avoidance')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(
            lambda future: self._result_cb(future, auto=auto, goal_sequence=goal_sequence))

    def _result_cb(self, future, auto=False, goal_sequence=None):
        if goal_sequence is not None and self.active_goal_sequence != goal_sequence:
            self.get_logger().info('Ignoring stale Nav2 result from a superseded goal')
            return
        status = future.result().status
        self.current_goal_handle = None
        self.active_goal_sequence = None
        if auto and self.mode == 'auto' and self.auto_replan_pending:
            self.get_logger().info(
                f'Automatic path target interrupted for obstacle replan (status={status})')
            return
        if auto and self.mode == 'manual' and self.pending_manual_goal is not None:
            self.get_logger().info('Automatic path tracking is stopped; sending pending RViz goal')
            self.schedule_pending_manual_goal(0.2)
            return
        if not auto:
            self.get_logger().info(f'RViz goal finished (status={status})')
            return
        if self.mode != 'auto':
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.current_auto_target_index is not None:
                self.goal_index = self.current_auto_target_index + 1
            else:
                self.goal_index += 1
            self.current_auto_target_index = None
            self.get_logger().info(
                f'Automatic path target reached; continuing from waypoint '
                f'{self.goal_index + 1}/{len(self.waypoints)}')
            self.send_auto_route()
        else:
            if self.current_auto_target_index is not None:
                self.goal_index = min(self.current_auto_target_index + 1, len(self.waypoints))
            else:
                self.goal_index += 1
            self.current_auto_target_index = None
            self.get_logger().warn(
                f'Automatic path target ended with status={status}; trying a later path target')
            self.send_auto_route()


def main():
    rclpy.init()
    node = PathWaypointSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
