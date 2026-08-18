"""Isolated fake ROS terminal driver; no Nav2, Gate, mission manager, or fault."""

import argparse
import time

import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from parking_robot_interfaces.msg import MissionState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node


class Driver(Node):
    def __init__(self, inverted):
        super().__init__("phase4_p4e6b_terminal_fake_driver", enable_rosout=False)
        self.inverted = inverted
        self.uuid = bytes.fromhex("00112233445566778899aabbccddeeff")
        self.mission = self.create_publisher(MissionState, "/mission/state", 10)
        self.status = self.create_publisher(GoalStatusArray, "/navigate_to_pose/_action/status", 10)
        self.raw = self.create_publisher(Twist, "/cmd_vel_nav_raw", 10)
        self.safe = self.create_publisher(Twist, "/cmd_vel_nav_safe", 10)
        self.vehicle = self.create_publisher(TwistStamped, "/vehicle_cmd_safe", 10)
        self.mock = self.create_publisher(Float32MultiArray, "/wheelchair_control_command_mock", 10)
        self.odom = self.create_publisher(Odometry, "/Odometry", 10)
        self.diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.step = 0
        self.done = False
        self.timer = self.create_timer(.02, self.tick)

    def state(self, code, reason="", detail="", active=True):
        msg = MissionState(); msg.mission_id = "fake-mission"; msg.route_id = "fake-route"
        msg.state = code; msg.current_waypoint_index = 0; msg.total_waypoint_count = 1
        msg.active_goal_uuid = self.uuid.hex() if active else ""; msg.reason_code = reason; msg.detail = detail
        self.mission.publish(msg)

    def status_one(self, code):
        msg = GoalStatusArray(); row = GoalStatus(); row.goal_info = GoalInfo(); row.goal_info.goal_id.uuid = list(self.uuid); row.status = code
        msg.status_list = [row]; self.status.publish(msg)

    def physical(self, nonzero):
        t = Twist(); t.linear.x = .08 if nonzero else 0.; self.raw.publish(t); self.safe.publish(t)
        ts = TwistStamped(); ts.twist = t; self.vehicle.publish(ts)
        m = Float32MultiArray(); m.data = [.08, 0., 0.] if nonzero else [0., 0., 0.]; self.mock.publish(m)
        od = Odometry(); od.pose.pose.orientation.w = 1.; self.odom.publish(od)
        d = DiagnosticArray(); s = DiagnosticStatus(); s.name = "phase4_vehicle_cmd_fake_base"; s.message = "VALID"; s.level = DiagnosticStatus.OK; s.values = [KeyValue(key="applied_linear_x", value=str(.08 if nonzero else 0.)), KeyValue(key="applied_angular_z", value="0.0")]; d.status = [s]; self.diag.publish(d)

    def tick(self):
        if self.count_subscribers("/mission/state") < 1 or self.count_subscribers("/navigate_to_pose/_action/status") < 1:
            return
        self.step += 1
        if self.step < 20: self.state(MissionState.NAVIGATING); self.status_one(GoalStatus.STATUS_ACCEPTED); self.physical(True)
        elif self.step == 20: self.state(MissionState.CANCELLING, "FEEDBACK_STALE", "health failure termination requested"); self.status_one(GoalStatus.STATUS_CANCELING); self.physical(False)
        elif self.step == 21: self.state(MissionState.CANCELLING, "HEALTH_CANCEL_ACK_ACCEPTED", "active goal cancellation acknowledged; waiting for CANCELED result"); self.physical(False)
        elif self.inverted and self.step == 22: self.state(MissionState.FAILED, "FEEDBACK_STALE", "active goal cancelled after health failure", active=False); self.physical(False)
        elif self.inverted and self.step == 23: self.status_one(GoalStatus.STATUS_CANCELED); self.physical(False)
        elif not self.inverted and self.step == 22: self.status_one(GoalStatus.STATUS_CANCELED); self.physical(False)
        elif not self.inverted and self.step == 23: self.state(MissionState.FAILED, "FEEDBACK_STALE", "active goal cancelled after health failure", active=False); self.physical(False)
        elif self.step > 28:
            self.done = True


def main():
    p = argparse.ArgumentParser(); p.add_argument("--inverted", action="store_true"); args = p.parse_args()
    rclpy.init(); node = Driver(args.inverted)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    finally: node.destroy_node();
    if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
