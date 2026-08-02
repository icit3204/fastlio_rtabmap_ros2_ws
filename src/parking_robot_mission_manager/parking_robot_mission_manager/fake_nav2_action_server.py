"""Deterministic NavigateToPose action server for P3-A synthetic tests."""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class FakeNavigateToPoseServer(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager_fake_nav2_server")
        self.declare_parameter("outcomes", "succeeded")
        self.declare_parameter("result_delay_sec", 0.05)
        self._outcomes = [item.strip() for item in str(self.get_parameter("outcomes").value).split(",") if item.strip()]
        self.goal_count = 0
        self.cancel_count = 0
        self._server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

    def _next_outcome(self) -> str:
        if self.goal_count < len(self._outcomes):
            return self._outcomes[self.goal_count]
        return "succeeded"

    def _goal_cb(self, goal_request) -> GoalResponse:
        del goal_request
        if self._next_outcome() == "rejected":
            self.goal_count += 1
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.cancel_count += 1
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        outcome = self._next_outcome()
        self.goal_count += 1
        delay = float(self.get_parameter("result_delay_sec").value)
        start = time.monotonic()
        while time.monotonic() - start < delay:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        if outcome == "aborted":
            goal_handle.abort()
        elif outcome == "canceled":
            goal_handle.canceled()
        elif outcome == "delayed":
            while not goal_handle.is_cancel_requested:
                time.sleep(0.05)
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeNavigateToPoseServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
