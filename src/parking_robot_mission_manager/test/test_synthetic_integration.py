import threading
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from parking_robot_interfaces.msg import MissionState, RouteMission, RouteWaypoint
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from parking_robot_mission_manager.mission_manager_node import MissionManagerNode


def make_waypoint(waypoint_id, x):
    waypoint = RouteWaypoint()
    waypoint.waypoint_id = waypoint_id
    waypoint.pose = PoseStamped()
    waypoint.pose.header.frame_id = "map"
    waypoint.pose.pose.position.x = x
    waypoint.pose.pose.orientation.w = 1.0
    return waypoint


def make_mission(count=2):
    msg = RouteMission()
    msg.header.frame_id = "map"
    msg.mission_id = uuid.uuid4().hex
    msg.route_id = "route-a"
    msg.route_version = "v1"
    msg.direction_id = "forward"
    msg.waypoints = [make_waypoint(f"wp{i}", float(i)) for i in range(count)]
    return msg


class Recorder(Node):
    def __init__(self):
        super().__init__("mission_state_recorder")
        self.states = []
        self.pub = self.create_publisher(RouteMission, "/mission_manager/route_mission", 10)
        self.sub = self.create_subscription(MissionState, "/mission_manager/state", self.states.append, 10)
        self.pause = self.create_client(Trigger, "/mission_manager/pause")
        self.resume = self.create_client(Trigger, "/mission_manager/resume")
        self.cancel = self.create_client(Trigger, "/mission_manager/cancel")


class FakeActionServer(Node):
    def __init__(self, outcomes):
        super().__init__("fake_nav2_action_server_test")
        self.outcomes = list(outcomes)
        self.goal_uuids = []
        self.cancel_count = 0
        self.server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

    def goal_cb(self, goal_request):
        del goal_request
        if self.outcomes and self.outcomes[0] == "rejected":
            self.outcomes.pop(0)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        del goal_handle
        self.cancel_count += 1
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        self.goal_uuids.append(bytes(goal_handle.goal_id.uuid).hex())
        outcome = self.outcomes.pop(0) if self.outcomes else "succeeded"
        if outcome == "delayed":
            start = time.monotonic()
            while time.monotonic() - start < 3.0:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.01)
            goal_handle.succeed()
        elif outcome == "aborted":
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def run_nodes(outcomes, mission, *, service_call=None, wait_predicate=None):
    manager = MissionManagerNode()
    server = FakeActionServer(outcomes)
    recorder = Recorder()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (manager, server, recorder):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert spin_until(lambda: manager._action_client.server_is_ready())
        recorder.pub.publish(mission)
        if service_call is not None:
            assert service_call(recorder)
        predicate = wait_predicate or (
            lambda: recorder.states
            and recorder.states[-1].state
            in (MissionState.SUCCEEDED, MissionState.FAILED, MissionState.REJECTED, MissionState.IDLE)
        )
        assert spin_until(predicate)
        return list(recorder.states), list(server.goal_uuids), server.cancel_count
    finally:
        executor.shutdown()
        for node in (manager, server, recorder):
            node.destroy_node()
        thread.join(timeout=1.0)


def call_service(client):
    assert client.wait_for_service(timeout_sec=2.0)
    future = client.call_async(Trigger.Request())
    assert spin_until(lambda: future.done())
    return future.result().success


def test_two_waypoint_success_synthetic(ros_context):
    states, goal_ids, cancel_count = run_nodes(["succeeded", "succeeded"], make_mission(2))
    assert [msg.state for msg in states][-1] == MissionState.SUCCEEDED
    assert len(goal_ids) == 2
    assert len(set(goal_ids)) == 2
    assert states[-1].progress == 1.0
    assert states[-1].completed_waypoint_count == 2
    assert cancel_count == 0


def test_middle_waypoint_failure_never_sends_third(ros_context):
    states, goal_ids, _ = run_nodes(["succeeded", "aborted", "succeeded"], make_mission(3))
    assert states[-1].state == MissionState.FAILED
    assert states[-1].current_waypoint_index == 1
    assert len(goal_ids) == 2


def test_cancel_during_active_waypoint_returns_idle(ros_context):
    def service(recorder):
        assert spin_until(lambda: len(recorder.states) and recorder.states[-1].state == MissionState.RUNNING)
        return call_service(recorder.cancel)

    states, goal_ids, cancel_count = run_nodes(
        ["delayed"],
        make_mission(2),
        service_call=service,
        wait_predicate=lambda: True,
    )
    assert goal_ids[:1]
    assert cancel_count == 1
    assert any(msg.state == MissionState.CANCELING for msg in states)


def test_ambiguous_identity_rejected_without_goal(ros_context):
    msg = make_mission(1)
    msg.direction_id = ""
    states, goal_ids, _ = run_nodes(["succeeded"], msg)
    assert states[-1].state == MissionState.REJECTED
    assert states[-1].reason_code == "EMPTY_DIRECTION_ID"
    assert goal_ids == []


def test_pause_and_resume_resends_current_waypoint(ros_context):
    def service(recorder):
        assert spin_until(lambda: len(recorder.states) and recorder.states[-1].active_goal_uuid)
        assert call_service(recorder.pause)
        assert spin_until(lambda: recorder.states[-1].state == MissionState.PAUSED)
        assert call_service(recorder.resume)
        return True

    states, goal_ids, cancel_count = run_nodes(["delayed", "succeeded", "succeeded"], make_mission(2), service_call=service)
    assert states[-1].state == MissionState.SUCCEEDED
    assert cancel_count == 1
    assert len(goal_ids) == 3
