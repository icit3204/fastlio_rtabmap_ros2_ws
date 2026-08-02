import threading
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from parking_robot_interfaces.msg import MissionState, RouteMission
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from parking_robot_mission_manager.mission_manager_node import MissionManagerNode


def pose(x):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.orientation.w = 1.0
    return msg


def make_mission(count=2, *, topology="v1", directions=None):
    msg = RouteMission()
    msg.header.frame_id = "map"
    msg.mission_id = uuid.uuid4().hex
    msg.route_id = "route-a"
    msg.topology_version = topology
    msg.node_ids = [f"node-{i}" for i in range(count)]
    msg.poses = [pose(float(i)) for i in range(count)]
    msg.edge_ids = [f"edge-{i}" for i in range(max(count - 1, 0))]
    msg.edge_directions = directions if directions is not None else [1 for _ in msg.edge_ids]
    return msg


class Recorder(Node):
    def __init__(self):
        super().__init__("mission_state_recorder")
        self.states = []
        self.statuses = []
        self.pub = self.create_publisher(RouteMission, "/mission/route", 10)
        self.sub = self.create_subscription(MissionState, "/mission/state", self.states.append, 10)
        self.start = self.create_client(Trigger, "/mission/start")
        self.cancel = self.create_client(Trigger, "/mission/cancel")
        self.pause = self.create_client(SetBool, "/mission/pause")


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
        if self.outcomes and self.outcomes[0] == "cancel_timeout":
            self.outcomes.pop(0)
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        self.goal_uuids.append(bytes(goal_handle.goal_id.uuid).hex())
        outcome = self.outcomes.pop(0) if self.outcomes else "succeeded"
        if outcome == "delayed":
            start = time.monotonic()
            while time.monotonic() - start < 0.3:
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


def call_trigger(client):
    assert client.wait_for_service(timeout_sec=2.0)
    future = client.call_async(Trigger.Request())
    assert spin_until(lambda: future.done())
    return future.result()


def call_pause(client, value):
    assert client.wait_for_service(timeout_sec=2.0)
    req = SetBool.Request()
    req.data = value
    future = client.call_async(req)
    assert spin_until(lambda: future.done())
    return future.result()


def terminal_state(msg):
    return msg.state in (
        MissionState.CANCELLED,
        MissionState.SUCCEEDED,
        MissionState.BLOCKED,
        MissionState.FAILED,
        MissionState.HELP_REQUIRED,
    )


def run_nodes(outcomes, mission, *, after_receive=None, wait_predicate=None):
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
        assert spin_until(lambda: recorder.states and recorder.states[-1].state == MissionState.RECEIVED)
        if after_receive is None:
            assert call_trigger(recorder.start).success
        else:
            after_receive(recorder)
        predicate = wait_predicate or (lambda: recorder.states and terminal_state(recorder.states[-1]))
        assert spin_until(predicate)
        time.sleep(0.4)
        return list(recorder.states), list(server.goal_uuids), server.cancel_count
    finally:
        executor.shutdown()
        for node in (manager, server, recorder):
            node.destroy_node()
        thread.join(timeout=1.0)


def test_receive_then_explicit_start_two_waypoint_success(ros_context):
    states, goal_ids, cancel_count = run_nodes(["succeeded", "succeeded"], make_mission(2))
    state_values = [msg.state for msg in states]
    assert state_values.index(MissionState.RECEIVED) < state_values.index(MissionState.VALIDATING)
    assert MissionState.PLANNING in state_values
    assert MissionState.NAVIGATING in state_values
    assert states[-1].state == MissionState.SUCCEEDED
    assert len(goal_ids) == 2
    assert len(set(goal_ids)) == 2
    assert cancel_count == 0


def test_topology_mismatch_rejection_with_zero_goals(ros_context):
    def start(recorder):
        result = call_trigger(recorder.start)
        assert not result.success

    states, goal_ids, _ = run_nodes(["succeeded"], make_mission(1, topology="wrong"), after_receive=start)
    assert states[-1].state == MissionState.FAILED
    assert states[-1].reason_code == "TOPOLOGY_VERSION_MISMATCH"
    assert goal_ids == []


def test_mixed_edge_direction_valid_mission(ros_context):
    states, goal_ids, _ = run_nodes(["succeeded", "succeeded", "succeeded"], make_mission(3, directions=[-1, 0]))
    assert states[-1].state == MissionState.SUCCEEDED
    assert len(goal_ids) == 3


def test_middle_waypoint_abort_with_no_later_dispatch(ros_context):
    states, goal_ids, _ = run_nodes(["succeeded", "aborted", "succeeded"], make_mission(3))
    assert states[-1].state == MissionState.FAILED
    assert states[-1].current_waypoint_index == 1
    assert len(goal_ids) == 2


def test_cancel_acknowledgement_to_cancelled(ros_context):
    def start_cancel(recorder):
        assert call_trigger(recorder.start).success
        assert spin_until(lambda: recorder.states[-1].active_goal_uuid)
        assert call_trigger(recorder.cancel).success

    states, goal_ids, cancel_count = run_nodes(["delayed"], make_mission(2), after_receive=start_cancel)
    assert states[-1].state == MissionState.CANCELLED
    assert len(goal_ids) == 1
    assert cancel_count == 1


def test_cancel_timeout_to_failed(ros_context):
    def start_cancel(recorder):
        assert call_trigger(recorder.start).success
        assert spin_until(lambda: recorder.states[-1].active_goal_uuid)
        assert call_trigger(recorder.cancel).success

    states, goal_ids, cancel_count = run_nodes(["delayed", "cancel_timeout"], make_mission(2), after_receive=start_cancel)
    assert states[-1].state == MissionState.FAILED
    assert states[-1].reason_code == "CANCEL_ACK_TIMEOUT"
    assert len(goal_ids) == 1
    assert cancel_count == 1


def test_pause_ack_no_goal_while_paused_resume_same_waypoint_success(ros_context):
    def start_pause_resume(recorder):
        assert call_trigger(recorder.start).success
        assert spin_until(lambda: recorder.states[-1].active_goal_uuid)
        assert call_pause(recorder.pause, True).success
        assert spin_until(lambda: recorder.states[-1].state == MissionState.PAUSED)
        goal_count_while_paused = len([s for s in recorder.states if s.active_goal_uuid])
        time.sleep(0.05)
        assert len([s for s in recorder.states if s.active_goal_uuid]) == goal_count_while_paused
        assert call_pause(recorder.pause, False).success

    states, goal_ids, cancel_count = run_nodes(["delayed", "succeeded", "succeeded"], make_mission(2), after_receive=start_pause_resume)
    assert states[-1].state == MissionState.SUCCEEDED
    assert len(goal_ids) == 3
    assert cancel_count == 1


def test_pause_timeout_to_failed(ros_context):
    def start_pause(recorder):
        assert call_trigger(recorder.start).success
        assert spin_until(lambda: recorder.states[-1].active_goal_uuid)
        assert call_pause(recorder.pause, True).success

    states, goal_ids, cancel_count = run_nodes(["delayed", "cancel_timeout"], make_mission(2), after_receive=start_pause)
    assert states[-1].state == MissionState.FAILED
    assert states[-1].reason_code == "PAUSE_CANCEL_ACK_TIMEOUT"
    assert len(goal_ids) == 1
    assert cancel_count == 1
