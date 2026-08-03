import os
import shutil
import sys
import threading
import time
import uuid
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.mission_bridge import (
    AuthorityMode,
    MissionBridgeNode,
    prepare_route_spec_from_topology,
    route_spec_to_msg,
    verify_route_topology_current,
)
from core.topology_identity import (
    EdgeRecord,
    build_manifest,
    build_sparse_route_spec,
    load_topology,
    read_manifest,
    render_edges_v2,
    write_manifest_atomic,
)


REPO = Path(__file__).resolve().parents[2]
TOPOLOGY = REPO / "plan_nav" / "underGround_split1"


def _write_nodes(path: Path, rows):
    lines = ["# node_id, label, annotation, x, y, z, yaw_deg, timestamp_unix, traj_idx"]
    for row in rows:
        lines.append(", ".join(str(item) for item in row))
    (path / "nodes.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_edges(path: Path, edges):
    (path / "edges.txt").write_text(render_edges_v2(edges), encoding="utf-8")


def _fixture_topology(tmp_path: Path, *, bi=False, ambiguous=False) -> Path:
    work = tmp_path / "topology"
    work.mkdir(parents=True)
    _write_nodes(
        work,
        [
            [1, "WP-01", "", 0.0, 0.0, 0.0, 0.0, 1.0, -1],
            [2, "WP-02", "", 1.0, 0.0, 0.0, 0.0, 2.0, -1],
            [3, "WP-03", "", 2.0, 0.0, 0.0, 0.0, 3.0, -1],
        ],
    )
    edges = [EdgeRecord("edge-000001", 1, 2, 1.0, "bi" if bi else "uni", "")]
    if ambiguous:
        edges.append(EdgeRecord("edge-000002", 1, 2, 1.0, "uni", ""))
    else:
        edges.append(EdgeRecord("edge-000002", 2, 3, 1.0, "uni", ""))
    _write_edges(work, edges)
    nodes, parsed_edges, _ = load_topology(work)
    write_manifest_atomic(work, build_manifest(work, nodes, parsed_edges))
    return work


def test_prepare_route_spec_uses_authoritative_topology_identity():
    result = prepare_route_spec_from_topology(work_dir=TOPOLOGY, start_node_id=1, end_node_id=3, mission_id="mission-a")
    assert result.valid
    assert result.path_node_ids == [1, 2, 3]
    assert result.route.mission_id == "mission-a"
    assert result.route.header_frame_id == "map"
    assert result.route.topology_version == read_manifest(TOPOLOGY)["topology_version"]
    assert result.route.node_ids == ["1", "2", "3"]
    assert len(result.route.edge_ids) == 2
    assert result.route.edge_directions == [1, 1]
    again = prepare_route_spec_from_topology(work_dir=TOPOLOGY, start_node_id=1, end_node_id=3, mission_id="mission-b")
    assert again.route.route_id == result.route.route_id
    assert again.route.mission_id != result.route.mission_id


def test_no_route_and_topology_changed_after_planning(tmp_path):
    no_route = prepare_route_spec_from_topology(work_dir=TOPOLOGY, start_node_id=1, end_node_id=20)
    assert not no_route.valid
    assert no_route.reason_code == "NO_TOPOLOGICAL_ROUTE"

    copied = tmp_path / "topology_copy"
    shutil.copytree(TOPOLOGY, copied)
    planned = prepare_route_spec_from_topology(work_dir=copied, start_node_id=1, end_node_id=2)
    assert planned.valid
    manifest = read_manifest(copied)
    manifest["topology_version"] = "sha256:" + "0" * 64
    write_manifest_atomic(copied, manifest)
    ok, reason, _detail = verify_route_topology_current(copied, planned.route)
    assert not ok
    assert reason == "TOPOLOGY_CHANGED_AFTER_PLANNING"


def test_reverse_bi_edge_and_ambiguous_route_contract(tmp_path):
    bi_topology = _fixture_topology(tmp_path / "bi", bi=True)
    reverse = prepare_route_spec_from_topology(work_dir=bi_topology, start_node_id=2, end_node_id=1, mission_id="mission-rev")
    assert reverse.valid
    assert reverse.route.edge_ids == ["edge-000001"]
    assert reverse.route.edge_directions == [-1]

    ambiguous = _fixture_topology(tmp_path / "ambiguous", ambiguous=True)
    result = prepare_route_spec_from_topology(work_dir=ambiguous, start_node_id=1, end_node_id=2)
    assert not result.valid
    assert result.reason_code == "AMBIGUOUS_ROUTE_EDGE"


def test_mission_nav2_mode_isolation_static_contract():
    bridge_source = (REPO / "plan_nav" / "core" / "mission_bridge.py").read_text(encoding="utf-8")
    assert "UdpSender" not in bridge_source
    assert "socket" not in bridge_source
    assert "/cmd_vel" not in bridge_source
    assert "/vehicle_cmd_safe" not in bridge_source
    assert AuthorityMode.LEGACY.value == "legacy"
    assert AuthorityMode.MISSION_NAV2.value == "mission_nav2"


@pytest.fixture
def ros_context():
    import rclpy

    rclpy.init(args=[])
    yield
    rclpy.shutdown()


def spin_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class StateRecorder:
    def __init__(self):
        self.states = []
        self.service_results = []

    def on_state(self, msg):
        self.states.append(msg)

    def on_service(self, name, success, reason, message):
        self.service_results.append((name, success, reason, message))


class FakeActionServer:
    def __init__(self, outcomes):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.node import Node

        self.Node = Node
        self.GoalResponse = GoalResponse
        self.CancelResponse = CancelResponse
        self.NavigateToPose = NavigateToPose
        self.node = Node(f"plan_nav_fake_nav2_{uuid.uuid4().hex[:8]}")
        self.outcomes = list(outcomes)
        self.goal_uuids = []
        self.cancel_count = 0
        self.server = ActionServer(
            self.node,
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
            return self.GoalResponse.REJECT
        return self.GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        del goal_handle
        self.cancel_count += 1
        return self.CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        outcome = self.outcomes.pop(0) if self.outcomes else "succeeded"
        self.goal_uuids.append(bytes(goal_handle.goal_id.uuid).hex())
        if outcome == "delayed":
            while not goal_handle.is_cancel_requested:
                time.sleep(0.01)
            goal_handle.canceled()
        elif outcome == "aborted":
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return self.NavigateToPose.Result()

    def destroy(self):
        self.node.destroy_node()


def _route(start=1, end=3):
    result = prepare_route_spec_from_topology(work_dir=TOPOLOGY, start_node_id=start, end_node_id=end)
    assert result.valid
    return result.route


def _make_manager(route):
    from rclpy.parameter import Parameter
    from parking_robot_mission_manager.mission_manager_node import MissionManagerNode

    return MissionManagerNode(
        parameter_overrides=[
            Parameter("expected_topology_version", Parameter.Type.STRING, route.topology_version),
        ]
    )


def _run_integration(route, outcomes, *, action=None, second_route=None):
    from rclpy.executors import MultiThreadedExecutor
    from parking_robot_interfaces.msg import MissionState

    recorder = StateRecorder()
    manager = _make_manager(route)
    server = FakeActionServer(outcomes)
    bridge = MissionBridgeNode(state_callback=recorder.on_state, service_callback=recorder.on_service)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (manager, server.node, bridge.node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert spin_until(lambda: manager._action_client.server_is_ready())
        published = bridge.publish_route(route)
        assert published.mission_id == route.mission_id
        assert published.route_id == route.route_id
        assert published.topology_version == route.topology_version
        assert published.node_ids == route.node_ids
        assert published.edge_ids == route.edge_ids
        assert list(published.edge_directions) == route.edge_directions
        assert spin_until(lambda: recorder.states and recorder.states[-1].state == MissionState.RECEIVED)
        if action == "no_start":
            time.sleep(0.2)
            return recorder, server, bridge
        bridge.request_start()
        assert spin_until(lambda: any(item[0] == "start" for item in recorder.service_results))
        if action == "cancel":
            assert spin_until(lambda: server.goal_uuids)
            bridge.request_cancel()
        elif action == "pause_resume":
            assert spin_until(lambda: server.goal_uuids)
            bridge.request_pause(True)
            assert spin_until(lambda: recorder.states and recorder.states[-1].state == MissionState.PAUSED)
            before_resume = len(server.goal_uuids)
            bridge.request_pause(False)
            assert spin_until(lambda: len(server.goal_uuids) > before_resume)
        assert spin_until(lambda: recorder.states and recorder.states[-1].state in {
            MissionState.CANCELLED,
            MissionState.SUCCEEDED,
            MissionState.FAILED,
        })
        if second_route is not None:
            bridge.publish_route(second_route)
            assert spin_until(lambda: recorder.states and recorder.states[-1].mission_id == second_route.mission_id)
            bridge.request_start()
            assert spin_until(lambda: recorder.states and recorder.states[-1].state == MissionState.SUCCEEDED)
        return recorder, server, bridge
    finally:
        executor.shutdown()
        for node in (manager, server.node, bridge.node):
            node.destroy_node()
        thread.join(timeout=1.0)


def test_ros_publish_route_then_explicit_start_success_and_second_mission(ros_context):
    from parking_robot_interfaces.msg import MissionState

    route = _route()
    no_start_recorder, no_start_server, _bridge = _run_integration(route, ["succeeded", "succeeded"], action="no_start")
    assert no_start_recorder.states[-1].state == MissionState.RECEIVED
    assert no_start_server.goal_uuids == []

    first = _route()
    second = _route()
    recorder, server, _ = _run_integration(first, ["succeeded", "succeeded", "succeeded", "succeeded"], second_route=second)
    assert first.mission_id != second.mission_id
    assert first.route_id == second.route_id
    assert first.topology_version == second.topology_version
    assert first.node_ids == second.node_ids
    assert recorder.states[-1].state == MissionState.SUCCEEDED
    assert len(server.goal_uuids) == len(first.poses) + len(second.poses)
    assert len(set(server.goal_uuids)) == len(server.goal_uuids)


def test_ros_middle_waypoint_aborted_cancel_and_pause_resume(ros_context):
    from parking_robot_interfaces.msg import MissionState

    aborted, aborted_server, _ = _run_integration(_route(), ["succeeded", "aborted"])
    assert aborted.states[-1].state == MissionState.FAILED
    assert len(aborted_server.goal_uuids) == 2

    canceled, cancel_server, _ = _run_integration(_route(), ["delayed"], action="cancel")
    assert canceled.states[-1].state == MissionState.CANCELLED
    assert cancel_server.cancel_count == 1

    paused, pause_server, _ = _run_integration(_route(start=1, end=2), ["delayed", "succeeded"], action="pause_resume")
    state_values = [msg.state for msg in paused.states]
    assert MissionState.PAUSED in state_values
    assert paused.states[-1].state == MissionState.SUCCEEDED
    assert pause_server.cancel_count == 1


def test_route_spec_to_msg_exact_arrays(ros_context):
    import rclpy

    node = rclpy.create_node("route_spec_to_msg_test")
    try:
        route = _route()
        msg = route_spec_to_msg(route, node)
        assert msg.header.frame_id == "map"
        assert msg.mission_id == route.mission_id
        assert msg.route_id == route.route_id
        assert msg.topology_version == route.topology_version
        assert msg.node_ids == route.node_ids
        assert msg.edge_ids == route.edge_ids
        assert list(msg.edge_directions) == route.edge_directions
        assert len(msg.poses) == len(route.poses)
    finally:
        node.destroy_node()
