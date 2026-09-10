import shutil
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.mission_bridge import MissionBridgeNode, prepare_route_spec_from_topology, route_spec_to_msg, verify_route_topology_current
from core.topology_identity import EdgeRecord, build_manifest, load_topology, read_manifest, render_edges_v2, write_manifest_atomic

REPO = Path(__file__).resolve().parents[2]
TOPOLOGY = REPO / 'plan_nav' / 'underGround_split1'


def _write_nodes(path, rows):
    lines = ['# node_id, label, annotation, x, y, z, yaw_deg, timestamp_unix, traj_idx']
    lines.extend(', '.join(str(item) for item in row) for row in rows)
    (path / 'nodes.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _fixture_topology(tmp_path, *, bi=False, ambiguous=False):
    work = tmp_path / 'topology'
    work.mkdir(parents=True)
    _write_nodes(work, [[1, 'WP-01', '', 0, 0, 0, 0, 1, -1], [2, 'WP-02', '', 1, 0, 0, 0, 2, -1], [3, 'WP-03', '', 2, 0, 0, 0, 3, -1]])
    edges = [EdgeRecord('edge-000001', 1, 2, 1.0, 'bi' if bi else 'uni', '')]
    edges.append(EdgeRecord('edge-000002', 1 if ambiguous else 2, 2 if ambiguous else 3, 1.0, 'uni', ''))
    (work / 'edges.txt').write_text(render_edges_v2(edges), encoding='utf-8')
    nodes, parsed_edges, _ = load_topology(work)
    write_manifest_atomic(work, build_manifest(work, nodes, parsed_edges))
    return work


def _route(start=1, end=3):
    result = prepare_route_spec_from_topology(work_dir=TOPOLOGY, start_node_id=start, end_node_id=end)
    assert result.valid
    return result.route


def test_prepare_route_spec_uses_authoritative_topology_identity():
    result = prepare_route_spec_from_topology(work_dir=TOPOLOGY, start_node_id=1, end_node_id=3, mission_id='mission-a')
    assert result.valid
    assert result.path_node_ids == [1, 2, 3]
    assert result.route.mission_id == 'mission-a'
    assert result.route.header_frame_id == 'map'
    assert result.route.topology_version == read_manifest(TOPOLOGY)['topology_version']
    assert result.route.node_ids == ['1', '2', '3']
    assert result.route.edge_directions == [1, 1]


def test_no_route_and_topology_changed_after_planning(tmp_path):
    no_route = prepare_route_spec_from_topology(work_dir=TOPOLOGY, start_node_id=1, end_node_id=20)
    assert not no_route.valid and no_route.reason_code == 'NO_TOPOLOGICAL_ROUTE'
    copied = tmp_path / 'copy'
    shutil.copytree(TOPOLOGY, copied)
    planned = prepare_route_spec_from_topology(work_dir=copied, start_node_id=1, end_node_id=2)
    manifest = read_manifest(copied)
    manifest['topology_version'] = 'sha256:' + '0' * 64
    write_manifest_atomic(copied, manifest)
    assert verify_route_topology_current(copied, planned.route)[1] == 'TOPOLOGY_CHANGED_AFTER_PLANNING'


def test_reverse_bi_edge_and_ambiguous_route_contract(tmp_path):
    reverse = prepare_route_spec_from_topology(work_dir=_fixture_topology(tmp_path / 'bi', bi=True), start_node_id=2, end_node_id=1)
    assert reverse.valid and reverse.route.edge_directions == [-1]
    ambiguous = prepare_route_spec_from_topology(work_dir=_fixture_topology(tmp_path / 'ambiguous', ambiguous=True), start_node_id=1, end_node_id=2)
    assert not ambiguous.valid and ambiguous.reason_code == 'AMBIGUOUS_ROUTE_EDGE'


def test_bridge_is_route_publication_only():
    source = (REPO / 'plan_nav' / 'core' / 'mission_bridge.py').read_text(encoding='utf-8')
    assert '"/mission/route"' in source
    for forbidden in ('/mission/start', '/mission/pause', '/mission/cancel', '/mission/state', 'ActionClient', 'UdpSender', 'socket', '/cmd_vel', '/vehicle_cmd_safe'):
        assert forbidden not in source


@pytest.fixture
def ros_context():
    import rclpy
    rclpy.init(args=[])
    yield rclpy
    rclpy.shutdown()


def test_route_spec_to_msg_exact_arrays(ros_context):
    node = ros_context.create_node('route_spec_to_msg_test')
    try:
        route = _route()
        msg = route_spec_to_msg(route, node)
        assert msg.header.frame_id == 'map'
        assert msg.mission_id == route.mission_id
        assert msg.route_id == route.route_id
        assert msg.topology_version == route.topology_version
        assert msg.node_ids == route.node_ids
        assert msg.edge_ids == route.edge_ids
        assert list(msg.edge_directions) == route.edge_directions
        assert len(msg.poses) == len(route.poses)
    finally:
        node.destroy_node()


def test_explicit_route_publish_does_not_create_runtime_control(ros_context):
    from parking_robot_interfaces.msg import RouteMission
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    bridge = MissionBridgeNode(node_name='plan_nav_route_publish_test')
    observer = ros_context.create_node('plan_nav_route_observer_test')
    received = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    observer.create_subscription(RouteMission, '/mission/route', received.append, qos)
    try:
        route = _route()
        bridge.publish_route(route)
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            ros_context.spin_once(observer, timeout_sec=0.05)
        assert received and received[-1].mission_id == route.mission_id
        assert not hasattr(bridge, '_start_client')
        assert not hasattr(bridge, '_pause_client')
        assert not hasattr(bridge, '_cancel_client')
    finally:
        observer.destroy_node()
        bridge.destroy()
