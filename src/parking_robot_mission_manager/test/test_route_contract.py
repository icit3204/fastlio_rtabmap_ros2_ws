import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from parking_robot_interfaces.msg import RouteMission

from parking_robot_mission_manager.route_contract import (
    DEFAULT_MIN_WAYPOINT_SEPARATION_M,
    configured_min_waypoint_separation_m,
    validate_route_mission,
)


EXPECTED_TOPOLOGY = "topology-v1"


def pose(x=0.0, y=0.0, frame_id="map", qw=1.0):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = qw
    return msg


def mission(count=1, *, directions=None):
    msg = RouteMission()
    msg.header.frame_id = "map"
    msg.mission_id = "mission-1"
    msg.route_id = "route-a"
    msg.topology_version = EXPECTED_TOPOLOGY
    msg.node_ids = [f"node-{i}" for i in range(count)]
    msg.poses = [pose(float(i)) for i in range(count)]
    msg.edge_ids = [f"edge-{i}" for i in range(max(count - 1, 0))]
    msg.edge_directions = directions if directions is not None else [1 for _ in msg.edge_ids]
    return msg


def validate(msg, **kwargs):
    return validate_route_mission(msg, expected_topology_version=EXPECTED_TOPOLOGY, **kwargs)


def assert_invalid(msg, reason):
    result = validate(msg)
    assert not result.valid
    assert result.reason_code == reason


def test_valid_one_pose_mission_and_two_edge_forward_route():
    result = validate(mission(1))
    assert result.valid
    assert result.mission.node_ids == ("node-0",)
    assert result.mission.edge_ids == ()

    result = validate(mission(3, directions=[1, 1]))
    assert result.valid
    assert result.mission.edge_directions == (1, 1)


def test_mixed_edge_directions_are_valid():
    result = validate(mission(4, directions=[-1, 0, 1]))
    assert result.valid
    assert result.mission.edge_directions == (-1, 0, 1)


def test_identity_and_topology_rejections():
    msg = mission()
    msg.mission_id = ""
    assert_invalid(msg, "EMPTY_MISSION_ID")
    msg = mission()
    msg.route_id = ""
    assert_invalid(msg, "EMPTY_ROUTE_ID")
    msg = mission()
    msg.topology_version = ""
    assert_invalid(msg, "EMPTY_TOPOLOGY_VERSION")
    msg = mission()
    msg.topology_version = "other"
    assert_invalid(msg, "TOPOLOGY_VERSION_MISMATCH")


def test_array_length_rejections():
    msg = mission()
    msg.poses = []
    msg.node_ids = []
    assert_invalid(msg, "ZERO_POSES")
    msg = mission(2)
    msg.node_ids = ["only-one"]
    assert_invalid(msg, "NODE_POSE_LENGTH_MISMATCH")
    msg = mission(3)
    msg.edge_ids = ["one-edge"]
    msg.edge_directions = [1]
    assert_invalid(msg, "EDGE_COUNT_MISMATCH")
    msg = mission(3)
    msg.edge_directions = [1]
    assert_invalid(msg, "DIRECTION_COUNT_MISMATCH")


def test_direction_and_id_rejections():
    msg = mission(2)
    msg.edge_directions = [-2]
    assert_invalid(msg, "INVALID_EDGE_DIRECTION")
    msg = mission(2)
    msg.edge_directions = [2]
    assert_invalid(msg, "INVALID_EDGE_DIRECTION")
    msg = mission(1)
    msg.node_ids = [""]
    assert_invalid(msg, "EMPTY_NODE_ID")
    msg = mission(2)
    msg.edge_ids = [""]
    assert_invalid(msg, "EMPTY_EDGE_ID")


def test_frame_and_pose_rejections():
    msg = mission()
    msg.header.frame_id = "odom"
    assert_invalid(msg, "MISSION_FRAME_NOT_MAP")
    msg = mission()
    msg.poses[0].header.frame_id = "odom"
    assert_invalid(msg, "POSE_FRAME_NOT_MAP")
    msg = mission()
    msg.poses[0].pose.position.x = math.nan
    assert_invalid(msg, "NONFINITE_POSE")
    msg = mission()
    msg.poses[0].pose.orientation.w = 0.0
    assert_invalid(msg, "ZERO_QUATERNION")
    msg = mission()
    msg.poses[0].pose.orientation.w = 2.0
    assert_invalid(msg, "QUATERNION_NOT_NORMALIZED")


def test_consecutive_pose_spacing_policy_and_boundary():
    assert DEFAULT_MIN_WAYPOINT_SEPARATION_M == 0.55
    assert configured_min_waypoint_separation_m(0.25, 0.05) == 0.55
    msg = mission(2)
    msg.poses[1].pose.position.x = 0.549
    assert_invalid(msg, "CONSECUTIVE_WAYPOINTS_TOO_CLOSE")
    msg = mission(2)
    msg.poses[1].pose.position.x = 0.55
    assert validate(msg).valid
    msg = mission(2)
    msg.poses[1].pose.position.x = 0.10
    result = validate(msg, min_waypoint_separation_m=0.05)
    assert result.valid


def test_no_geometry_derived_identity_or_path_authority():
    path = Path()
    path.header.frame_id = "map"
    path.poses = [pose(0.0), pose(1.0)]
    result = validate(path)
    assert not result.valid
    assert result.reason_code == "EMPTY_MISSION_ID"

    msg = mission(2)
    msg.edge_ids = [""]
    assert_invalid(msg, "EMPTY_EDGE_ID")
