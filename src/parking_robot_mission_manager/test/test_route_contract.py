import math

from geometry_msgs.msg import PoseStamped
from parking_robot_interfaces.msg import RouteMission, RouteWaypoint

from parking_robot_mission_manager.route_contract import validate_route_mission


def make_waypoint(waypoint_id="wp1", x=1.0, y=0.0, frame_id="map", qw=1.0):
    waypoint = RouteWaypoint()
    waypoint.waypoint_id = waypoint_id
    waypoint.pose = PoseStamped()
    waypoint.pose.header.frame_id = frame_id
    waypoint.pose.pose.position.x = x
    waypoint.pose.pose.position.y = y
    waypoint.pose.pose.orientation.w = qw
    return waypoint


def make_mission(*waypoints):
    mission = RouteMission()
    mission.header.frame_id = "map"
    mission.mission_id = "mission-1"
    mission.route_id = "route-a"
    mission.route_version = "v1"
    mission.direction_id = "forward"
    mission.waypoints = list(waypoints or [make_waypoint()])
    return mission


def assert_invalid(mission, reason):
    result = validate_route_mission(mission)
    assert not result.valid
    assert result.reason_code == reason


def test_valid_route_contract_normalizes_typed_mission():
    result = validate_route_mission(make_mission(make_waypoint("a", 1.0), make_waypoint("b", 2.0)))
    assert result.valid
    assert result.mission.route_id == "route-a"
    assert [wp.waypoint_id for wp in result.mission.waypoints] == ["a", "b"]


def test_rejects_required_identity_and_frame_fields():
    mission = make_mission()
    mission.mission_id = ""
    assert_invalid(mission, "EMPTY_MISSION_ID")
    mission = make_mission()
    mission.route_id = ""
    assert_invalid(mission, "EMPTY_ROUTE_ID")
    mission = make_mission()
    mission.route_version = ""
    assert_invalid(mission, "EMPTY_ROUTE_VERSION")
    mission = make_mission()
    mission.direction_id = ""
    assert_invalid(mission, "EMPTY_DIRECTION_ID")
    mission = make_mission()
    mission.header.frame_id = "odom"
    assert_invalid(mission, "MISSION_FRAME_NOT_MAP")


def test_rejects_waypoint_contract_violations():
    mission = make_mission()
    mission.waypoints = []
    assert_invalid(mission, "ZERO_WAYPOINTS")
    mission = make_mission(make_waypoint(""))
    assert_invalid(mission, "EMPTY_WAYPOINT_ID")
    mission = make_mission(make_waypoint("a", 1.0), make_waypoint("a", 2.0))
    assert_invalid(mission, "DUPLICATE_WAYPOINT_ID")
    mission = make_mission(make_waypoint("a", 1.0, frame_id="odom"))
    assert_invalid(mission, "WAYPOINT_FRAME_NOT_MAP")


def test_rejects_nonfinite_and_invalid_quaternion():
    mission = make_mission(make_waypoint("a", math.nan))
    assert_invalid(mission, "NONFINITE_POSE")
    mission = make_mission(make_waypoint("a", 1.0, qw=0.0))
    assert_invalid(mission, "ZERO_QUATERNION")
    mission = make_mission(make_waypoint("a", 1.0, qw=2.0))
    assert_invalid(mission, "QUATERNION_NOT_NORMALIZED")


def test_rejects_effectively_identical_consecutive_waypoints_without_deleting_them():
    mission = make_mission(make_waypoint("a", 1.0), make_waypoint("b", 1.01))
    result = validate_route_mission(mission, min_waypoint_separation_m=0.05)
    assert not result.valid
    assert result.reason_code == "CONSECUTIVE_WAYPOINTS_TOO_CLOSE"
    assert len(mission.waypoints) == 2


def test_geometry_only_path_is_not_a_valid_authoritative_mission():
    from nav_msgs.msg import Path

    path = Path()
    path.header.frame_id = "map"
    result = validate_route_mission(path)
    assert not result.valid
    assert result.reason_code in {"EMPTY_MISSION_ID", "ZERO_WAYPOINTS"}
