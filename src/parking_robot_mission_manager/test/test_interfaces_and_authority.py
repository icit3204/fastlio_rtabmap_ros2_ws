from pathlib import Path
import re

from parking_robot_interfaces.msg import MissionState


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent


def test_mission_state_constants_are_unique_and_typed():
    constants = [
        MissionState.IDLE,
        MissionState.VALIDATING,
        MissionState.READY,
        MissionState.RUNNING,
        MissionState.PAUSED,
        MissionState.CANCELING,
        MissionState.SUCCEEDED,
        MissionState.FAILED,
        MissionState.BLOCKED,
        MissionState.REJECTED,
    ]
    assert len(constants) == len(set(constants))
    assert all(isinstance(value, int) for value in constants)


def test_interface_dependencies_do_not_include_command_or_physical_control():
    package_xml = (SRC / "parking_robot_interfaces" / "package.xml").read_text()
    for forbidden in ("Twist", "cmd_vel", "wheelchair", "CAN", "UDP", "plan_nav"):
        assert forbidden not in package_xml


def test_mission_manager_has_no_velocity_or_physical_publishers():
    module_dir = SRC / "parking_robot_mission_manager" / "parking_robot_mission_manager"
    source = "\n".join(path.read_text() for path in module_dir.rglob("*.py"))
    forbidden_topics = [
        "/cmd_vel",
        "/cmd_vel_nav",
        "/cmd_vel_phase2_mock",
        "/wheelchair_control_command",
        "/wheel",
    ]
    for token in forbidden_topics:
        assert token not in source
    for forbidden_import in ("from geometry_msgs.msg import Twist", "TwistStamped", "Float32MultiArray"):
        assert forbidden_import not in source
    for forbidden_dependency in ("plan_nav_laser_avoidance", "laser_avoidance", "wheelchair_controller"):
        assert forbidden_dependency not in source
    assert not re.search(r"create_publisher\s*\([^)]*(Twist|Float32MultiArray)", source)


def test_message_files_define_required_fields_and_zero_based_index_comment():
    msg_dir = SRC / "parking_robot_interfaces" / "msg"
    assert "geometry_msgs/PoseStamped pose" in (msg_dir / "RouteWaypoint.msg").read_text()
    route = (msg_dir / "RouteMission.msg").read_text()
    for field in ("mission_id", "route_id", "route_version", "direction_id", "RouteWaypoint[] waypoints"):
        assert field in route
    state = (msg_dir / "MissionState.msg").read_text()
    assert "zero-based" in state
    for constant in ("IDLE", "VALIDATING", "READY", "RUNNING", "PAUSED", "CANCELING", "SUCCEEDED", "FAILED", "BLOCKED", "REJECTED"):
        assert constant in state
