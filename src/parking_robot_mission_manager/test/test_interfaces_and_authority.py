from pathlib import Path
import re

from parking_robot_interfaces.msg import MissionState


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent


def test_mission_state_constants_are_unique_and_authority_aligned():
    constants = [
        MissionState.IDLE,
        MissionState.RECEIVED,
        MissionState.VALIDATING,
        MissionState.PLANNING,
        MissionState.NAVIGATING,
        MissionState.PAUSED,
        MissionState.CANCELLING,
        MissionState.CANCELLED,
        MissionState.SUCCEEDED,
        MissionState.TEMPORARILY_BLOCKED,
        MissionState.BLOCKED,
        MissionState.FAILED,
        MissionState.HELP_REQUIRED,
    ]
    assert len(constants) == len(set(constants))
    assert all(isinstance(value, int) for value in constants)
    for removed in ("READY", "RUNNING", "REJECTED"):
        assert not hasattr(MissionState, removed)


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


def test_message_files_define_required_authority_fields():
    msg_dir = SRC / "parking_robot_interfaces" / "msg"
    assert not (msg_dir / "RouteWaypoint.msg").exists()
    route = (msg_dir / "RouteMission.msg").read_text()
    for field in (
        "std_msgs/Header header",
        "string mission_id",
        "string route_id",
        "string topology_version",
        "string[] node_ids",
        "string[] edge_ids",
        "int8[] edge_directions",
        "geometry_msgs/PoseStamped[] poses",
    ):
        assert field in route
    for removed in ("route_version", "direction_id", "RouteWaypoint[]"):
        assert removed not in route
    state = (msg_dir / "MissionState.msg").read_text()
    assert "zero-based" in state
    for constant in (
        "IDLE",
        "RECEIVED",
        "VALIDATING",
        "PLANNING",
        "NAVIGATING",
        "PAUSED",
        "CANCELLING",
        "CANCELLED",
        "SUCCEEDED",
        "TEMPORARILY_BLOCKED",
        "BLOCKED",
        "FAILED",
        "HELP_REQUIRED",
    ):
        assert constant in state


def test_authoritative_ros_api_names_and_no_legacy_aliases():
    source = (SRC / "parking_robot_mission_manager" / "parking_robot_mission_manager" / "mission_manager_node.py").read_text()
    for topic in ("/mission/route", "/mission/state", "/mission/start", "/mission/cancel", "/mission/pause", "/mission/status", "/mission/block_reason"):
        assert topic in source
    for legacy in (
        "/mission_manager/route_mission",
        "/mission_manager/state",
        "/mission_manager/pause",
        "/mission_manager/resume",
        "/mission_manager/cancel",
    ):
        assert legacy not in source
    assert "SetBool" in source
    assert "Trigger" in source


def test_mission_manager_node_uses_core_state_authority():
    source = (SRC / "parking_robot_mission_manager" / "parking_robot_mission_manager" / "mission_manager_node.py").read_text()
    assert "MissionStateMachine(" in source
    assert "self._core" in source
    assert "def _set_state" not in source
    assert "self._state =" not in source
    assert "validate_route_mission(" not in source
