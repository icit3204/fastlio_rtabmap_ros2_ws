import ast
from pathlib import Path

import parking_robot_mission_manager.mission_progress_supervisor_core as module


def test_static_purity_contract():
    path = Path(module.__file__)
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = {alias.name for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom))
               for alias in node.names}
    forbidden_import_roots = {"rclpy", "nav2_msgs", "geometry_msgs", "parking_robot_interfaces",
                              "action_msgs", "socket", "threading", "time", "can"}
    assert not ({name.split(".")[0] for name in imports} & forbidden_import_roots)
    forbidden_tokens = ("create_publisher", "create_subscription", "ActionClient", "sleep(",
                        "time.time", "monotonic(", "socket(", "udp", "vcan", "hardware")
    lowered = source.lower()
    assert all(token.lower() not in lowered for token in forbidden_tokens)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    assert not any(isinstance(call.func, ast.Name) and call.func.id == "open" for call in calls)


def test_existing_authority_modules_do_not_import_supervisor():
    package = Path(module.__file__).parent
    # P4-E.4B permits the sole ROS owner to call the passive core; the state
    # authority must remain independent of it.
    assert "mission_progress_supervisor_core" not in (package / "mission_state_machine.py").read_text(encoding="utf-8")
