"""Full C-P02 qualification composition with one shadow-feedback authority.

The accepted C.2B stack is reused, but the old health feedback relay is
removed from the action list and the single Mission Manager remap is changed
to the C-P02 recovery shadow topic.  This keeps one Mission Manager and one
canonical NavigateToPose action topology.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def _base_description():
    base_path = Path(__file__).with_name("phase4_p4e6b_health_matrix.launch.py")
    spec = importlib.util.spec_from_file_location("p4e6c_base_matrix", base_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def generate_launch_description():
    base = _base_description()
    entities = []
    for entity in base.entities:
        executable = getattr(entity, "_Node__node_executable", None)
        node_name = getattr(entity, "_Node__node_name", None)
        if executable == "phase4_p4e6b_feedback_relay":
            continue
        if executable == "mission_manager_node" and node_name == "mission_manager":
            remappings = list(getattr(entity, "_Node__remappings", ()))
            def text_value(value):
                if isinstance(value, tuple) and len(value) == 1:
                    value = value[0]
                return getattr(value, "text", value)
            remappings = [
                (source, (TextSubstitution(text="/phase4_qualification/p4e6c/navigate_to_pose_feedback"),)
                 if text_value(source) == "/navigate_to_pose/_action/feedback" else target)
                for source, target in remappings
            ]
            entity._Node__remappings = remappings
        entities.append(entity)
    entities.append(Node(
        package="parking_robot_bringup",
        executable="phase4_p4e6c_recovery_feedback_relay",
        name="phase4_p4e6c_recovery_feedback_relay",
        output="screen",
    ))
    entities.append(Node(
        package="parking_robot_bringup",
        executable="phase4_p4e6c_progress_observer",
        name="phase4_p4e6c_progress_observer",
        parameters=[{"case_id": "C-P02"}],
        output="screen",
    ))
    return LaunchDescription(entities)
