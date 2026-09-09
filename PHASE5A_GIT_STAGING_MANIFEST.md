# Phase 5A Git Staging Manifest

Classification performed before staging for the MK2F4 mock-autonomy
checkpoint. Protected baseline:
`de990c7b4b27e56a8ae1402e5d55777eb60a9c18`.

## A-E — accepted project content to stage

The following modified tracked files are accepted Phase-5 project source,
configuration, tests, launch/package metadata, or project documentation:

```text
src/FAST_LIO_ROS2/CMakeLists.txt
src/FAST_LIO_ROS2/launch/mapping.launch.py
src/FAST_LIO_ROS2/src/laserMapping.cpp
src/livox_ros_driver2/config/MID360_config.json
src/parking_robot_bringup/config/phase4_p4e_gate_mock.yaml
src/parking_robot_bringup/package.xml
src/parking_robot_mission_manager/parking_robot_mission_manager/mission_manager_node.py
src/parking_robot_mission_manager/parking_robot_mission_manager/mission_state_machine.py
src/parking_robot_mission_manager/test/test_mission_progress_observation_adapter.py
src/parking_robot_mission_manager/test/test_mission_state_machine.py
src/robot_bringup/CMakeLists.txt
src/robot_bringup/config/nav2_common.yaml
src/robot_bringup/launch/bringup.launch.py
src/robot_bringup/package.xml
src/vehicle_cmd_safety/config/phase4_p4c_gate_mock.yaml
src/vehicle_cmd_safety/package.xml
src/vehicle_cmd_safety/setup.py
src/vehicle_cmd_safety/test/test_phase4_p4c_static_contract.py
src/vehicle_cmd_safety/vehicle_cmd_safety/collision_monitor_validity_monitor.py
src/vehicle_cmd_safety/vehicle_cmd_safety/gate_core.py
src/vehicle_cmd_safety/vehicle_cmd_safety/guarded_vehicle_cmd_gate.py
src/vehicle_cmd_safety/vehicle_cmd_safety/validity_core.py
src/wheelchair_cmd_adapter/setup.py
src/wheelchair_cmd_adapter/test/test_static_contract.py
src/wheelchair_controller/CMakeLists.txt
src/wheelchair_controller/config/wheelchair_controller_param.yaml
src/wheelchair_controller/launch/wheelchair_controller.launch.py
src/wheelchair_controller/package.xml
src/wheelchair_controller/src/wheelchair_controller_node.cpp
键盘控制.md
```

The following untracked project paths are also accepted Phase-5 source,
configuration, tests, launch metadata, or documentation and will be staged in
full:

```text
src/FAST_LIO_ROS2/include/startup_freshness_guard.hpp
src/FAST_LIO_ROS2/test/
src/mkmini_cmd_adapter/
src/mkmini_description/
src/parking_robot_bringup/config/collision_monitor_phase5_raw_mid360.yaml
src/parking_robot_bringup/config/phase5_collision_monitor_validity.yaml
src/parking_robot_bringup/config/phase5_gate_mock.yaml
src/parking_robot_bringup/config/phase5_no_motion_nav2.yaml
src/parking_robot_bringup/launch/phase5_no_motion.launch.py
src/parking_robot_bringup/test/test_phase5_fast_lio_startup_guard_contract.py
src/parking_robot_bringup/test/test_phase5_no_motion_static_contract.py
src/parking_robot_bringup/test/test_phase5_sensor_freshness_boundary_contract.py
src/phase5_sensor_freshness_boundary/
src/robot_bringup/config/ackermann_navigate_through_poses.xml
src/robot_bringup/config/ackermann_navigate_to_pose.xml
src/robot_bringup/config/collision_monitor_dual_sensor.yaml
src/robot_bringup/config/tmini_mk2c.yaml
src/robot_bringup/docs/
src/robot_bringup/launch/mk2c_local_costmap_observation.launch.py
src/robot_bringup/launch/mk2e4_labmate_backend_mock.launch.py
src/robot_bringup/launch/mkmini_calibrated_localization.launch.py
src/robot_bringup/launch/phase5_calibrated_localization.launch.py
src/robot_bringup/scripts/mk2c1r_reconciliation_observer.py
src/robot_bringup/scripts/mk2c_costmap_observer.py
src/robot_bringup/scripts/mk2c_floor_return_observer.py
src/robot_bringup/scripts/mk2f1_mppi_stationary_runner.py
src/robot_bringup/scripts/mk2f2_planner_stationary_runner.py
src/robot_bringup/scripts/mk2f3_navigate_to_pose_stationary_runner.py
src/robot_bringup/scripts/mk2f4_mission_manager_stationary_runner.py
src/robot_bringup/test/
src/vehicle_cmd_safety/config/phase5_dual_fail_close_gate.yaml
src/vehicle_cmd_safety/config/phase5_dual_mid360_validity.yaml
src/vehicle_cmd_safety/config/phase5_dual_tmini_validity.yaml
src/vehicle_cmd_safety/config/phase5_localization_validity.yaml
src/vehicle_cmd_safety/test/mk2d2b_command_injector.py
src/vehicle_cmd_safety/test/mk2d2b_event_recorder.py
src/vehicle_cmd_safety/test/test_p5_3f5j1_fault_reset.py
src/vehicle_cmd_safety/test/test_p5_3f5j2_arm_pending_fault_latch.py
src/vehicle_cmd_safety/test/test_p5_3f5j_safe_zero_quiescence.py
src/vehicle_cmd_safety/test/test_phase5_collision_validity_timestamp_parity.py
src/vehicle_cmd_safety/test/test_phase5_localization_validity_contract.py
src/vehicle_cmd_safety/test/test_phase5_localization_validity_core.py
src/vehicle_cmd_safety/test/test_phase5_localization_validity_monitor_logging.py
src/vehicle_cmd_safety/vehicle_cmd_safety/localization_validity_core.py
src/vehicle_cmd_safety/vehicle_cmd_safety/localization_validity_monitor.py
src/vehicle_cmd_safety/vehicle_cmd_safety/required_perception_validity.py
src/wheelchair_cmd_adapter/config/gate_to_labmate_bridge.yaml
src/wheelchair_cmd_adapter/test/mk2e4_mock_qualification_runner.py
src/wheelchair_cmd_adapter/test/mk2e5_canonical_chain_runner.py
src/wheelchair_cmd_adapter/test/mk2e5_passive_chain_observer.py
src/wheelchair_cmd_adapter/test/test_labmate_bridge_core.py
src/wheelchair_cmd_adapter/wheelchair_cmd_adapter/gate_to_labmate_bridge.py
src/wheelchair_cmd_adapter/wheelchair_cmd_adapter/labmate_bridge_core.py
src/wheelchair_controller/include/
src/wheelchair_controller/src/mkmini_backend_core.cpp
src/wheelchair_controller/test/
MISSION_MANAGER_GUI_STATUS_CONTRACT.md
MISSION_MANAGER_RUNTIME_CONTRACT.md
MISSION_MANAGER_STATE_MACHINE.md
docs/phases/phase_05/
```

The two root mission-manager contracts are project documentation for this
checkpoint. `src/mkmini_cmd_adapter` is retained earlier Phase-5 support code;
the canonical MK2F4 launch selects the labmate-derived
`wheelchair_controller` MockTransport path.

## F-K — excluded content

The following was reviewed and excluded:

- `build/`, `install/`, `log/`, Python bytecode/cache, and editor output:
  generated/cache/runtime content, covered by `.gitignore` where applicable.
- `B2*` files: later/historical B2 evidence and replay outputs, outside the
  MK2F4 checkpoint scope.
- `docs/phases/phase_04/PHASE4_*`: historical uncommitted worktree manifests,
  not needed to represent the accepted Phase-5A implementation.
- `docs/phases/temp text.txt`: temporary session text.
- `frames_*.gv`: diagnostic graph captures.
- `RUN.TXT`, `can_faliar.md`, and `keyboard_can_control.py`: operator/CAN
  notes and manual hardware-control material, not part of mock autonomy.
- `/home/dog/phase5_reports/` evidence archives and `/home/dog/phase5_runtime/`
  RTAB working copies: external evidence/runtime content, not copied into the
  repository.
- `map/rtabmap_2d.db`: ignored canonical runtime map; its required hash was
  verified separately and it is intentionally not staged.

No unknown (`L`) files remain in the reviewed untracked inventory. No secrets,
private keys, unexpected binaries, or large generated files were found among
the proposed staged paths.
