from pathlib import Path


def test_r3h_launch_binds_only_production_authorities():
    source = (Path(__file__).parents[1] / "launch" / "r3h_physical_navigation.launch.py").read_text()
    assert "20260915_155828_current_room_final_production/rtabmap_2d.db" in source
    assert "2f517e07e2cf788f0695ea421a816fc030467075a79c75457ff2840529f2f1a8" in source
    assert "prepare_rtabmap_working_copy" in source
    assert 'context.launch_configurations["database_path"] = str(runtime_db)' in source
    assert "nav2_common.yaml" in source
    assert "collision_monitor_dual_sensor.yaml" in source
    assert "r3h_physical_gate.yaml" in source
    assert "mkmini_physical_ros_backend" in source
    assert 'prefix="taskset -c 5,7"' in source
    assert "R3H_NAVIGATION_CPUS = {0, 1, 2, 3}" in source
    assert "os.sched_setaffinity(0, R3H_NAVIGATION_CPUS)" in source
    assert "required_cpus.issubset(available_cpus)" in source
    assert 'prefix="taskset -c 7"' in source
    assert source.index("OpaqueFunction(function=isolate_r3h_navigation_processes)") < source.index("instance_guard, guard_exit_shutdown")
    assert 'executable="nav2_controller_validity_monitor"' in source
    assert '"validity_output_topic": "/system/controller_valid"' in source
    assert '"validity_output_topic": "/system/collision_monitor_valid"' in source
    assert "phase4_p4c_permission_fixture" not in source
    assert "mock_wheelchair_cmd_adapter" not in source
    assert "wheelchair_cmd_adapter" not in source
    assert "r3h_instance_guard" in source
    assert "TimerAction(period=0.75" in source
    assert "OnProcessExit" in source
    assert 'Shutdown(reason="R3H singleton guard lost")' in source
    gate = (Path(__file__).parents[2] / "vehicle_cmd_safety" / "config" / "r3h_physical_gate.yaml").read_text()
    assert "output_topic: /vehicle_cmd_safe" in gate
    assert "heartbeat_hz: 20.0" in gate
    assert "max_forward_velocity: 0.040" in gate


def test_r3h_singleton_lock_rejects_second_owner(tmp_path):
    from parking_robot_bringup.r3h_instance_guard import (
        InstanceAlreadyRunning,
        R3hInstanceLock,
    )

    path = tmp_path / "r3h.lock"
    first = R3hInstanceLock(path)
    second = R3hInstanceLock(path)
    first.acquire()
    try:
        try:
            second.acquire()
        except InstanceAlreadyRunning:
            pass
        else:
            raise AssertionError("a second R3H authority acquired the singleton lock")
    finally:
        first.close()

    second.acquire()
    second.close()
