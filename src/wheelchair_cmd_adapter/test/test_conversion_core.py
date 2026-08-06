import math

from wheelchair_cmd_adapter.conversion_core import (
    AdapterConfig,
    AdapterReason,
    AdapterState,
    CommandValues,
    evaluate_command,
    validate_topic_contract,
)


CFG = AdapterConfig()


def state(*, age=0.0, input_publishers=1, output_publishers=1, has_input=True):
    return AdapterState(
        has_input=has_input,
        input_age_sec=age,
        input_publisher_count=input_publishers,
        output_publisher_count=output_publishers,
    )


def cmd(**kwargs):
    values = {
        "frame_id": "base_footprint",
        "linear_x": 0.0,
        "linear_y": 0.0,
        "linear_z": 0.0,
        "angular_x": 0.0,
        "angular_y": 0.0,
        "angular_z": 0.0,
    }
    values.update(kwargs)
    return CommandValues(**values)


def assert_result(values, expected_output, expected_reason=AdapterReason.VALID, st=None):
    result = evaluate_command(values, st or state(), CFG)
    assert result.reason == expected_reason
    assert list(result.output) == expected_output
    assert len(result.output) == 3
    assert result.output[2] == 0.0
    return result


def test_stop():
    assert_result(cmd(linear_x=0.0, angular_z=0.0), [0.0, 0.0, 0.0])


def test_straight():
    assert_result(cmd(linear_x=0.10, angular_z=0.0), [10000.0, 100.0, 0.0])


def test_left_curve():
    assert_result(cmd(linear_x=0.10, angular_z=0.05), [-2000.0, 100.0, 0.0])


def test_right_curve():
    assert_result(cmd(linear_x=0.10, angular_z=-0.05), [2000.0, 100.0, 0.0])


def test_minimum_accepted_left_radius():
    assert_result(cmd(linear_x=0.10, angular_z=0.10), [-1000.0, 100.0, 0.0])


def test_large_radius_canonical_straight():
    assert_result(cmd(linear_x=0.10, angular_z=0.005), [10000.0, 100.0, 0.0])


def test_tight_radius_rejection():
    assert_result(
        cmd(linear_x=0.10, angular_z=0.20),
        [0.0, 0.0, 0.0],
        AdapterReason.TURN_RADIUS_UNSUPPORTED,
    )


def test_reverse_rejection_and_noise_normalization():
    assert_result(
        cmd(linear_x=-0.01, angular_z=0.0),
        [0.0, 0.0, 0.0],
        AdapterReason.REVERSE_UNSUPPORTED,
    )
    assert_result(cmd(linear_x=-1e-7, angular_z=0.0), [0.0, 0.0, 0.0])


def test_in_place_and_zero_linear_nonzero_angular_rejection():
    assert_result(
        cmd(linear_x=0.005, angular_z=0.03),
        [0.0, 0.0, 0.0],
        AdapterReason.IN_PLACE_ROTATION_UNSUPPORTED,
    )
    assert_result(
        cmd(linear_x=0.0, angular_z=0.01),
        [0.0, 0.0, 0.0],
        AdapterReason.IN_PLACE_ROTATION_UNSUPPORTED,
    )


def test_nan_and_inf_rejection_for_every_twist_field():
    fields = ["linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"]
    for field in fields:
        for value in [math.nan, math.inf, -math.inf]:
            assert_result(cmd(**{field: value}), [0.0, 0.0, 0.0], AdapterReason.NUMERICAL_INVALID)


def test_unsupported_axes_rejection():
    for field in ["linear_y", "linear_z", "angular_x", "angular_y"]:
        assert_result(cmd(**{field: 2e-6}), [0.0, 0.0, 0.0], AdapterReason.UNSUPPORTED_AXES)
        assert_result(cmd(**{field: 1e-6}), [0.0, 0.0, 0.0])


def test_wrong_and_empty_frame_rejection():
    assert_result(cmd(frame_id="map"), [0.0, 0.0, 0.0], AdapterReason.FRAME_INVALID)
    assert_result(cmd(frame_id=""), [0.0, 0.0, 0.0], AdapterReason.FRAME_INVALID)


def test_input_over_limit_rejection():
    assert_result(cmd(linear_x=0.200001), [0.0, 0.0, 0.0], AdapterReason.OVER_LIMIT)
    assert_result(cmd(linear_x=0.10, angular_z=0.500001), [0.0, 0.0, 0.0], AdapterReason.OVER_LIMIT)


def test_stale_and_exact_timeout_boundary():
    assert_result(cmd(), [0.0, 0.0, 0.0], st=state(age=0.25))
    assert_result(cmd(), [0.0, 0.0, 0.0], AdapterReason.INPUT_STALE, st=state(age=0.250001))


def test_missing_and_duplicate_authority():
    assert_result(cmd(), [0.0, 0.0, 0.0], AdapterReason.INPUT_AUTHORITY_INVALID, st=state(input_publishers=0))
    assert_result(cmd(), [0.0, 0.0, 0.0], AdapterReason.INPUT_AUTHORITY_INVALID, st=state(input_publishers=2))
    assert_result(cmd(), [0.0, 0.0, 0.0], AdapterReason.OUTPUT_AUTHORITY_INVALID, st=state(output_publishers=0))
    assert_result(cmd(), [0.0, 0.0, 0.0], AdapterReason.OUTPUT_AUTHORITY_INVALID, st=state(output_publishers=2))


def test_startup_zero_and_automatic_recovery():
    result = evaluate_command(None, state(has_input=False), CFG)
    assert result.reason == AdapterReason.STARTUP_ZERO
    assert list(result.output) == [0.0, 0.0, 0.0]
    assert_result(cmd(linear_x=0.10), [10000.0, 100.0, 0.0])


def test_turn_radius_boundaries():
    below_min = math.nextafter(1.0, 0.0)
    below_straight = math.nextafter(10.0, 0.0)
    assert_result(
        cmd(linear_x=0.10, angular_z=0.10 / below_min),
        [0.0, 0.0, 0.0],
        AdapterReason.TURN_RADIUS_UNSUPPORTED,
    )
    assert_result(cmd(linear_x=0.10, angular_z=0.10), [-1000.0, 100.0, 0.0])
    result = assert_result(cmd(linear_x=0.10, angular_z=0.10 / below_straight), [-(below_straight * 1000.0), 100.0, 0.0])
    assert result.output[2] == 0.0
    assert_result(cmd(linear_x=0.10, angular_z=0.01), [10000.0, 100.0, 0.0])


def test_authority_loss_conflict_and_automatic_recovery():
    straight = cmd(linear_x=0.10)
    assert_result(straight, [10000.0, 100.0, 0.0], st=state(input_publishers=1))
    assert_result(straight, [0.0, 0.0, 0.0], AdapterReason.INPUT_AUTHORITY_INVALID, st=state(input_publishers=2))
    assert_result(straight, [10000.0, 100.0, 0.0], st=state(input_publishers=1))
    assert_result(straight, [0.0, 0.0, 0.0], AdapterReason.INPUT_AUTHORITY_INVALID, st=state(input_publishers=0))
    assert_result(straight, [0.0, 0.0, 0.0], AdapterReason.OUTPUT_AUTHORITY_INVALID, st=state(output_publishers=2))
    assert_result(straight, [10000.0, 100.0, 0.0], st=state(output_publishers=1))


def test_topic_contract_rejects_every_non_contract_override():
    validate_topic_contract("/vehicle_cmd_safe", "/wheelchair_control_command_mock")
    for unsafe_input in [
        "/cmd_vel_nav_raw",
        "/cmd_vel_nav_safe",
        "/cmd_vel",
        "/wheelchair_control_command_raw",
        "/wheelchair_control_command",
        "/some_other_input",
    ]:
        try:
            validate_topic_contract(unsafe_input, "/wheelchair_control_command_mock")
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted unsafe input topic {unsafe_input}")
    for unsafe_output in ["/wheelchair_control_command", "/wheelchair_control_command_raw", "/some_other_output"]:
        try:
            validate_topic_contract("/vehicle_cmd_safe", unsafe_output)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted unsafe output topic {unsafe_output}")
