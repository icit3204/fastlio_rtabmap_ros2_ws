import math
from itertools import permutations
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from parking_robot_mission_manager.mission_manager_node import (
    INITIAL_COMMAND_ACQUISITION_TIMEOUT_SEC,
    MissionManagerNode,
    ProgressPolicyActivationState,
)
from parking_robot_mission_manager.mission_progress_observation_adapter import (
    CausalCollisionMeaning, CausalCommandPairer, CausalPairState,
    CommandObservationRecord, CommandStreamPhase,
    adapter_health_sample, bool_health_sample, command_sample, feedback_sample,
    gate_health_sample, odometry_sample, transform_sample, transform_stamp_key,
)
from parking_robot_mission_manager.mission_progress_supervisor_core import (
    BoolHealthSample, CommandSample, FeedbackSample, GateHealthSample, GateState,
    CollisionClassification, MissionProgressSupervisorCore, OdometrySample,
    OptionalAdapterHealthSample, SupervisorEvent, SupervisorInputs, TfSample,
)
from parking_robot_mission_manager.mission_state_machine import (
    GoalOutcome, MissionGoalExecutor, MissionSnapshot, MissionStateCode,
)


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "parking_robot_mission_manager" / "mission_manager_node.py"


def feedback(distance=4.5, recoveries=2):
    msg = NavigateToPose.Feedback()
    msg.current_pose.pose.position.x = 1.25
    msg.current_pose.pose.position.y = -2.5
    msg.current_pose.pose.orientation.z = math.sin(0.25)
    msg.current_pose.pose.orientation.w = math.cos(0.25)
    msg.navigation_time.sec = 3
    msg.navigation_time.nanosec = 250_000_000
    msg.estimated_time_remaining.sec = 7
    msg.estimated_time_remaining.nanosec = 500_000_000
    msg.number_of_recoveries = recoveries
    msg.distance_remaining = distance
    return msg


def status(state="ARMED", fault=False, level=DiagnosticStatus.OK):
    msg = DiagnosticStatus()
    msg.level = level
    msg.message = "TEST"
    msg.values = [KeyValue(key="state", value=state),
                  KeyValue(key="fault_latched", value=str(fault).lower()),
                  KeyValue(key="reason_code", value="TEST")]
    return msg


def adapter(valid=True):
    array = DiagnosticArray()
    item = DiagnosticStatus()
    item.name = "mock_wheelchair_cmd_adapter"
    item.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
    item.message = "VALID" if valid else "INPUT_STALE"
    array.status = [item]
    return array


def test_feedback_converts_all_frozen_fields():
    sample = feedback_sample(feedback(), 10.0)
    assert (sample.x, sample.y, sample.yaw) == pytest.approx((1.25, -2.5, 0.5))
    assert (sample.navigation_time_sec, sample.estimated_time_remaining_sec) == (3.25, 7.5)
    assert sample.number_of_recoveries == 2
    assert sample.distance_remaining_m == pytest.approx(4.5)


def test_feedback_uses_supplied_steady_receipt():
    assert feedback_sample(feedback(), 123.75).steady_receipt_sec == 123.75


def test_odometry_converts_pose_twist_and_planar_yaw():
    msg = Odometry(); msg.pose.pose.position.x = 2.0; msg.pose.pose.position.y = 3.0
    msg.pose.pose.orientation.z = math.sin(-0.3); msg.pose.pose.orientation.w = math.cos(-0.3)
    msg.twist.twist.linear.x = 0.4; msg.twist.twist.angular.z = -0.2
    sample = odometry_sample(msg, 8.0)
    assert (sample.x, sample.y, sample.yaw, sample.linear_x, sample.angular_z) == pytest.approx((2, 3, -0.6, .4, -.2))


@pytest.mark.parametrize("linear,angular", [(0.3, 0.1), (-0.2, -0.4)])
def test_raw_and_safe_twist_conversion(linear, angular):
    msg = Twist(); msg.linear.x = linear; msg.angular.z = angular
    sample = command_sample(msg, 9.0)
    assert (sample.steady_receipt_sec, sample.linear_x, sample.angular_z) == (9.0, linear, angular)


@pytest.mark.parametrize("value", [True, False])
def test_bool_health_conversion_for_each_value(value):
    msg = Bool(); msg.data = value
    sample = bool_health_sample(msg, 4.0)
    assert sample.value is value and sample.steady_receipt_sec == 4.0


@pytest.mark.parametrize("state_name,expected", [("DISARMED", GateState.DISARMED), ("ARMED", GateState.ARMED)])
def test_gate_state_parser(state_name, expected):
    assert gate_health_sample(status(state_name), 2.0).state is expected


def test_gate_fault_and_latch_parser():
    sample = gate_health_sample(status("FAULT", True, DiagnosticStatus.ERROR), 2.0)
    assert sample.state is GateState.FAULT and sample.fault_latched


@pytest.mark.parametrize("values", [[], [KeyValue(key="state", value="ARMED")],
                                         [KeyValue(key="state", value="BROKEN"), KeyValue(key="fault_latched", value="false")],
                                         [KeyValue(key="state", value="ARMED"), KeyValue(key="fault_latched", value="maybe")]])
def test_malformed_gate_diagnostic_rejected(values):
    msg = DiagnosticStatus(); msg.values = values
    assert gate_health_sample(msg, 1.0) is None


def test_optional_adapter_valid():
    assert adapter_health_sample(adapter(True), 3.0).valid


def test_optional_adapter_invalid():
    sample = adapter_health_sample(adapter(False), 3.0)
    assert not sample.valid and sample.reason == "INPUT_STALE"


def test_optional_adapter_unrelated_or_ambiguous_rejected():
    assert adapter_health_sample(DiagnosticArray(), 3.0) is None
    msg = adapter(); msg.status.append(msg.status[0])
    assert adapter_health_sample(msg, 3.0) is None


@pytest.mark.parametrize("converter,message", [
    (command_sample, NS(linear=NS(x=float("nan")), angular=NS(z=0.0))),
    (odometry_sample, NS()),
    (feedback_sample, feedback(float("inf"))),
])
def test_malformed_numeric_does_not_fabricate_sample(converter, message):
    assert converter(message, 1.0) is None


def test_transform_conversion_accepts_composite_result():
    msg = TransformStamped(); msg.header.frame_id = "odom"; msg.child_frame_id = "base_footprint"
    msg.transform.translation.x = 1.0; msg.transform.translation.y = 2.0; msg.transform.rotation.w = 1.0
    sample = transform_sample(msg, 12.0)
    assert (sample.x, sample.y, sample.yaw, sample.steady_receipt_sec) == (1.0, 2.0, 0.0, 12.0)


def test_cached_transform_stamp_does_not_become_new_evidence():
    msg = TransformStamped(); msg.header.stamp.sec = 8; msg.header.stamp.nanosec = 5
    assert transform_stamp_key(msg) == transform_stamp_key(msg) == (8, 5)


def test_genuinely_new_transform_stamp_changes_evidence_key():
    first = TransformStamped(); first.header.stamp.sec = 8
    second = TransformStamped(); second.header.stamp.sec = 8; second.header.stamp.nanosec = 1
    assert transform_stamp_key(first) != transform_stamp_key(second)


class FixedBuffer:
    def __init__(self, msg): self.msg = msg
    def lookup_transform(self, *args, **kwargs): del args, kwargs; return self.msg


def test_repeated_cached_lookup_does_not_refresh_tf_sample(manager):
    node, _ = manager; msg = TransformStamped(); msg.header.stamp.sec = 5; msg.transform.rotation.w = 1.0
    node._tf_buffer = FixedBuffer(msg); node._observe_transform(10.0); node._observe_transform(11.0)
    assert node._latest_transform.steady_receipt_sec == 10.0


def test_new_tf_evidence_refreshes_tf_receipt(manager):
    node, _ = manager; msg = TransformStamped(); msg.header.stamp.sec = 5; msg.transform.rotation.w = 1.0
    buffer = FixedBuffer(msg); node._tf_buffer = buffer; node._observe_transform(10.0)
    msg.header.stamp.nanosec = 1; node._observe_transform(11.0)
    assert node._latest_transform.steady_receipt_sec == 11.0


def test_static_zero_stamp_cannot_refresh_dynamic_freshness():
    assert transform_stamp_key(TransformStamped()) is None


class QuietExecutor(MissionGoalExecutor):
    def __init__(self): self.sent = 0; self.cancelled = 0
    def server_available(self): return True
    def send_goal(self, pose, result_callback):
        del pose, result_callback; self.sent += 1
        return GoalOutcome(True, goal_uuid="transport-owned", terminal_status=None)
    def cancel_goal(self, goal_uuid, timeout_sec):
        del goal_uuid, timeout_sec; self.cancelled += 1; return True


@pytest.fixture
def manager():
    rclpy.init(); executor = QuietExecutor(); node = MissionManagerNode(goal_executor=executor, steady_clock=lambda: 10.0)
    yield node, executor
    node.destroy_node(); rclpy.shutdown()


def test_old_uuid_feedback_ignored(manager):
    node, _ = manager; node.accept_progress_goal("new")
    node.receive_nav_feedback("old", feedback(), 10.0)
    assert node._latest_feedback is None


def test_active_uuid_feedback_accepted(manager):
    node, _ = manager; node.accept_progress_goal("active")
    node.receive_nav_feedback("active", feedback(), 10.0)
    assert node._latest_feedback.distance_remaining_m == pytest.approx(4.5)


def test_new_accepted_waypoint_resets_exactly_once(manager):
    node, _ = manager
    node.accept_progress_goal("a"); node.accept_progress_goal("a"); node.accept_progress_goal("b")
    assert node._progress_goal_reset_count == 2


def test_new_goal_uses_available_independent_pose_baseline(manager):
    node, _ = manager
    odom = Odometry(); odom.pose.pose.position.x = 4.0; odom.pose.pose.position.y = -2.0
    node._odometry_cb(odom); node.accept_progress_goal("pose-baseline")
    assert node._progress_supervisor._baseline_pose == (4.0, -2.0)


def test_duplicate_feedback_does_not_reset_goal_history(manager):
    node, _ = manager; node.accept_progress_goal("a")
    node.receive_nav_feedback("a", feedback(), 10.0); node.receive_nav_feedback("a", feedback(), 10.1)
    assert node._progress_goal_reset_count == 1 and len(node._feedback_receipt_times) == 2


def test_stale_sample_receipt_is_not_refreshed_without_callback(manager):
    node, _ = manager; node._odometry_cb(Odometry())
    assert node._latest_odometry.steady_receipt_sec == 10.0


def test_freshness_boundary_is_not_reimplemented_in_adapter_source():
    source = (ROOT / "parking_robot_mission_manager" / "mission_progress_observation_adapter.py").read_text()
    for threshold_name in ("feedback_freshness_sec", "odometry_freshness_sec",
                           "tf_freshness_sec", "command_pair_freshness_sec"):
        assert threshold_name not in source


def test_passive_evaluation_has_no_state_or_action_authority(manager):
    node, executor = manager; before = node.mission_state_machine.state
    node.accept_progress_goal("a"); node._evaluate_progress_passively(10.0)
    assert node.mission_state_machine.state is before
    assert executor.sent == 0 and executor.cancelled == 0


def test_existing_action_client_is_only_action_client():
    source = NODE.read_text()
    assert source.count("ActionClient(") == 1
    assert "feedback_callback=self._feedback_cb" in source


def test_no_command_publisher_or_new_public_message_introduced():
    source = NODE.read_text()
    assert "create_publisher(Twist" not in source
    assert not list((ROOT.parent / "parking_robot_interfaces" / "msg").glob("*Supervisor*"))


def test_supervisor_result_is_never_routed_to_mission_core():
    source = NODE.read_text()
    body = source[source.index("def _evaluate_progress_passively"):source.index("def _publish_passive_status")]
    for forbidden in ("apply_core_event", "request_cancel", "send_goal", "transition"):
        assert forbidden not in body


def policy_evidence(node, now, *, feedback_present=True, gate=GateState.ARMED,
                    gate_fault=False, collision=True, localization=True,
                    controller=True, adapter_valid=None, stale=None):
    stamp = lambda family: now - 3.0 if stale == family else now
    node._latest_feedback = (
        FeedbackSample(stamp("feedback"), 0.0, 0.0, 0.0, 0.0, 0.0, 0, 3.0)
        if feedback_present else None
    )
    node._latest_odometry = OdometrySample(stamp("odometry"), 0.0, 0.0, 0.0, 0.0, 0.0)
    node._latest_transform = TfSample(stamp("tf"), 0.0, 0.0, 0.0)
    node._latest_raw_command = CommandSample(stamp("command"), 0.2, 0.0)
    node._latest_safe_command = CommandSample(stamp("command"), 0.2, 0.0)
    node._command_pairer.observe_raw(node._latest_raw_command)
    pair = node._command_pairer.observe_safe(node._latest_safe_command)
    node._causal_raw_command = CommandSample(
        pair.raw.steady_receipt_sec, pair.raw.linear_x, pair.raw.angular_z)
    node._causal_safe_command = CommandSample(
        pair.safe.steady_receipt_sec, pair.safe.linear_x, pair.safe.angular_z)
    node._latest_gate = GateHealthSample(stamp("gate"), gate, gate_fault)
    node._latest_collision_valid = BoolHealthSample(stamp("collision"), collision)
    node._latest_localization_valid = BoolHealthSample(stamp("localization"), localization)
    node._latest_controller_valid = BoolHealthSample(stamp("controller"), controller)
    node._latest_adapter_health = (
        None if adapter_valid is None else OptionalAdapterHealthSample(stamp("adapter"), adapter_valid)
    )


def evaluate_policy(node, now):
    node._evaluate_progress_passively(now)
    assert node._latest_passive_result is not None
    selected = node._progress_policy_result_to_apply(node._latest_passive_result, now)
    return selected or node._latest_passive_result, selected is not None


def pair_sample(at, linear, angular=0.0):
    return CommandSample(at, linear, angular)


def qualified_inputs(now, raw=(0.2, 0.0), safe=(0.0, 0.0)):
    return SupervisorInputs(
        FeedbackSample(now, 0.0, 0.0, 0.0, now, 1.0, 0, 5.0),
        OdometrySample(now, 0.0, 0.0, 0.0, 0.0, 0.0),
        TfSample(now, 0.0, 0.0, 0.0), CommandSample(now, *raw),
        CommandSample(now, *safe), GateHealthSample(now, GateState.ARMED, False),
        BoolHealthSample(now, True), BoolHealthSample(now, True),
        BoolHealthSample(now, True), None,
    )


@pytest.mark.parametrize("raw,safe,meaning", [
    ((0.2, 0.1), (0.2, 0.1), CausalCollisionMeaning.CLEAR),
    ((0.2, 0.1), (0.06, 0.03), CausalCollisionMeaning.SLOWDOWN),
    ((0.2, 0.1), (0.0, 0.0), CausalCollisionMeaning.STOP),
    ((0.0, 0.0), (0.0, 0.0), CausalCollisionMeaning.NO_MOVEMENT),
])
def test_causal_pairer_classifies_generic_cm_transformations(raw, safe, meaning):
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.01, *raw))
    result = pairer.observe_safe(pair_sample(1.02, *safe))
    assert result.state is CausalPairState.VALID
    assert result.meaning is meaning and not result.drain_only
    assert pairer.command_stream_phase is CommandStreamPhase.STREAM_ESTABLISHED


def test_epoch_starts_no_raw_authority_without_pair_clock():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    assert pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW
    assert pairer.no_raw_acquisition_start_sec == 1.0
    assert pairer.unavailable_since_sec is None
    assert pairer.no_raw_acquisition_age(1.999) == pytest.approx(0.999)
    assert pairer.pair_liveness_age(1.999) is None
    assert pairer.adjudicate(9.0).state is CausalPairState.PENDING


def test_first_raw_atomically_transfers_timer_ownership_and_repeated_raw_does_not_slide():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.9, 0.2))
    assert pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_RAW_PENDING
    assert pairer.no_raw_acquisition_start_sec is None
    assert pairer.unavailable_since_sec == pytest.approx(1.9)
    pairer.observe_raw(pair_sample(1.99, 0.3))
    assert pairer.unavailable_since_sec == pytest.approx(1.9)


@pytest.mark.parametrize("raw_at,safe_at", [(1.9, 2.08), (1.99, 2.239)])
def test_late_first_raw_gets_full_250ms_pair_window(raw_at, safe_at):
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(raw_at, 0.2))
    assert pairer.adjudicate(2.0).state is CausalPairState.PENDING
    result = pairer.observe_safe(pair_sample(safe_at, 0.2))
    assert result.state is CausalPairState.VALID
    assert pairer.command_stream_phase is CommandStreamPhase.STREAM_ESTABLISHED


def test_late_first_raw_missing_safe_stales_at_raw_plus_250ms():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.99, 0.2))
    assert pairer.adjudicate(2.239999).state is CausalPairState.PENDING
    assert pairer.adjudicate(2.24).state is CausalPairState.STALE


def test_safe_before_raw_neither_starts_pair_clock_nor_establishes():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    assert pairer.observe_safe(pair_sample(1.1, 0.2)).state is CausalPairState.PENDING
    assert pairer.unavailable_since_sec is None
    pairer.observe_raw(pair_sample(1.2, 0.2))
    assert pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_RAW_PENDING
    assert pairer.observe_safe(pair_sample(1.21, 0.2)).state is CausalPairState.VALID


def test_exact_a8d_chronology_has_no_old_epoch_stale_and_establishes():
    epoch = 693987.735263591
    raw = 693987.970802396
    safe = 693987.971830331
    old_deadline = 693987.985263591
    pairer = CausalCommandPairer(); pairer.start_epoch(epoch)
    assert pairer.adjudicate(old_deadline).state is CausalPairState.PENDING
    pairer.observe_raw(pair_sample(raw, 0.2))
    assert pairer.unavailable_since_sec == pytest.approx(raw)
    assert pairer.observe_safe(pair_sample(safe, 0.2)).state is CausalPairState.VALID
    assert pairer.command_stream_phase is CommandStreamPhase.STREAM_ESTABLISHED


def test_causal_pairer_ambiguous_zero_safe_fails_closed():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.01, 0.0))
    pairer.observe_raw(pair_sample(1.02, 0.2))
    result = pairer.observe_safe(pair_sample(1.03, 0.0))
    assert result.state is CausalPairState.UNKNOWN
    assert result.meaning is None and pairer.pending_within_budget(1.03)


def test_a7i_cross_waypoint_pair_is_never_current_stop():
    pairer = CausalCommandPairer(); pairer.start_epoch(520933.0)
    pairer.observe_raw(pair_sample(520933.60, 0.0))
    assert pairer.observe_safe(pair_sample(520933.699636744, 0.0)).meaning is CausalCollisionMeaning.NO_MOVEMENT
    # The first waypoint-1 raw was observed before its goal UUID was accepted.
    pairer.observe_raw(pair_sample(520933.819298876, 0.07018991559743881,
                                   -0.005333541892468929))
    pairer.start_epoch(520933.83)
    assert pairer.current.state is CausalPairState.NO_PAIR
    delayed = pairer.observe_safe(pair_sample(520933.84, 0.07018991559743881,
                                               -0.005333541892468929))
    assert delayed.state is CausalPairState.VALID and delayed.drain_only
    assert delayed.meaning is CausalCollisionMeaning.CLEAR
    assert pairer.current.state is CausalPairState.NO_PAIR
    pairer.observe_raw(pair_sample(520933.86, 0.08, -0.006))
    current = pairer.observe_safe(pair_sample(520933.87, 0.08, -0.006))
    assert current.state is CausalPairState.VALID and not current.drain_only
    assert current.meaning is CausalCollisionMeaning.CLEAR


def test_identical_clear_and_stop_streams_remain_live_with_observer_loss():
    for safe_value, meaning in ((0.2, CausalCollisionMeaning.CLEAR),
                                (0.0, CausalCollisionMeaning.STOP)):
        pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
        pairer.observe_raw(pair_sample(1.01, 0.2))
        pairer.observe_raw(pair_sample(1.02, 0.2))  # one safe observation was missed
        result = pairer.observe_safe(pair_sample(1.03, safe_value))
        assert result.state is CausalPairState.VALID and result.meaning is meaning
        pairer.observe_raw(pair_sample(1.04, 0.2))
        assert pairer.observe_safe(pair_sample(1.05, safe_value)).state is CausalPairState.VALID


def test_one_dropped_raw_or_safe_does_not_create_false_stop():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.01, 0.10))
    pairer.observe_raw(pair_sample(1.02, 0.15))
    assert pairer.observe_safe(pair_sample(1.03, 0.15)).meaning is CausalCollisionMeaning.CLEAR
    pairer.observe_raw(pair_sample(1.04, 0.18))
    pairer.observe_raw(pair_sample(1.05, 0.19))
    assert pairer.observe_safe(pair_sample(1.06, 0.19)).meaning is CausalCollisionMeaning.CLEAR


def test_history_capacity_covers_every_eligible_sample_at_100_hz():
    pairer = CausalCommandPairer(capacity=32); pairer.start_epoch(1.0)
    for index in range(25):
        pairer.observe_raw(pair_sample(1.0 + index * 0.01, 0.2))
    assert len(pairer.raw_history) == 25
    assert pairer.raw_history[0].steady_receipt_sec == pytest.approx(1.0)


def test_pairer_prunes_oldest_first_and_rejects_stale_or_reverse_order():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.0, 0.2))
    assert pairer.observe_safe(pair_sample(1.251, 0.2)).state is CausalPairState.NO_PAIR
    pairer.observe_raw(pair_sample(2.0, 0.2))
    assert pairer.observe_safe(pair_sample(1.99, 0.2)).state is CausalPairState.NO_PAIR


def test_terminal_reset_never_reuses_collision_pair():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.01, 0.2))
    assert pairer.observe_safe(pair_sample(1.02, 0.0)).meaning is CausalCollisionMeaning.STOP
    pairer.end_epoch(1.03)
    assert pairer.current.state is CausalPairState.NO_PAIR
    drained = pairer.observe_safe(pair_sample(1.04, 0.0))
    assert drained.drain_only or drained.state is not CausalPairState.VALID
    pairer.reset()
    assert not pairer.raw_history and not pairer.safe_history


def test_causal_observer_contains_no_chassis_specific_policy():
    source = (ROOT / "parking_robot_mission_manager" /
              "mission_progress_observation_adapter.py").read_text().lower()
    source = source[source.index("class causalpairstate"):source.index("def _yaw")]
    for forbidden in ("wheelchair", "turning radius", "minimum radius",
                      "reverse prohibition", "legacy radius", "millimetre"):
        assert forbidden not in source


def test_actual_command_callbacks_invalidate_then_publish_only_causal_pair(manager):
    node, _ = manager; node.accept_progress_goal("goal")
    now = [10.01]; node._steady_clock = lambda: now[0]
    raw = Twist(); raw.linear.x = 0.2
    safe = Twist(); safe.linear.x = 0.2
    node._raw_command_cb(raw)
    assert node._causal_raw_command is None and node._causal_safe_command is None
    assert node._command_pairer.pending_within_budget(10.01)
    now[0] = 10.02; node._safe_command_cb(safe)
    assert node._causal_raw_command.linear_x == pytest.approx(0.2)
    assert node._causal_safe_command.linear_x == pytest.approx(0.2)
    now[0] = 10.03; node._raw_command_cb(raw)
    assert node._causal_raw_command is None and node._causal_safe_command is None


@pytest.mark.parametrize("raw,safe", [
    ((0.0, 0.0), (0.0, 0.0)),
    ((0.01, 0.0), (0.0, 0.0)),
    ((math.nextafter(0.01, 0.0), 0.02), (0.0, 0.0)),
    ((0.2, 0.1), (0.2, 0.1)),
    ((0.2, 0.1), (0.16, 0.08)),
    ((0.2, 0.1), (math.nextafter(0.16, math.inf), 0.08)),
    ((0.2, 0.1), (-0.1, -0.05)),
])
def test_semantic_classifier_has_exact_core_parity(raw, safe):
    pairer = CausalCommandPairer()
    raw_record = CommandObservationRecord(1.01, *raw, 1, 1)
    safe_record = CommandObservationRecord(1.02, *safe, -1, 2)
    meaning = pairer._meaning(raw_record, safe_record)
    inputs = SupervisorInputs(None, None, None, pair_sample(1.01, *raw),
                              pair_sample(1.02, *safe), None, None, None, None)
    collision, movement, _ = MissionProgressSupervisorCore()._collision(inputs)
    expected = {
        CausalCollisionMeaning.NO_MOVEMENT: (CollisionClassification.CLEAR, False),
        CausalCollisionMeaning.CLEAR: (CollisionClassification.CLEAR, True),
        CausalCollisionMeaning.SLOWDOWN: (CollisionClassification.SLOWDOWN, True),
        CausalCollisionMeaning.STOP: (CollisionClassification.STOP, True),
    }[meaning]
    assert (collision, movement) == expected


@pytest.mark.parametrize("raw,safe", [
    ((0.0, 0.0), (0.2, 0.0)),
    ((0.2, 0.0), (-0.2, 0.0)),
    ((0.0, 0.2), (0.0, -0.2)),
])
def test_frontier_rejects_transformation_infeasible_assignments(raw, safe):
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.01, *raw))
    assert pairer.observe_safe(pair_sample(1.02, *safe)).state is CausalPairState.PENDING


@pytest.mark.parametrize("raws,safes,expected", [
    (((0.2, 0.0), (0.1, 0.0)), ((0.0, 0.0), (0.1, 0.0)), ("STOP", "CLEAR")),
    (((0.2, 0.0), (0.1, 0.0)), ((0.0, 0.0), (0.0, 0.0)), ("STOP", "STOP")),
    (((0.2, 0.0), (0.15, 0.0)), ((0.2, 0.0), (0.15, 0.0)), ("CLEAR", "CLEAR")),
    (((0.2, 0.0), (0.25, 0.0)), ((0.1, 0.0), (0.0, 0.0)), ("SLOWDOWN", "STOP")),
])
def test_semantic_frontier_preserves_future_pair(raws, safes, expected):
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    for index, value in enumerate(raws):
        pairer.observe_raw(pair_sample(1.01 + index * 0.01, *value))
    first = pairer.observe_safe(pair_sample(1.03, *safes[0]))
    assert pairer.frontier_state_count == 2  # physical identity remains unresolved
    second = pairer.observe_safe(pair_sample(1.04, *safes[1]))
    assert (first.meaning.name, second.meaning.name) == expected
    assert second.state is CausalPairState.VALID


def test_non_sliding_deadline_with_continuous_raw_and_no_safe():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    for index in range(1, 7):
        pairer.observe_raw(pair_sample(1.0 + index * 0.04, 0.2))
        assert pairer.unavailable_since_sec == pytest.approx(1.04)
    assert pairer.adjudicate(1.289999).state is CausalPairState.PENDING
    assert pairer.adjudicate(1.29).state is CausalPairState.STALE


def test_non_sliding_deadline_with_continuous_safe_and_no_raw():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    for index in range(1, 7):
        assert pairer.observe_safe(pair_sample(1.0 + index * 0.04, 0.0)).state is CausalPairState.PENDING
        assert pairer.unavailable_since_sec is None
    assert pairer.adjudicate(9.0).state is CausalPairState.PENDING
    assert pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW


def test_adversarial_32_by_32_frontier_is_bounded():
    pairer = CausalCommandPairer(capacity=32); pairer.start_epoch(1.0)
    for index in range(32):
        pairer.observe_raw(pair_sample(1.001 + index * 0.001, 0.2))
    for index in range(32):
        result = pairer.observe_safe(pair_sample(1.040 + index * 0.001, 0.0))
        assert result.state is CausalPairState.VALID
        assert len(pairer.raw_history) <= 32 and len(pairer.safe_history) <= 32
        assert pairer.frontier_state_count <= 32


def test_ambiguity_does_not_refresh_unavailability_deadline():
    pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
    pairer.observe_raw(pair_sample(1.01, 0.0))
    pairer.observe_raw(pair_sample(1.02, 0.2))
    assert pairer.observe_safe(pair_sample(1.03, 0.0)).state is CausalPairState.AMBIGUOUS
    assert pairer.unavailable_since_sec == pytest.approx(1.01)
    assert pairer.adjudicate(1.259999).state is CausalPairState.AMBIGUOUS
    assert pairer.adjudicate(1.26).state is CausalPairState.STALE


def test_pending_crosses_temporary_boundary_only_next_qualified_stop_emits_threshold():
    core = MissionProgressSupervisorCore()
    core.accept_new_goal(10.0, recovery_baseline=0, distance_remaining_m=5.0,
                         pose_xy=(0.0, 0.0))
    assert core.evaluate(10.0, qualified_inputs(10.0)).primary_event is SupervisorEvent.COLLISION_STOP_PENDING
    # A normal pending acquisition makes no core call at the 1.0 s boundary.
    assert core.evaluate(11.02, qualified_inputs(11.02)).primary_event is SupervisorEvent.COLLISION_STOP_TEMPORARY


def test_pending_crosses_persistent_boundary_only_next_qualified_stop_emits_threshold():
    core = MissionProgressSupervisorCore()
    core.accept_new_goal(10.0, recovery_baseline=0, distance_remaining_m=5.0,
                         pose_xy=(0.0, 0.0))
    core.evaluate(10.0, qualified_inputs(10.0))
    core.evaluate(11.0, qualified_inputs(11.0))
    assert core.evaluate(30.02, qualified_inputs(30.02)).primary_event is SupervisorEvent.PERSISTENT_COLLISION_STOP


def test_stop_pending_clear_never_synthesizes_a_stop_threshold_event():
    core = MissionProgressSupervisorCore()
    core.accept_new_goal(10.0, recovery_baseline=0, distance_remaining_m=5.0,
                         pose_xy=(0.0, 0.0))
    core.evaluate(10.0, qualified_inputs(10.0))
    clear = core.evaluate(11.02, qualified_inputs(11.02, safe=(0.2, 0.0)))
    assert clear.primary_event not in (SupervisorEvent.COLLISION_STOP_TEMPORARY,
                                       SupervisorEvent.PERSISTENT_COLLISION_STOP)
    assert clear.stop_age_sec is None


def test_ambiguity_breaks_old_stop_episode_before_next_stop():
    core = MissionProgressSupervisorCore()
    core.accept_new_goal(10.0, recovery_baseline=0, distance_remaining_m=5.0,
                         pose_xy=(0.0, 0.0))
    core.evaluate(10.0, qualified_inputs(10.0))
    core.evaluate(29.8, qualified_inputs(29.8))
    unavailable = qualified_inputs(29.81)
    unavailable = SupervisorInputs(
        unavailable.feedback, unavailable.odometry, unavailable.transform,
        None, None, unavailable.gate, unavailable.collision_monitor_valid,
        unavailable.localization_valid, unavailable.controller_valid,
        unavailable.adapter_health)
    assert core.evaluate(29.81, unavailable).primary_event is SupervisorEvent.COMMAND_PAIR_STALE
    resumed = core.evaluate(29.82, qualified_inputs(29.82))
    assert resumed.primary_event is SupervisorEvent.COLLISION_STOP_PENDING
    assert resumed.stop_age_sec == 0.0


def test_node_watchdog_source_defers_healthy_pending_but_adjudicates_stale():
    source = NODE.read_text()
    watchdog = source[source.index("def _watchdog_cb"):source.index("def main")]
    assert "self._command_pairer.adjudicate(now)" in watchdog
    assert "pair_state is CausalPairState.STALE" in watchdog
    assert "self._evaluate_progress_passively(now)" in watchdog
    safe_cb = source[source.index("def _safe_command_cb"):source.index("def _gate_state_cb")]
    assert "self._evaluate_progress_passively(now)" in safe_cb


def test_node_calls_policy_on_valid_safe_not_normal_pending_raw(manager):
    node, _ = manager; node.accept_progress_goal("goal")
    now = [10.01]; node._steady_clock = lambda: now[0]
    calls = []; node._evaluate_progress_passively = lambda stamp: calls.append(stamp)
    raw = Twist(); raw.linear.x = 0.2
    safe = Twist(); safe.linear.x = 0.2
    node._raw_command_cb(raw)
    assert calls == []
    now[0] = 10.02; node._safe_command_cb(safe)
    assert calls == [10.02]


def test_node_watchdog_does_not_evaluate_pending_but_evaluates_stale(manager):
    node, _ = manager; node.accept_progress_goal("goal")
    now = [10.01]; node._steady_clock = lambda: now[0]
    calls = []; node._evaluate_progress_passively = lambda stamp: calls.append(stamp)
    node._observe_transform = lambda stamp: None
    node._publish_passive_status = lambda: None
    raw = Twist(); raw.linear.x = 0.2; node._raw_command_cb(raw)
    now[0] = 10.20; node._watchdog_cb(); assert calls == []
    now[0] = 10.259; node._watchdog_cb(); assert calls == []
    now[0] = 10.26; node._watchdog_cb(); assert calls == [10.26]


@pytest.mark.parametrize("age,expected", [(0.999, None), (1.0, "INITIAL_COMMAND_ACQUISITION_TIMEOUT"),
                                           (1.001, "INITIAL_COMMAND_ACQUISITION_TIMEOUT")])
def test_no_raw_timeout_exact_boundary(manager, age, expected):
    node, _ = manager; node.accept_progress_goal("goal")
    event = node._explicit_failure_event(10.0 + age)
    assert (None if event is None else event.name) == expected


def test_raw_processed_before_timeout_adjudication_retires_no_raw_authority(manager):
    node, _ = manager; node.accept_progress_goal("goal")
    node._steady_clock = lambda: 10.999
    raw = Twist(); raw.linear.x = 0.2; node._raw_command_cb(raw)
    assert node._explicit_failure_event(11.001) is None
    assert node._command_pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_RAW_PENDING


def test_timeout_adjudication_before_first_raw_is_fail_closed(manager):
    node, _ = manager; node.accept_progress_goal("goal")
    event = node._explicit_failure_event(11.0)
    assert event.name == "INITIAL_COMMAND_ACQUISITION_TIMEOUT"
    assert node._command_pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW


@pytest.mark.parametrize("activation", [ProgressPolicyActivationState.INITIAL_PRIMING,
                                         ProgressPolicyActivationState.ACTIVE])
def test_no_raw_timeout_reason_applies_during_priming_and_active_waypoint(manager, activation):
    node, _ = manager; node.accept_progress_goal("goal")
    node._progress_policy_activation_state = activation
    now = 11.0
    node._latest_feedback = FeedbackSample(now, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 3.0)
    node._latest_odometry = OdometrySample(now, 0.0, 0.0, 0.0, 0.0, 0.0)
    node._latest_transform = TfSample(now, 0.0, 0.0, 0.0)
    node._latest_gate = GateHealthSample(now, GateState.DISARMED, False)
    node._latest_collision_valid = BoolHealthSample(now, True)
    node._latest_localization_valid = BoolHealthSample(now, True)
    node._latest_controller_valid = BoolHealthSample(now, True)
    node._evaluate_progress_passively(now)
    selected = node._progress_policy_result_to_apply(node._latest_passive_result, now)
    assert selected.primary_event.name == "INITIAL_COMMAND_ACQUISITION_TIMEOUT"
    assert selected.reason == "INITIAL_COMMAND_ACQUISITION_TIMEOUT"


def test_new_waypoint_restarts_no_raw_authority_and_old_tail_is_drain_only(manager):
    node, _ = manager; node.accept_progress_goal("first")
    now = [10.01]; node._steady_clock = lambda: now[0]
    raw = Twist(); raw.linear.x = 0.2; safe = Twist(); safe.linear.x = 0.2
    node._raw_command_cb(raw); now[0] = 10.02; node._safe_command_cb(safe)
    now[0] = 10.03; node._raw_command_cb(raw)
    now[0] = 10.04; node.accept_progress_goal("second")
    assert node._command_pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW
    assert node._command_pairer.unavailable_since_sec is None
    now[0] = 10.05; node._safe_command_cb(safe)
    assert node._command_pairer.current.state is CausalPairState.PENDING
    assert node._command_pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW


def test_acquisition_diagnostics_are_read_only_internal_values(manager):
    node, _ = manager; node.accept_progress_goal("goal")
    diagnostics = node._progress_policy_diagnostics(10.5)
    assert diagnostics["command_stream_phase"] == "ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW"
    assert diagnostics["initial_command_acquisition_timeout_sec"] == "1.000000"
    assert diagnostics["no_raw_acquisition_age_sec"] == "0.500000"
    assert diagnostics["pair_liveness_age_sec"] == "none"
    assert INITIAL_COMMAND_ACQUISITION_TIMEOUT_SEC == 1.0
    assert node._initial_command_acquisition_timeout_sec == 1.0


def test_exact_a8d_boundary_sweeps_60_watchdog_callback_phase_orders():
    epoch = 693987.735263591
    orders = list(permutations(("raw", "safe", "watchdog")))
    cases = 0
    for phase_index in range(10):
        phase = 0.245 + phase_index * 0.002
        for order in orders:
            pairer = CausalCommandPairer(); pairer.start_epoch(epoch)
            now = epoch + phase
            for index, event in enumerate(order):
                receipt = now + index * 0.000001
                if event == "raw":
                    pairer.observe_raw(pair_sample(receipt, 0.2))
                elif event == "safe":
                    pairer.observe_safe(pair_sample(receipt, 0.2))
                else:
                    assert pairer.adjudicate(receipt).state is not CausalPairState.STALE
            if pairer.command_stream_phase is not CommandStreamPhase.STREAM_ESTABLISHED:
                # Safe-before-raw is not back-paired; use the next causal CM output.
                pairer.observe_safe(pair_sample(now + 0.020, 0.2))
            assert pairer.command_stream_phase is CommandStreamPhase.STREAM_ESTABLISHED
            cases += 1
    assert cases == 60


def test_new_one_second_boundary_sweeps_60_callback_watchdog_orders():
    cases = timeouts = transfers = 0
    for offset_index in range(30):
        raw_offset = 0.9995 + offset_index * 0.00002
        for raw_first in (False, True):
            pairer = CausalCommandPairer(); pairer.start_epoch(10.0)
            adjudication = 11.0
            if raw_first:
                pairer.observe_raw(pair_sample(10.0 + raw_offset, 0.2))
                assert pairer.command_stream_phase is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_RAW_PENDING
                transfers += 1
            else:
                assert pairer.no_raw_acquisition_age(adjudication) >= 1.0
                timeouts += 1
            cases += 1
    assert (cases, timeouts, transfers) == (60, 30, 30)


def test_established_stream_sweep_preserves_exact_250ms_boundary():
    true_stale = false_stale = 0
    for age_ms in range(300):
        pairer = CausalCommandPairer(); pairer.start_epoch(1.0)
        pairer.observe_raw(pair_sample(1.01, 0.2)); pairer.observe_safe(pair_sample(1.02, 0.2))
        pairer.observe_raw(pair_sample(2.0, 0.2))
        state = pairer.adjudicate(2.0 + age_ms / 1000.0).state
        if age_ms >= 250:
            true_stale += state is CausalPairState.STALE
        else:
            false_stale += state is CausalPairState.STALE
    assert (true_stale, false_stale) == (50, 0)


@pytest.mark.parametrize("family,event", [
    ("feedback", "FEEDBACK_STALE"),
    ("odometry", "ODOMETRY_STALE"),
    ("tf", "TF_STALE"),
    ("adapter", "ADAPTER_INVALID"),
    ("gate", "GATE_FAULT"),
    ("controller", "CONTROLLER_INVALID"),
    ("localization", "LOCALIZATION_INVALID"),
    ("collision", "COLLISION_MONITOR_INVALID"),
])
def test_positive_health_failures_preempt_no_raw_timeout(manager, family, event):
    node, _ = manager; node.accept_progress_goal("goal")
    now = 11.0
    node._latest_feedback = FeedbackSample(now, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 3.0)
    node._latest_odometry = OdometrySample(now, 0.0, 0.0, 0.0, 0.0, 0.0)
    node._latest_transform = TfSample(now, 0.0, 0.0, 0.0)
    node._latest_gate = GateHealthSample(now, GateState.DISARMED, False)
    node._latest_controller_valid = BoolHealthSample(now, True)
    node._latest_localization_valid = BoolHealthSample(now, True)
    node._latest_collision_valid = BoolHealthSample(now, True)
    if family == "feedback": node._latest_feedback = FeedbackSample(8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 3.0)
    if family == "odometry": node._latest_odometry = OdometrySample(8.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    if family == "tf": node._latest_transform = TfSample(8.0, 0.0, 0.0, 0.0)
    if family == "adapter": node._latest_adapter_health = OptionalAdapterHealthSample(now, False)
    if family == "gate": node._latest_gate = GateHealthSample(now, GateState.FAULT, True)
    if family == "controller": node._latest_controller_valid = BoolHealthSample(now, False)
    if family == "localization": node._latest_localization_valid = BoolHealthSample(now, False)
    if family == "collision": node._latest_collision_valid = BoolHealthSample(now, False)
    assert node._explicit_failure_event(now).name == event


def test_ambiguity_breaks_continuity_once_not_each_watchdog(manager):
    node, _ = manager; node.accept_progress_goal("goal")
    now = [10.01]; node._steady_clock = lambda: now[0]
    calls = []; node._evaluate_progress_passively = lambda stamp: calls.append(stamp)
    zero = Twist(); moving = Twist(); moving.linear.x = 0.2
    node._raw_command_cb(zero); now[0] = 10.02; node._raw_command_cb(moving)
    now[0] = 10.03; node._safe_command_cb(zero)
    assert node._command_pairer.current.state is CausalPairState.AMBIGUOUS
    assert calls == [10.03]
    node._observe_transform = lambda stamp: None
    node._publish_passive_status = lambda: None
    now[0] = 10.10; node._watchdog_cb(); now[0] = 10.20; node._watchdog_cb()
    assert calls == [10.03]


@pytest.mark.parametrize("field,event", [
    ("gate_fault", "GATE_FAULT"),
    ("collision", "COLLISION_MONITOR_INVALID"),
    ("localization", "LOCALIZATION_INVALID"),
    ("controller", "CONTROLLER_INVALID"),
])
def test_explicit_health_bypasses_missing_feedback_during_initial_priming(manager, field, event):
    node, _ = manager
    node.accept_progress_goal("first")
    kwargs = {"feedback_present": False}
    if field == "gate_fault":
        kwargs.update(gate=GateState.FAULT, gate_fault=True)
    else:
        kwargs[field] = False
    policy_evidence(node, 10.0, **kwargs)
    result, apply = evaluate_policy(node, 10.0)
    assert apply and result.primary_event.name == event
    assert node._progress_policy_activation_state is ProgressPolicyActivationState.INITIAL_PRIMING


@pytest.mark.parametrize("offset,expected", [(0.5, False), (1.999, False), (2.0, True)])
def test_prearm_disarmed_is_tolerated_only_before_initial_deadline(manager, offset, expected):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.0 + offset, feedback_present=False, gate=GateState.DISARMED)
    result, apply = evaluate_policy(node, 10.0 + offset)
    assert result.primary_event.name == "GATE_DISARMED"
    assert apply is expected


def test_all_fresh_healthy_armed_activates_mission_immediately_without_adapter(manager):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.4, adapter_valid=None)
    _, apply = evaluate_policy(node, 10.4)
    assert apply
    assert node._progress_policy_activation_state is ProgressPolicyActivationState.ACTIVE
    assert node._progress_policy_activation_start_sec == 10.0
    assert node._progress_policy_activated_sec == 10.4


@pytest.mark.parametrize("kwargs,event", [
    ({"gate": GateState.DISARMED}, "GATE_DISARMED"),
    ({"gate": GateState.FAULT, "gate_fault": True}, "GATE_FAULT"),
    ({"collision": False}, "COLLISION_MONITOR_INVALID"),
])
def test_health_is_immediate_after_active(manager, kwargs, event):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.2)
    evaluate_policy(node, 10.2)
    policy_evidence(node, 10.3, **kwargs)
    result, apply = evaluate_policy(node, 10.3)
    assert apply and result.primary_event.name == event


def test_second_waypoint_keeps_activation_latch_and_resets_only_progress(manager):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.2)
    evaluate_policy(node, 10.2)
    start = node._progress_policy_activation_start_sec
    activated = node._progress_policy_activated_sec
    reset_count = node._progress_goal_reset_count
    baseline_identity = id(node._progress_supervisor)
    node._steady_clock = lambda: 10.3
    node.accept_progress_goal("second")
    assert node._progress_policy_activation_state is ProgressPolicyActivationState.ACTIVE
    assert node._progress_policy_activation_start_sec == start
    assert node._progress_policy_activated_sec == activated
    assert node._progress_goal_reset_count == reset_count + 1
    assert id(node._progress_supervisor) == baseline_identity
    assert node._latest_feedback is None


@pytest.mark.parametrize("kwargs,event", [
    ({"gate": GateState.FAULT, "gate_fault": True}, "GATE_FAULT"),
    ({"collision": False}, "COLLISION_MONITOR_INVALID"),
    ({"localization": False}, "LOCALIZATION_INVALID"),
    ({"controller": False}, "CONTROLLER_INVALID"),
])
def test_second_waypoint_missing_feedback_has_no_new_health_grace(manager, kwargs, event):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.2)
    evaluate_policy(node, 10.2)
    node._steady_clock = lambda: 10.3
    node.accept_progress_goal("second")
    policy_evidence(node, 10.3, feedback_present=False, **kwargs)
    result, apply = evaluate_policy(node, 10.3)
    assert apply and result.primary_event.name == event


def test_second_waypoint_missing_feedback_alone_is_immediate_stale(manager):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.2)
    evaluate_policy(node, 10.2)
    node._steady_clock = lambda: 10.3
    node.accept_progress_goal("second")
    policy_evidence(node, 10.3, feedback_present=False)
    result, apply = evaluate_policy(node, 10.3)
    assert apply and result.primary_event.name == "FEEDBACK_STALE"


def test_optional_adapter_invalid_is_immediate_during_priming(manager):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.0, feedback_present=False, adapter_valid=False)
    result, apply = evaluate_policy(node, 10.0)
    assert apply and result.primary_event.name == "ADAPTER_INVALID"


def test_explicit_collision_failure_bypasses_unrelated_missing_gate(manager):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.0, collision=False)
    node._latest_gate = None
    result, apply = evaluate_policy(node, 10.0)
    assert apply and result.primary_event.name == "COLLISION_MONITOR_INVALID"


@pytest.mark.parametrize("stale,event", [
    ("feedback", "FEEDBACK_STALE"), ("odometry", "ODOMETRY_STALE"),
    ("tf", "TF_STALE"), ("command", "COMMAND_PAIR_STALE"),
])
def test_present_stale_evidence_is_immediate_during_priming(manager, stale, event):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.0, stale=stale)
    result, apply = evaluate_policy(node, 10.0)
    assert apply and result.primary_event.name == event


@pytest.mark.parametrize("offset,expected", [(1.999, False), (2.0, True)])
def test_missing_feedback_only_uses_exact_initial_deadline_boundary(manager, offset, expected):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.0 + offset, feedback_present=False)
    result, apply = evaluate_policy(node, 10.0 + offset)
    assert result.primary_event.name == "FEEDBACK_STALE"
    assert apply is expected


def test_active_survives_waypoint_and_temporary_nonterminal_snapshots(manager):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.2)
    evaluate_policy(node, 10.2)
    for state in (MissionStateCode.TEMPORARILY_BLOCKED, MissionStateCode.NAVIGATING,
                  MissionStateCode.PLANNING):
        node._publish_snapshot(MissionSnapshot(state=state))
        assert node._progress_policy_activation_state is ProgressPolicyActivationState.ACTIVE


@pytest.mark.parametrize("terminal", [
    MissionStateCode.SUCCEEDED, MissionStateCode.CANCELLED,
    MissionStateCode.BLOCKED, MissionStateCode.FAILED,
])
def test_terminal_resets_activation_for_next_mission(manager, terminal):
    node, _ = manager
    node.accept_progress_goal("first")
    policy_evidence(node, 10.2)
    evaluate_policy(node, 10.2)
    node._publish_snapshot(MissionSnapshot(state=terminal))
    assert node._progress_policy_activation_state is ProgressPolicyActivationState.NOT_STARTED
    assert node._progress_policy_activation_start_sec is None
    node._steady_clock = lambda: 10.3
    node.accept_progress_goal("next-mission")
    assert node._progress_policy_activation_state is ProgressPolicyActivationState.INITIAL_PRIMING


def test_preactivation_pause_resume_keeps_original_initial_deadline(manager):
    node, _ = manager
    node.accept_progress_goal("before-pause")
    start = node._progress_policy_activation_start_sec
    node.finish_progress_goal("before-pause")
    assert node._progress_policy_activation_state is ProgressPolicyActivationState.INITIAL_PRIMING
    node.accept_progress_goal("after-resume")
    assert node._progress_policy_activation_start_sec == start
    policy_evidence(node, 12.0, feedback_present=False, gate=GateState.DISARMED)
    result, apply = evaluate_policy(node, 12.0)
    assert apply and result.primary_event.name == "GATE_DISARMED"


def test_goal_rejection_without_accepted_uuid_does_not_start_activation(manager):
    node, _ = manager
    node.mission_state_machine.on_goal_rejected()
    assert node._progress_policy_activation_state is ProgressPolicyActivationState.NOT_STARTED
