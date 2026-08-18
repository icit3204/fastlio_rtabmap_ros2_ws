"""Bounded P4-E.5B Mission Manager temporary-block runtime evidence runner."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticStatus
from nav2_msgs.action import NavigateToPose
from parking_robot_interfaces.msg import MissionState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy

from .phase4_p4e1b_clear_runner import LIFECYCLE, TOPICS, nz, rates
from .phase4_p4e2a_slowdown_runner import pair_commands
from .phase4_p4e3a_mission_cancel_runner import MissionCancelRunner


def command_mag(row):
    return math.hypot(float(row[2]), float(row[7]))


def first_pair(pairs, predicate):
    for raw, safe, skew in pairs:
        if predicate(raw, safe):
            return {"raw_ns": raw[0], "safe_ns": safe[0], "reference_ns": max(raw[0], safe[0]),
                    "skew_ns": skew, "raw_linear": raw[2], "safe_linear": safe[2]}
    return None


class P4E5BRunner(MissionCancelRunner):
    def __init__(self, out: Path):
        super().__init__(out)
        self.set_mode_client = self.create_client(
            SetParameters, "/phase4_p4b_synthetic_obstacles/set_parameters")
        self.collision_parameters = self.create_client(
            GetParameters, "/collision_monitor/get_parameters")
        self.mission_diagnostics = []
        self.block_diagnostics = []
        self.feedback = []
        self.create_subscription(DiagnosticStatus, "/mission/status", self.mission_status, 100)
        self.create_subscription(DiagnosticStatus, "/mission/block_reason", self.block_status, 100)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage,
                                 "/navigate_to_pose/_action/feedback", self.nav_feedback, 100)
        for name in ("mission_policy_diagnostics.jsonl", "mission_block_reason.jsonl",
                     "navigate_feedback.jsonl", "obstacle_mode_events.jsonl"):
            (out / name).write_text("", encoding="utf-8")

    def _diagnostic_item(self, msg):
        mono, ros = self.now()
        raw_level = msg.level
        level = (raw_level[0] if isinstance(raw_level, (bytes, bytearray, memoryview))
                 else int(raw_level))
        return {"monotonic_ns": mono, "ros_ns": ros, "name": msg.name,
                "level": level, "message": msg.message,
                "values": {v.key: v.value for v in msg.values}}

    def mission_status(self, msg):
        item = self._diagnostic_item(msg)
        self.mission_diagnostics.append(item)
        self.emit("mission_policy_diagnostics.jsonl", item)

    def block_status(self, msg):
        item = self._diagnostic_item(msg)
        self.block_diagnostics.append(item)
        self.emit("mission_block_reason.jsonl", item)

    def nav_feedback(self, msg):
        mono, ros = self.now()
        item = {"monotonic_ns": mono, "ros_ns": ros,
                "goal_uuid": bytes(msg.goal_id.uuid).hex(),
                "distance_remaining": float(msg.feedback.distance_remaining),
                "number_of_recoveries": int(msg.feedback.number_of_recoveries)}
        self.feedback.append(item)
        self.emit("navigate_feedback.jsonl", item)

    def mode(self, value):
        if value not in ("CLEAR", "STOP"):
            raise RuntimeError("P4-E.5B permits only CLEAR and STOP")
        request = self.emit("obstacle_mode_events.jsonl",
                            {"event": "mode_request", "mode": value})
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("synthetic obstacle parameter service unavailable")
        parameter = Parameter(name="mode", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=value))
        future = self.set_mode_client.call_async(SetParameters.Request(parameters=[parameter]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if (not future.done() or future.result() is None
                or not future.result().results[0].successful):
            raise RuntimeError(f"mode change failed: {value}")
        return self.emit("obstacle_mode_events.jsonl",
                         {"event": "mode_confirmation", "mode": value,
                          "request_ns": request["monotonic_ns"]})

    def latest_policy(self, state=None, after_ns=0, uuid=None):
        rows = []
        for item in self.mission_diagnostics:
            if item["monotonic_ns"] < after_ns:
                continue
            values = item["values"]
            if state is not None and values.get("progress_policy_activation_state") != state:
                continue
            if uuid is not None and values.get("active_goal_uuid") != uuid:
                continue
            rows.append(item)
        return rows[-1] if rows else None

    def wait_for(self, predicate, timeout, reason):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            result = predicate()
            if result:
                return result
        raise RuntimeError(reason)

    def collision_stop_pub_timeout(self):
        if not self.collision_parameters.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Collision Monitor parameter service unavailable")
        future = self.collision_parameters.call_async(
            GetParameters.Request(names=["stop_pub_timeout"]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None or not future.result().values:
            raise RuntimeError("stop_pub_timeout query failed")
        return float(future.result().values[0].double_value)

    def wait_fresh_safe_disarmed_since(self, accepted_ns, timeout_sec=1.6):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            raw = [x for x in self.samples["/cmd_vel_nav_raw"]
                   if x[0] >= accepted_ns and nz(x[2:8])]
            safe = [x for x in self.samples["/cmd_vel_nav_safe"]
                    if x[0] >= accepted_ns and nz(x[2:8])]
            if len(raw) < 3 or len(safe) < 3:
                continue
            now_ns = time.monotonic_ns()
            if now_ns - safe[-1][0] > 250_000_000 or now_ns - safe[0][0] < 500_000_000:
                continue
            gate = self.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
            if gate is None:
                continue
            if gate[1].get("state") != "DISARMED" or gate[1].get("fault_latched") != "false":
                raise RuntimeError(f"gate not healthy DISARMED: {gate[1]}")
            if any(nz(x[2:8]) for x in self.samples["/vehicle_cmd_safe"]
                   if x[0] >= accepted_ns):
                raise RuntimeError("gate bypass while DISARMED")
            if any(nz(x[2]) for x in self.samples["/wheelchair_control_command_mock"]
                   if x[0] >= accepted_ns):
                raise RuntimeError("adapter bypass while DISARMED")
            if any(not self.permissions.get(t, (False, 0))[0]
                   or now_ns - self.permissions[t][1] > 500_000_000 for t in TOPICS[2:5]):
                raise RuntimeError("permission freshness lost")
            skew = abs(safe[-1][0] - raw[-1][0])
            numeric = max(abs(a - b) for a, b in zip(safe[-1][2:8], raw[-1][2:8]))
            if skew > 75_000_000 or numeric > .05:
                raise RuntimeError(f"raw/safe pairing failed: {skew},{numeric}")
            return self.emit("scenario_events.jsonl", {
                "event": "P4E5B_GOAL_ACTIVE_DISARMED_INTERLOCK_PASS",
                "accepted_ns": accepted_ns, "safe_sample_count": len(safe),
                "raw_sample_count": len(raw), "newest_safe_age_ns": now_ns - safe[-1][0],
                "raw_safe_skew_ns": skew, "raw_safe_max_error": numeric,
                "goal_active_disarmed_duration_ns": now_ns - accepted_ns})
        raise RuntimeError("fresh safe-command precondition timeout")


def state_after(node, state, after_ns, reason=None):
    rows = [x for x in node.states if x["monotonic_ns"] >= after_ns and x["state"] == state]
    if reason is not None:
        rows = [x for x in rows if x["reason_code"] == reason]
    return rows[0] if rows else None


def health_ok(node, now_ns):
    gate = node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
    permissions = all(node.permissions.get(t, (False, 0))[0]
                      and now_ns - node.permissions[t][1] <= 500_000_000 for t in TOPICS[2:5])
    return bool(gate and gate[1].get("state") == "ARMED"
                and gate[1].get("fault_latched") == "false" and permissions)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    ns = parser.parse_args(args)
    out = Path(ns.output_dir); out.mkdir(parents=True, exist_ok=True)
    metrics = {}; error = None
    rclpy.init(); node = P4E5BRunner(out)
    try:
        (out / "process_environment.tsv").write_text(
            "key\tvalue\n" + "".join(f"{k}\t{v}\n" for k, v in sorted(os.environ.items())
                                      if k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION")),
            encoding="utf-8")

        node.spin(5.0); node.graph(); lifecycle = node.lifecycle()
        if any(lifecycle.get(name, (0, ""))[1] != "active" for name in LIFECYCLE):
            raise RuntimeError(f"inactive lifecycle: {lifecycle}")
        names = set(node.get_node_names())
        forbidden = {name for name in names if any(token in name.lower() for token in
                    ("plan_nav", "rtab", "fast_lio", "pure_pursuit", "wheelchair_controller"))}
        if forbidden or sum(name == "mission_manager" for name in names) != 1:
            raise RuntimeError(f"runtime authority nodes={names}, forbidden={forbidden}")
        publishers = {topic: len(node.get_publishers_info_by_topic(topic)) for topic in TOPICS}
        if any(publishers[topic] != 1 for topic in TOPICS[1:]):
            raise RuntimeError(f"publisher authority: {publishers}")
        raw_nodes = {ep.node_name for ep in node.get_publishers_info_by_topic("/cmd_vel_nav_raw")}
        if not raw_nodes or not raw_nodes <= {"controller_server", "behavior_server"}:
            raise RuntimeError(f"raw authority: {raw_nodes}")
        gate = node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
        if (gate is None or gate[1].get("state") != "DISARMED"
                or gate[1].get("fault_latched") != "false"):
            raise RuntimeError(f"initial gate state: {gate}")
        ready_start = time.monotonic_ns(); ready_pose = node.latest_odom; node.spin(2.2)
        ready_end = time.monotonic_ns()
        ready_rates = {"gate": rates(node.samples["/vehicle_cmd_safe"], ready_start, ready_end),
                       "mock": rates(node.samples["/wheelchair_control_command_mock"], ready_start, ready_end),
                       "odom": rates(node.samples["/Odometry"], ready_start, ready_end),
                       "tf": rates(node.tf_samples["odom->base_footprint"], ready_start, ready_end)}
        if not (18 <= ready_rates["gate"] <= 22 and 18 <= ready_rates["mock"] <= 22
                and 48 <= ready_rates["odom"] <= 52 and 48 <= ready_rates["tf"] <= 52):
            raise RuntimeError(f"readiness rates: {ready_rates}")
        if any(nz(x[2:8]) for x in node.samples["/vehicle_cmd_safe"] if ready_start <= x[0] <= ready_end):
            raise RuntimeError("nonzero disarmed readiness output")
        if (ready_pose is None or math.hypot(node.latest_odom[0] - ready_pose[0],
                                             node.latest_odom[1] - ready_pose[1]) > .002):
            raise RuntimeError("readiness odometry drift")
        now_ns = time.monotonic_ns()
        if any(not node.permissions.get(t, (False, 0))[0]
               or now_ns - node.permissions[t][1] > 500_000_000 for t in TOPICS[2:5]):
            raise RuntimeError("readiness validity not fresh true")
        stop_pub_timeout = node.collision_stop_pub_timeout()
        if stop_pub_timeout != 30.0:
            raise RuntimeError(f"stop_pub_timeout={stop_pub_timeout}")
        readiness = node.emit("scenario_events.jsonl", {"event": "P4E5B_CHAIN_READINESS_PASS",
                              "rates_hz": ready_rates, "lifecycle": lifecycle,
                              "publisher_counts": publishers, "raw_nodes": sorted(raw_nodes),
                              "stop_pub_timeout_sec": stop_pub_timeout})

        node.publish_initial(); node.spin(.5); route = node.publish_route()
        node.wait_for(lambda: state_after(node, MissionState.RECEIVED, route["monotonic_ns"]),
                      5.0, "RECEIVED timeout")
        start_request, start_response = node.trigger("start")
        nav0 = node.wait_for(lambda: next((x for x in node.states
            if x["monotonic_ns"] >= start_request["monotonic_ns"]
            and x["state"] == MissionState.NAVIGATING and x["active_goal_uuid"]), None),
            10.0, "waypoint-0 NAVIGATING UUID timeout")
        uuid0 = nav0["active_goal_uuid"]; accepted_ns = nav0["monotonic_ns"]
        priming = node.wait_for(lambda: node.latest_policy("INITIAL_PRIMING", accepted_ns, uuid0),
                                1.0, "INITIAL_PRIMING diagnostic timeout")
        gate = node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
        if gate[1].get("state") != "DISARMED" or gate[1].get("fault_latched") != "false":
            raise RuntimeError("pre-arm gate is not healthy DISARMED")
        if any(x["state"] in (MissionState.CANCELLING, MissionState.FAILED,
                              MissionState.BLOCKED, MissionState.CANCELLED)
               for x in node.states if x["monotonic_ns"] >= accepted_ns):
            raise RuntimeError("termination occurred during initial priming")
        interlock = node.wait_fresh_safe_disarmed_since(accepted_ns, timeout_sec=1.6)
        arm_request_ns = time.monotonic_ns(); arm_response = node.arm_gate()
        active = node.wait_for(lambda: node.latest_policy("ACTIVE", arm_request_ns, uuid0),
                               1.0, "ACTIVE diagnostic timeout")
        active_ns = active["monotonic_ns"]
        activation_start_ns = int(float(active["values"]["progress_policy_activation_start_sec"]) * 1e9)
        activated_ns = int(float(active["values"].get("progress_policy_activation_age_sec", "0")) * 1e9) + activation_start_ns
        if active_ns - activation_start_ns >= 2_000_000_000:
            raise RuntimeError("initial activation missed 2.0-second deadline")
        activation = node.emit("scenario_events.jsonl", {
            "event": "P4E5B_INITIAL_MISSION_ACTIVATION_PASS",
            "goal_uuid": uuid0, "activation_start_ns": activation_start_ns,
            "arm_request_ns": arm_request_ns, "arm_response_ns": arm_response["monotonic_ns"],
            "active_observation_ns": active_ns, "activated_ns": activated_ns,
            "activation_delta_ns": active_ns - activation_start_ns})

        clear_start = time.monotonic_ns(); node.spin(2.1); clear_end = time.monotonic_ns()
        if node.states[-1]["state"] != MissionState.NAVIGATING or not health_ok(node, clear_end):
            raise RuntimeError("unstable pre-block CLEAR")
        clear_pairs = pair_commands(node.samples["/cmd_vel_nav_raw"], node.samples["/cmd_vel_nav_safe"],
                                    clear_start, clear_end)
        if len([1 for raw, safe, _ in clear_pairs if command_mag(raw) > .01 and command_mag(safe) > .01]) < 30:
            raise RuntimeError("insufficient moving CLEAR pairs")
        if node.latest_policy("ACTIVE", clear_start, uuid0) is None:
            raise RuntimeError("activation latch lost during CLEAR")
        pre_clear = node.emit("scenario_events.jsonl", {"event": "P4E5B_PRE_BLOCK_CLEAR_PASS",
                              "start_ns": clear_start, "end_ns": clear_end,
                              "duration_ns": clear_end - clear_start, "goal_uuid": uuid0})

        stop = node.mode("STOP"); stop_confirmation = stop["monotonic_ns"]
        temp = node.wait_for(lambda: state_after(node, MissionState.TEMPORARILY_BLOCKED,
                                                 stop_confirmation, "COLLISION_STOP_TEMPORARY"),
                             3.0, "TEMPORARILY_BLOCKED timeout")
        temp_ns = temp["monotonic_ns"]
        stop_pairs = pair_commands(node.samples["/cmd_vel_nav_raw"], node.samples["/cmd_vel_nav_safe"],
                                   stop_confirmation, temp_ns)
        qualifying_stop = first_pair(stop_pairs, lambda raw, safe:
                                     command_mag(raw) > .01 and command_mag(safe) < .01)
        if qualifying_stop is None:
            raise RuntimeError("no qualifying STOP pair")
        stop_delta = temp_ns - qualifying_stop["reference_ns"]
        if not 1_000_000_000 <= stop_delta <= 1_500_000_000:
            raise RuntimeError(f"temporary block entry delta {stop_delta}")
        if temp["active_goal_uuid"] != uuid0 or not health_ok(node, temp_ns):
            raise RuntimeError("UUID/health changed at temporary block")
        block = node.wait_for(lambda: next((x for x in node.block_diagnostics
            if x["monotonic_ns"] >= temp_ns and x["message"] == "COLLISION_STOP_TEMPORARY"), None),
            .5, "block_reason timeout")
        entry = node.emit("scenario_events.jsonl", {"event": "P4E5B_TEMPORARILY_BLOCKED_ENTRY_PASS",
                          "qualifying_stop": qualifying_stop, "temporarily_blocked_ns": temp_ns,
                          "entry_delta_ns": stop_delta, "block_reason_ns": block["monotonic_ns"],
                          "goal_uuid": uuid0})

        hold_pose = node.latest_odom; hold_start = time.monotonic_ns(); node.spin(.70); hold_end = time.monotonic_ns()
        hold_translation = math.hypot(node.latest_odom[0] - hold_pose[0], node.latest_odom[1] - hold_pose[1])
        if (node.states[-1]["state"] != MissionState.TEMPORARILY_BLOCKED
                or hold_translation > .02 or not health_ok(node, hold_end)):
            raise RuntimeError("bounded blocked hold failed")
        hold = node.emit("scenario_events.jsonl", {"event": "P4E5B_BOUNDED_BLOCK_HOLD_PASS",
                         "start_ns": hold_start, "end_ns": hold_end,
                         "duration_ns": hold_end - hold_start, "translation_m": hold_translation})

        clear = node.mode("CLEAR"); clear_confirmation = clear["monotonic_ns"]
        recovered = node.wait_for(lambda: state_after(node, MissionState.NAVIGATING,
            clear_confirmation, "MISSION_PROGRESS_RESUMED"), 2.0, "NAVIGATING recovery timeout")
        recovery_ns = recovered["monotonic_ns"]
        recovery_pairs = pair_commands(node.samples["/cmd_vel_nav_raw"], node.samples["/cmd_vel_nav_safe"],
                                       clear_confirmation, recovery_ns)
        qualifying_clear = first_pair(recovery_pairs, lambda raw, safe:
                                      command_mag(raw) > .01 and command_mag(safe) > .01)
        if qualifying_clear is None:
            raise RuntimeError("no qualifying CLEAR pair")
        recovery_delta = recovery_ns - qualifying_clear["reference_ns"]
        if not 500_000_000 <= recovery_delta <= 1_000_000_000:
            raise RuntimeError(f"temporary recovery delta {recovery_delta}")
        if recovered["active_goal_uuid"] != uuid0 or node.latest_policy("ACTIVE", clear_confirmation, uuid0) is None:
            raise RuntimeError("same-goal ACTIVE latch lost on recovery")
        recovery = node.emit("scenario_events.jsonl", {"event": "P4E5B_TEMPORARY_BLOCK_RECOVERY_PASS",
                             "qualifying_clear": qualifying_clear, "recovery_ns": recovery_ns,
                             "recovery_delta_ns": recovery_delta, "goal_uuid": uuid0})
        motion = node.wait_for(lambda: next((x for x in node.samples["/Odometry"]
            if x[0] >= recovery_ns and (abs(x[5]) > .01 or abs(x[6]) > .02)), None),
            1.0, "resumed odometry timeout")
        same_goal = node.emit("scenario_events.jsonl", {"event": "P4E5B_SAME_GOAL_RESUME_PASS",
                               "goal_uuid": uuid0, "resumed_motion_ns": motion[0]})

        wp1 = node.wait_for(lambda: next((x for x in node.states
            if x["monotonic_ns"] > recovery_ns and x["state"] == MissionState.NAVIGATING
            and x["waypoint_index"] == 1 and x["active_goal_uuid"]
            and x["active_goal_uuid"] != uuid0), None), 90.0, "waypoint-1 acceptance timeout")
        uuid1 = wp1["active_goal_uuid"]; wp1_ns = wp1["monotonic_ns"]
        wp1_policy = node.wait_for(lambda: node.latest_policy("ACTIVE", wp1_ns, uuid1),
                                   2.0, "waypoint-1 ACTIVE diagnostic timeout")
        if wp1_policy["values"]["progress_policy_activation_start_sec"] != active["values"]["progress_policy_activation_start_sec"]:
            raise RuntimeError("activation start reset on waypoint-1")
        if any(x["values"].get("progress_policy_activation_state") == "INITIAL_PRIMING"
               for x in node.mission_diagnostics if x["monotonic_ns"] >= wp1_ns):
            raise RuntimeError("waypoint-1 re-entered INITIAL_PRIMING")
        feedback1 = node.wait_for(lambda: next((x for x in node.feedback
            if x["monotonic_ns"] >= wp1_ns and x["goal_uuid"] == uuid1), None),
            2.0, "waypoint-1 feedback ownership timeout")
        latch = node.emit("scenario_events.jsonl", {"event": "P4E5B_SEQUENTIAL_WAYPOINT_LATCH_PASS",
                          "waypoint0_uuid": uuid0, "waypoint1_uuid": uuid1,
                          "waypoint1_acceptance_ns": wp1_ns,
                          "activation_start_sec": wp1_policy["values"]["progress_policy_activation_start_sec"],
                          "initial_active_observation_ns": active_ns,
                          "source_correlated_activated_ns": activated_ns,
                          "waypoint1_feedback_ns": feedback1["monotonic_ns"],
                          "source_correlated_progress_reset_count": 2})

        final = node.wait_for(lambda: state_after(node, MissionState.SUCCEEDED, wp1_ns),
                              90.0, "mission SUCCEEDED timeout")
        if final["completed"] != 2 or len({uuid0, uuid1}) != 2:
            raise RuntimeError("final mission counters/UUIDs invalid")
        if any(x["state"] in (MissionState.CANCELLING, MissionState.BLOCKED,
                              MissionState.FAILED, MissionState.CANCELLED) for x in node.states):
            raise RuntimeError("forbidden terminal/cancellation state observed")
        terminal = node.emit("scenario_events.jsonl", {"event": "P4E5B_TERMINAL_ACTIVATION_RESET_PASS",
                             "mission_succeeded_ns": final["monotonic_ns"],
                             "activation_reset": "NOT_STARTED_SOURCE_AND_DETERMINISTIC_TEST_CORRELATED",
                             "completed_waypoints": final["completed"]})
        node.spin(.5); end_ns = time.monotonic_ns(); node.graph(); node.lifecycle()
        all_rates = {topic: rates(node.samples[topic], accepted_ns, end_ns) for topic in TOPICS}
        all_rates.update({"tf": rates(node.tf_samples["odom->base_footprint"], accepted_ns, end_ns),
                          "feedback": rates([(x["monotonic_ns"],) for x in node.feedback], accepted_ns, end_ns),
                          "mission_status": rates([(x["monotonic_ns"],) for x in node.mission_diagnostics], accepted_ns, end_ns),
                          "adapter_diagnostics": rates([(x["monotonic_ns"],) for x in node.diagnostic_history["adapter"]], accepted_ns, end_ns)})
        statuses = ["P4E5B_CHAIN_READINESS_PASS", "P4E5B_INITIAL_MISSION_ACTIVATION_PASS",
                    "P4E5B_PRE_BLOCK_CLEAR_PASS", "P4E5B_TEMPORARILY_BLOCKED_ENTRY_PASS",
                    "P4E5B_TEMPORARY_BLOCK_RECOVERY_PASS", "P4E5B_SAME_GOAL_RESUME_PASS",
                    "P4E5B_SEQUENTIAL_WAYPOINT_LATCH_PASS", "P4E5B_TERMINAL_ACTIVATION_RESET_PASS"]
        metrics = {"pass": True, "status": "PHASE4_P4E5B_TEMPORARY_BLOCK_RUNTIME_COMPLETED_NEEDS_REVIEW",
                   "statuses": statuses, "route_count": node.route_count, "start_count": node.start_count,
                   "arm_count": node.arm_request_count, "cancel_count": node.cancel_count,
                   "goal_uuids": [uuid0, uuid1], "goal_uuid_count": 2,
                   "final_state": "SUCCEEDED", "completed_waypoints": final["completed"],
                   "readiness": readiness, "route": route, "start_request": start_request,
                   "start_response": start_response, "priming": priming, "interlock": interlock,
                   "activation": activation, "pre_block_clear": pre_clear, "entry": entry,
                   "hold": hold, "recovery": recovery, "same_goal": same_goal,
                   "sequential_latch": latch, "terminal_reset": terminal,
                   "rates_hz": all_rates,
                   "timing_ns": {"activation_start_to_arm": arm_request_ns - activation_start_ns,
                                 "activation_start_to_active": active_ns - activation_start_ns,
                                 "qualifying_stop_to_temporary": stop_delta,
                                 "temporary_to_clear_request": clear["request_ns"] - temp_ns,
                                 "qualifying_clear_to_navigating": recovery_delta,
                                 "navigating_to_resumed_motion": motion[0] - recovery_ns,
                                 "waypoint0_success_to_waypoint1_acceptance": wp1_ns - next(x["monotonic_ns"] for x in node.action_statuses if x["goal_uuid"] == uuid0 and x["status"] == GoalStatus.STATUS_SUCCEEDED)},
                   "exclusions": {"PERSISTENT_COLLISION_STOP": "NOT TESTED", "BLOCK_TERMINATION": "NOT TESTED",
                                  "CANCELLING_TO_BLOCKED": "NOT RUNTIME QUALIFIED HERE",
                                  "HEALTH_FAILURE_TERMINATION": "NOT TESTED", "20_SECOND_STOP": "NOT RUN"}}
    except Exception as exc:
        error = repr(exc)
        metrics = {"pass": False, "status": "PHASE4_P4E5B_TEMPORARY_BLOCK_RUNTIME_NEEDS_REVIEW",
                   "error": error}
    finally:
        (out / "terminal_metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n",
                                                   encoding="utf-8")
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)


if __name__ == "__main__":
    main()
