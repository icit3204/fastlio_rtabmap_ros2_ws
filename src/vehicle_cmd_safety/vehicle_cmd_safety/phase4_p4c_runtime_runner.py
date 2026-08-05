"""Runtime orchestrator and evidence collector for Phase 4 P4-C scenarios."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


FORBIDDEN_TOPICS = (
    "/wheelchair_control_command_mock",
    "/wheelchair_control_command",
    "/wheelchair_control_command_raw",
    "/cmd_vel_nav_raw",
)

AUTHORITY_TOPICS = (
    "/cmd_vel_nav_safe",
    "/vehicle_cmd_safe",
    "/system/localization_valid",
    "/system/controller_valid",
    "/system/collision_monitor_valid",
    *FORBIDDEN_TOPICS,
)

PROCESS_PATTERNS = (
    "guarded_vehicle_cmd_gate",
    "collision_monitor_validity_monitor",
    "collision_monitor",
    "phase4_p4c",
    "phase4_p4b",
    "wheelchair",
    "pure_pursuit",
    "laser_command_safety_filter",
    "velocity_smoother",
    "UdpSender",
    "SocketCAN",
    "can0",
    "vcan",
)

ROGUE_OUTPUT_FRAME_ID = "phase4_p4c_duplicate_output_rogue_zero"
ROGUE_OUTPUT_RATE_HZ = 20.0
VALIDITY_DIAGNOSTIC_NAME = "vehicle_cmd_safety/collision_monitor_validity_monitor"
EXPECTED_VALIDITY_NODE = "/collision_monitor_validity_monitor"


@dataclass
class ValidityReadinessSnapshot:
    monitor_process_alive: bool = False
    validity_node_discovered: bool = False
    intended_validity_publishers: int = 0
    unintended_validity_publishers: int = 0
    evidence_subscriber_active: bool = False
    consecutive_validity_samples: int = 0
    matching_validity_diagnostics: int = 0
    monitor_heartbeat_alive: bool = False
    unexpected_process_exit: bool = False
    lifecycle_service_discovered: bool = False
    parameter_service_discovered: bool = False
    lifecycle_active: bool = False
    source_configuration_ok: bool = False
    synthetic_observation_publisher_exists: bool = False


def validity_readiness_missing(snapshot: ValidityReadinessSnapshot, *, positive_collision_monitor: bool) -> list[str]:
    missing: list[str] = []
    if not snapshot.monitor_process_alive:
        missing.append("validity-monitor process alive")
    if not snapshot.validity_node_discovered:
        missing.append("expected validity node discovered")
    if snapshot.intended_validity_publishers != 1 or snapshot.unintended_validity_publishers != 0:
        missing.append("exactly one intended publisher on /system/collision_monitor_valid")
    if not snapshot.evidence_subscriber_active:
        missing.append("evidence subscriber active")
    if snapshot.consecutive_validity_samples < 3:
        missing.append("at least three consecutive validity samples")
    if snapshot.matching_validity_diagnostics < 1:
        missing.append("at least one matching validity DiagnosticStatus")
    if not snapshot.monitor_heartbeat_alive:
        missing.append("monitor heartbeat alive")
    if snapshot.unexpected_process_exit:
        missing.append("no launch process exited unexpectedly")
    if positive_collision_monitor:
        if not snapshot.lifecycle_service_discovered:
            missing.append("/collision_monitor/get_state discovered")
        if not snapshot.parameter_service_discovered:
            missing.append("/collision_monitor/get_parameters discovered")
        if not snapshot.lifecycle_active:
            missing.append("Collision Monitor lifecycle ACTIVE")
        if not snapshot.source_configuration_ok:
            missing.append("expected source configuration query succeeds")
        if not snapshot.synthetic_observation_publisher_exists:
            missing.append("selected synthetic observation publisher exists")
    return missing


def validity_readiness_ready(snapshot: ValidityReadinessSnapshot, *, positive_collision_monitor: bool) -> bool:
    return not validity_readiness_missing(snapshot, positive_collision_monitor=positive_collision_monitor)


def scenario_elapsed_after_readiness(readiness_ns: int, event_ns: int) -> float:
    if event_ns < readiness_ns:
        raise ValueError("scenario assertion clock began before readiness")
    return (event_ns - readiness_ns) / 1.0e9


def health_epoch_elapsed_sec(health_epoch_ns: int, event_ns: int) -> float:
    if event_ns < health_epoch_ns:
        raise ValueError("event occurred before health epoch")
    return (event_ns - health_epoch_ns) / 1.0e9


def true_sample_satisfies_stability(health_epoch_ns: int, true_ns: int, required_sec: float = 0.50) -> bool:
    return health_epoch_elapsed_sec(health_epoch_ns, true_ns) >= required_sec


def twist_values(msg: Twist) -> tuple[float, float, float, float, float, float]:
    return (
        float(msg.linear.x),
        float(msg.linear.y),
        float(msg.linear.z),
        float(msg.angular.x),
        float(msg.angular.y),
        float(msg.angular.z),
    )


def stamped_values(msg: TwistStamped) -> tuple[float, float, float, float, float, float]:
    return twist_values(msg.twist)


def finite(values: tuple[float, ...]) -> bool:
    return all(math.isfinite(value) for value in values)


def nonzero(values: tuple[float, ...]) -> bool:
    return any(abs(value) > 1.0e-6 for value in values)


def sample_duration_sec(samples: list[Any]) -> float | None:
    if len(samples) < 2:
        return None
    first = int(samples[0].monotonic_ns if hasattr(samples[0], "monotonic_ns") else samples[0][0])
    last = int(samples[-1].monotonic_ns if hasattr(samples[-1], "monotonic_ns") else samples[-1][0])
    return (last - first) / 1.0e9


def sample_rate_hz(samples: list[Any]) -> float | None:
    duration = sample_duration_sec(samples)
    if duration is None or duration <= 0.0:
        return None
    return (len(samples) - 1) / duration


def contiguous_gate_owned_zero_windows(
    samples: list[VehicleOutputSample],
    *,
    max_interval_sec: float = (1.0 / 18.0) + 1.0e-6,
) -> list[list[VehicleOutputSample]]:
    windows: list[list[VehicleOutputSample]] = []
    current: list[VehicleOutputSample] = []
    for sample in sorted(samples, key=lambda item: item.monotonic_ns):
        qualifies = sample.is_gate_output and not sample.nonzero
        if qualifies:
            if current and (sample.monotonic_ns - current[-1].monotonic_ns) / 1.0e9 > max_interval_sec:
                windows.append(current)
                current = []
            current.append(sample)
        elif current:
            windows.append(current)
            current = []
    if current:
        windows.append(current)
    return windows


def select_gate_owned_zero_readiness_window(
    samples: list[VehicleOutputSample],
    *,
    min_duration_sec: float = 2.0,
    min_rate_hz: float = 18.0,
    max_rate_hz: float = 22.0,
) -> list[VehicleOutputSample]:
    for window in contiguous_gate_owned_zero_windows(samples):
        duration = sample_duration_sec(window)
        observed_rate = sample_rate_hz(window)
        if (
            duration is not None
            and observed_rate is not None
            and duration >= min_duration_sec
            and min_rate_hz <= observed_rate <= max_rate_hz
        ):
            return window
    return []


def endpoint_gid_text(endpoint_gid: Any) -> str:
    chunks = []
    for item in endpoint_gid or []:
        if isinstance(item, int):
            chunks.append(f"{item:02x}")
        elif isinstance(item, (bytes, bytearray)):
            chunks.append(bytes(item).hex())
        else:
            chunks.append(str(item))
    return "".join(chunks)


def publisher_gid_from_message_info(message_info: Any) -> str:
    if message_info is None:
        return ""
    return endpoint_gid_text(getattr(message_info, "publisher_gid", None))


def uint8_text_value(value: Any) -> int:
    if isinstance(value, int):
        return value
    if isinstance(value, (bytes, bytearray)):
        data = bytes(value)
        return data[0] if data else 0
    return int(value)


GATE_DIAGNOSTIC_NAME = "vehicle_cmd_safety/guarded_vehicle_cmd_gate"


@dataclass(frozen=True)
class VehicleOutputSample:
    monotonic_ns: int
    ros_stamp: float
    frame_id: str
    values: tuple[float, float, float, float, float, float]
    publisher_gid: str
    publisher_node: str

    @property
    def finite(self) -> bool:
        return finite(self.values)

    @property
    def nonzero(self) -> bool:
        return nonzero(self.values)

    @property
    def is_rogue_output(self) -> bool:
        return self.frame_id == ROGUE_OUTPUT_FRAME_ID

    @property
    def is_gate_output(self) -> bool:
        return self.publisher_node == "/guarded_vehicle_cmd_gate"


@dataclass(frozen=True)
class ValidityBoolSample:
    monotonic_ns: int
    value: bool
    publisher_gid: str = ""
    publisher_node: str = ""


@dataclass(frozen=True)
class ValidityWindowMetrics:
    first_sample_monotonic_ns: int | None
    last_sample_monotonic_ns: int | None
    duration_sec: float | None
    sample_count: int
    interval_count: int
    mean_frequency_hz: float | None
    median_interval_sec: float | None
    min_interval_sec: float | None
    max_interval_sec: float | None
    p95_interval_sec: float | None
    publisher_gid_set: tuple[str, ...]
    publisher_node_set: tuple[str, ...]
    unexpected_opposite_value_sample_count: int


def _sample_ns(sample: Any) -> int:
    return int(sample.monotonic_ns if hasattr(sample, "monotonic_ns") else sample["monotonic_ns"])


def _sample_value(sample: Any) -> bool:
    return bool(sample.value if hasattr(sample, "value") else sample["value"])


def _sample_gid(sample: Any) -> str:
    if hasattr(sample, "publisher_gid"):
        return str(sample.publisher_gid)
    return str(sample.get("publisher_gid", ""))


def _sample_node(sample: Any) -> str:
    if hasattr(sample, "publisher_node"):
        return str(sample.publisher_node)
    return str(sample.get("publisher_node", ""))


def contiguous_bool_windows(samples: list[Any]) -> list[list[Any]]:
    windows: list[list[Any]] = []
    current: list[Any] = []
    current_value: bool | None = None
    for sample in sorted(samples, key=_sample_ns):
        value = _sample_value(sample)
        if current and value != current_value:
            windows.append(current)
            current = []
        current.append(sample)
        current_value = value
    if current:
        windows.append(current)
    return windows


def select_contiguous_window(
    samples: list[Any],
    *,
    value: bool,
    start_ns: int | None = None,
    end_ns: int | None = None,
    containing_ns: int | None = None,
) -> list[Any]:
    for window in contiguous_bool_windows(samples):
        if not window or _sample_value(window[0]) != value:
            continue
        first = _sample_ns(window[0])
        last = _sample_ns(window[-1])
        if start_ns is not None and last < start_ns:
            continue
        if end_ns is not None and first > end_ns:
            continue
        if containing_ns is not None and not (first <= containing_ns <= last):
            continue
        return [
            sample
            for sample in window
            if (start_ns is None or _sample_ns(sample) >= start_ns)
            and (end_ns is None or _sample_ns(sample) <= end_ns)
        ]
    return []


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * q
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def validity_window_metrics(window: list[Any], *, expected_value: bool, all_samples: list[Any]) -> ValidityWindowMetrics:
    ordered = sorted(window, key=_sample_ns)
    if not ordered:
        return ValidityWindowMetrics(
            first_sample_monotonic_ns=None,
            last_sample_monotonic_ns=None,
            duration_sec=None,
            sample_count=0,
            interval_count=0,
            mean_frequency_hz=None,
            median_interval_sec=None,
            min_interval_sec=None,
            max_interval_sec=None,
            p95_interval_sec=None,
            publisher_gid_set=(),
            publisher_node_set=(),
            unexpected_opposite_value_sample_count=0,
        )
    first = _sample_ns(ordered[0])
    last = _sample_ns(ordered[-1])
    intervals = [(_sample_ns(b) - _sample_ns(a)) / 1.0e9 for a, b in zip(ordered, ordered[1:])]
    duration = (last - first) / 1.0e9
    mean_frequency = None if len(ordered) < 2 or duration <= 0.0 else (len(ordered) - 1) / duration
    opposite = sum(
        1
        for sample in all_samples
        if first <= _sample_ns(sample) <= last and _sample_value(sample) != expected_value
    )
    return ValidityWindowMetrics(
        first_sample_monotonic_ns=first,
        last_sample_monotonic_ns=last,
        duration_sec=duration,
        sample_count=len(ordered),
        interval_count=max(0, len(ordered) - 1),
        mean_frequency_hz=mean_frequency,
        median_interval_sec=None if not intervals else sorted(intervals)[len(intervals) // 2]
        if len(intervals) % 2 == 1
        else (sorted(intervals)[len(intervals) // 2 - 1] + sorted(intervals)[len(intervals) // 2]) / 2.0,
        min_interval_sec=None if not intervals else min(intervals),
        max_interval_sec=None if not intervals else max(intervals),
        p95_interval_sec=_percentile(intervals, 0.95),
        publisher_gid_set=tuple(sorted({_sample_gid(sample) for sample in ordered})),
        publisher_node_set=tuple(sorted({_sample_node(sample) for sample in ordered})),
        unexpected_opposite_value_sample_count=opposite,
    )


def validity_window_passes(
    metrics: ValidityWindowMetrics,
    *,
    min_duration_sec: float,
    min_rate_hz: float = 18.0,
    max_rate_hz: float = 22.0,
    require_one_publisher: bool = True,
) -> bool:
    if metrics.duration_sec is None or metrics.duration_sec < min_duration_sec:
        return False
    if metrics.mean_frequency_hz is None or not min_rate_hz <= metrics.mean_frequency_hz <= max_rate_hz:
        return False
    if metrics.unexpected_opposite_value_sample_count != 0:
        return False
    if require_one_publisher and (len(metrics.publisher_gid_set) != 1 or len(metrics.publisher_node_set) != 1):
        return False
    return True


def is_gate_fault_status(status: dict[str, Any]) -> bool:
    values = status.get("values") or {}
    level = uint8_text_value(status.get("level", 0))
    return (
        status.get("name", GATE_DIAGNOSTIC_NAME) == GATE_DIAGNOSTIC_NAME
        and level in {2, 3}
        and values.get("state") == "FAULT"
        and values.get("fault_latched") == "true"
    )


def select_latest_gate_fault(events: list[dict[str, Any]]) -> dict[str, Any] | None:
    faults = [event for event in events if is_gate_fault_status(event)]
    if not faults:
        return None
    return max(faults, key=lambda event: int(event["monotonic_ns"]))


def select_authoritative_gate_fault(
    state_events: list[dict[str, Any]],
    diagnostic_events: list[dict[str, Any]],
) -> dict[str, Any] | None:
    """Select a gate fault from either public state or diagnostics evidence."""
    return select_latest_gate_fault([*state_events, *diagnostic_events])


class P4CRuntimeRunner(Node):
    def __init__(self, out_dir: Path, scenario: str) -> None:
        super().__init__("phase4_p4c_runtime_runner")
        self.out_dir = out_dir
        self.scenario = scenario
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.metrics: dict[str, Any] = {"scenario": scenario, "events": []}
        self.last_safe_ns: int | None = None
        self.last_vehicle_ns: int | None = None
        self.last_collision_valid_ns: int | None = None
        self.last_false_collision_ns: int | None = None
        self.first_fault_ns: int | None = None
        self.first_zero_after_fault_ns: int | None = None
        self.first_gate_zero_after_fault_ns: int | None = None
        self.last_observation_ns: int | None = None
        self.vehicle_samples: list[VehicleOutputSample] = []
        self.safe_samples: list[tuple[int, tuple[float, ...]]] = []
        self.collision_valid_samples: list[tuple[int, bool]] = []
        self.state_events: list[dict[str, Any]] = []
        self.diagnostic_status_events: list[dict[str, Any]] = []
        self._last_heartbeat = 0.0
        self._rogue_publishers: list[dict[str, Any]] = []
        self._open_files()

        self.create_subscription(Twist, "/cmd_vel_nav_safe", self._safe_cb, 100)
        self.create_subscription(TwistStamped, "/vehicle_cmd_safe", self._vehicle_cb, 100)
        self.create_subscription(Bool, "/system/localization_valid", lambda m: self._perm_cb("localization", m), 50)
        self.create_subscription(Bool, "/system/controller_valid", lambda m: self._perm_cb("controller", m), 50)
        self.create_subscription(Bool, "/system/collision_monitor_valid", self._collision_valid_cb, 50)
        self.create_subscription(DiagnosticStatus, "/vehicle_cmd_safety/state", self._state_cb, 50)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diag_cb, 50)
        self.create_timer(0.05, self._heartbeat_cb)
        self.arm_client = self.create_client(SetBool, "/vehicle_cmd_safety/arm")
        self.safe_param_client = self.create_client(SetParameters, "/phase4_p4c_safe_twist_fixture/set_parameters")
        self.permission_param_client = self.create_client(SetParameters, "/phase4_p4c_permission_fixture/set_parameters")
        self.obstacle_param_client = self.create_client(SetParameters, "/phase4_p4b_synthetic_obstacles/set_parameters")

    def _open_files(self) -> None:
        self.files = {
            "safe": (self.out_dir / "safe_input_timeline.tsv").open("w", encoding="utf-8"),
            "vehicle": (self.out_dir / "vehicle_output_timeline.tsv").open("w", encoding="utf-8"),
            "permission": (self.out_dir / "permission_timeline.tsv").open("w", encoding="utf-8"),
            "collision_valid": (self.out_dir / "collision_valid_timeline.tsv").open("w", encoding="utf-8"),
            "state": (self.out_dir / "gate_state_events.jsonl").open("w", encoding="utf-8"),
            "arm": (self.out_dir / "arm_service_events.jsonl").open("w", encoding="utf-8"),
            "diag": (self.out_dir / "diagnostics.jsonl").open("w", encoding="utf-8"),
            "authority": (self.out_dir / "authority_timeline.tsv").open("w", encoding="utf-8"),
            "provenance": (self.out_dir / "publisher_provenance.tsv").open("w", encoding="utf-8"),
            "process": (self.out_dir / "process_authority.tsv").open("w", encoding="utf-8"),
            "heartbeat": (self.out_dir / "monitor_heartbeat.tsv").open("w", encoding="utf-8"),
            "resource": (self.out_dir / "resource_timeline.tsv").open("w", encoding="utf-8"),
            "observation": (self.out_dir / "collision_observation_timeline.tsv").open("w", encoding="utf-8"),
        }
        self.files["safe"].write("monotonic_ns\tvalues\tfinite\tnonzero\tpublisher_gid\tpublisher_node\n")
        self.files["vehicle"].write("monotonic_ns\tros_stamp\tframe_id\tvalues\tfinite\tnonzero\tpublisher_gid\tpublisher_node\n")
        self.files["permission"].write("monotonic_ns\tname\tvalue\n")
        self.files["collision_valid"].write("monotonic_ns\tvalue\n")
        self.files["authority"].write("monotonic_ns\ttopic\tpublisher_count\tsubscriber_count\n")
        self.files["provenance"].write("monotonic_ns\ttopic\tpublisher_count\tgid\tnode_name\tnode_namespace\ttopic_type\n")
        self.files["process"].write("monotonic_ns\tpid\tcommand\n")
        self.files["heartbeat"].write("monotonic_ns\tsafe_count\tvehicle_count\tcollision_valid_count\tstate_count\n")
        self.files["resource"].write("monotonic_ns\tloadavg_1\tloadavg_5\tloadavg_15\n")
        self.files["observation"].write("monotonic_ns\tmode_or_event\n")

    def close(self) -> None:
        for handle in self.files.values():
            handle.close()

    def mono_ns(self) -> int:
        return time.monotonic_ns()

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _publisher_identity(self, topic: str, message_info: Any = None) -> tuple[str, str]:
        message_gid = publisher_gid_from_message_info(message_info)
        infos = self.get_publishers_info_by_topic(topic)
        if message_gid:
            for info in infos:
                gid_text = endpoint_gid_text(getattr(info, "endpoint_gid", None))
                if gid_text == message_gid:
                    return gid_text, f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            return message_gid, ""
        if len(infos) != 1:
            return "", ""
        info = infos[0]
        gid_text = endpoint_gid_text(getattr(info, "endpoint_gid", None))
        return gid_text, f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")

    def _vehicle_identity_from_distinguishable_payload(self, frame_id: str) -> tuple[str, str]:
        publishers = self.current_publishers_by_topic("/vehicle_cmd_safe")
        if len(publishers) != 2:
            return "", ""
        matching_node = ""
        if frame_id == ROGUE_OUTPUT_FRAME_ID:
            matching_node = "/phase4_p4c_runtime_runner"
        elif frame_id:
            matching_node = "/guarded_vehicle_cmd_gate"
        if not matching_node:
            return "", ""
        matches = [(gid, node) for gid, node in publishers.items() if node == matching_node]
        if len(matches) != 1:
            return "", ""
        return matches[0]

    def _safe_cb(self, msg: Twist) -> None:
        values = twist_values(msg)
        now = self.mono_ns()
        gid, node = self._publisher_identity("/cmd_vel_nav_safe")
        self.safe_samples.append((now, values))
        self.last_safe_ns = now
        self.files["safe"].write(f"{now}\t{','.join(str(v) for v in values)}\t{finite(values)}\t{nonzero(values)}\t{gid}\t{node}\n")
        self.files["safe"].flush()

    def _vehicle_cb(self, msg: TwistStamped, message_info: Any = None) -> None:
        values = stamped_values(msg)
        now = self.mono_ns()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1_000_000_000.0
        gid, node = self._publisher_identity("/vehicle_cmd_safe", message_info)
        if not node:
            gid, node = self._vehicle_identity_from_distinguishable_payload(msg.header.frame_id)
        sample = VehicleOutputSample(
            monotonic_ns=now,
            ros_stamp=stamp,
            frame_id=msg.header.frame_id,
            values=values,
            publisher_gid=gid,
            publisher_node=node,
        )
        self.vehicle_samples.append(sample)
        self.last_vehicle_ns = now
        if self.first_fault_ns is not None and self.first_zero_after_fault_ns is None and not nonzero(values):
            self.first_zero_after_fault_ns = now
        if (
            self.first_fault_ns is not None
            and self.first_gate_zero_after_fault_ns is None
            and sample.is_gate_output
            and not sample.nonzero
        ):
            self.first_gate_zero_after_fault_ns = now
        self.files["vehicle"].write(f"{now}\t{stamp:.9f}\t{msg.header.frame_id}\t{','.join(str(v) for v in values)}\t{finite(values)}\t{nonzero(values)}\t{gid}\t{node}\n")
        self.files["vehicle"].flush()

    def _perm_cb(self, name: str, msg: Bool) -> None:
        self.files["permission"].write(f"{self.mono_ns()}\t{name}\t{str(bool(msg.data)).lower()}\n")
        self.files["permission"].flush()

    def _collision_valid_cb(self, msg: Bool) -> None:
        now = self.mono_ns()
        value = bool(msg.data)
        self.last_collision_valid_ns = now
        if not value and self.last_false_collision_ns is None:
            self.last_false_collision_ns = now
        self.collision_valid_samples.append((now, value))
        self.files["collision_valid"].write(f"{now}\t{str(value).lower()}\n")
        self.files["collision_valid"].flush()

    def _state_cb(self, msg: DiagnosticStatus) -> None:
        payload = self._status_payload(msg)
        if is_gate_fault_status(payload) and self.first_fault_ns is None:
            self.first_fault_ns = int(payload["monotonic_ns"])
        self.state_events.append(payload)
        self.files["state"].write(json.dumps(payload, sort_keys=True) + "\n")
        self.files["state"].flush()

    def _diag_cb(self, msg: DiagnosticArray) -> None:
        payloads = [self._status_payload(status) for status in msg.status]
        self.diagnostic_status_events.extend(payloads)
        self._refresh_first_fault_from_evidence()
        payload = {
            "monotonic_ns": self.mono_ns(),
            "status": [
                {
                    "name": status.name,
                    "level": uint8_text_value(status.level),
                    "message": status.message,
                    "values": {item.key: item.value for item in status.values},
                }
                for status in msg.status
            ],
        }
        self.files["diag"].write(json.dumps(payload, sort_keys=True) + "\n")
        self.files["diag"].flush()

    def _refresh_first_fault_from_evidence(self) -> None:
        if self.first_fault_ns is not None:
            return
        fault = select_authoritative_gate_fault(self.state_events, self.diagnostic_status_events)
        if fault is not None:
            self.first_fault_ns = int(fault["monotonic_ns"])

    def _status_payload(self, msg: DiagnosticStatus) -> dict[str, Any]:
        return {
            "monotonic_ns": self.mono_ns(),
            "name": msg.name,
            "level": uint8_text_value(msg.level),
            "message": msg.message,
            "values": {item.key: item.value for item in msg.values},
        }

    def _heartbeat_cb(self) -> None:
        now = time.monotonic()
        if now - self._last_heartbeat < 0.1:
            return
        self._last_heartbeat = now
        mono = self.mono_ns()
        self.files["heartbeat"].write(
            f"{mono}\t{len(self.safe_samples)}\t{len(self.vehicle_samples)}\t{len(self.collision_valid_samples)}\t{len(self.state_events)}\n"
        )
        topic_types = dict(self.get_topic_names_and_types())
        for topic in AUTHORITY_TOPICS:
            pubs = self.get_publishers_info_by_topic(topic)
            subs = self.get_subscriptions_info_by_topic(topic)
            self.files["authority"].write(f"{mono}\t{topic}\t{len(pubs)}\t{len(subs)}\n")
            if not pubs:
                self.files["provenance"].write(f"{mono}\t{topic}\t0\t\t\t\t\n")
            for info in pubs:
                gid_text = endpoint_gid_text(getattr(info, "endpoint_gid", None))
                types = ",".join(sorted(topic_types.get(topic, [])))
                self.files["provenance"].write(
                    f"{mono}\t{topic}\t{len(pubs)}\t{gid_text}\t{info.node_name}\t{info.node_namespace}\t{types}\n"
                )
        for pid, cmd in self._matching_processes():
            self.files["process"].write(f"{mono}\t{pid}\t{cmd}\n")
        try:
            loads = os.getloadavg()
        except OSError:
            loads = (0.0, 0.0, 0.0)
        self.files["resource"].write(f"{mono}\t{loads[0]}\t{loads[1]}\t{loads[2]}\n")
        for name in ("heartbeat", "authority", "provenance", "process", "resource"):
            self.files[name].flush()

    def _matching_processes(self) -> list[tuple[int, str]]:
        result = subprocess.run(["ps", "-eo", "pid=,args="], check=False, text=True, capture_output=True)
        rows = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            pid_text, _, cmd = stripped.partition(" ")
            if not pid_text.isdigit():
                continue
            if any(pattern in cmd for pattern in PROCESS_PATTERNS):
                rows.append((int(pid_text), cmd))
        return rows

    def set_node_params(self, client, values: dict[str, Any], timeout: float = 5.0) -> None:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("parameter service unavailable")
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name, self._parameter_type(value), value).to_parameter_msg()
            for name, value in values.items()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("parameter request timeout")
        failed = [result.reason for result in future.result().results if not result.successful]
        if failed:
            raise RuntimeError(f"parameter request failed: {failed}")

    @staticmethod
    def _parameter_type(value: Any):
        if isinstance(value, bool):
            return Parameter.Type.BOOL
        if isinstance(value, float):
            return Parameter.Type.DOUBLE
        if isinstance(value, int):
            return Parameter.Type.INTEGER
        return Parameter.Type.STRING

    def arm(self, enable: bool, timeout: float = 5.0) -> tuple[bool, str]:
        if not self.arm_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("arm service unavailable")
        request = SetBool.Request()
        request.data = bool(enable)
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("arm service timeout")
        result = future.result()
        event = {"monotonic_ns": self.mono_ns(), "request": enable, "success": bool(result.success), "message": result.message}
        self.metrics["events"].append(event)
        self.files["arm"].write(json.dumps(event, sort_keys=True) + "\n")
        self.files["arm"].flush()
        return bool(result.success), str(result.message)

    def set_safe(self, **values: Any) -> None:
        self.set_node_params(self.safe_param_client, values)
        self.metrics["events"].append({"monotonic_ns": self.mono_ns(), "safe_params": values})

    def set_permission(self, **values: Any) -> None:
        self.set_node_params(self.permission_param_client, values)
        self.metrics["events"].append({"monotonic_ns": self.mono_ns(), "permission_params": values})

    def set_obstacle_mode(self, mode: str) -> None:
        self.set_node_params(self.obstacle_param_client, {"mode": mode})
        now = self.mono_ns()
        if mode != "SILENT":
            self.last_observation_ns = now
        self.metrics["events"].append({"monotonic_ns": now, "obstacle_mode": mode})
        self.files["observation"].write(f"{now}\t{mode}\n")
        self.files["observation"].flush()

    def set_validity_params(self, **values: Any) -> None:
        self.set_node_params(self.create_client(SetParameters, "/collision_monitor_validity_monitor/set_parameters"), values)
        self.metrics["events"].append({"monotonic_ns": self.mono_ns(), "validity_params": values})

    def wait_vehicle_nonzero(self, timeout_sec: float = 3.0) -> None:
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            if self.vehicle_samples and self.vehicle_samples[-1].nonzero:
                return
            self.spin_for(0.02)
        raise RuntimeError("vehicle output did not become nonzero")

    def wait_for_collision_valid(self, expected: bool, timeout_sec: float = 5.0) -> int:
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            for stamp, value in reversed(self.collision_valid_samples):
                if value == expected:
                    return stamp
            self.spin_for(0.02)
        raise RuntimeError(f"collision validity did not become {expected}")

    def create_rogue_publisher(self, topic: str) -> None:
        if topic == "/vehicle_cmd_safe":
            pub = self.create_publisher(TwistStamped, topic, 10)
        elif topic in {"/system/localization_valid", "/system/controller_valid", "/system/collision_monitor_valid"}:
            pub = self.create_publisher(Bool, topic, 10)
        else:
            pub = self.create_publisher(Twist, topic, 10)
        self._rogue_publishers.append(
            {
                "topic": topic,
                "publisher": pub,
                "last_publish_monotonic": None,
                "publish_count": 0,
                "published_nonzero_count": 0,
            }
        )
        self.metrics["events"].append(
            {
                "monotonic_ns": self.mono_ns(),
                "rogue_publisher": topic,
                "rogue_output_frame_id": ROGUE_OUTPUT_FRAME_ID if topic == "/vehicle_cmd_safe" else "",
                "rogue_rate_hz": ROGUE_OUTPUT_RATE_HZ if topic == "/vehicle_cmd_safe" else None,
            }
        )

    def publish_rogues(self) -> None:
        now = time.monotonic()
        for rogue in self._rogue_publishers:
            topic = rogue["topic"]
            pub = rogue["publisher"]
            last = rogue["last_publish_monotonic"]
            if topic == "/vehicle_cmd_safe" and last is not None and now - last < 1.0 / ROGUE_OUTPUT_RATE_HZ:
                continue
            if topic == "/vehicle_cmd_safe":
                msg = TwistStamped()
                msg.header.frame_id = ROGUE_OUTPUT_FRAME_ID
                values = stamped_values(msg)
                if nonzero(values):
                    rogue["published_nonzero_count"] += 1
                    raise RuntimeError("duplicate-output rogue attempted nonzero publish")
                pub.publish(msg)
            elif topic in {"/system/localization_valid", "/system/controller_valid", "/system/collision_monitor_valid"}:
                msg = Bool()
                msg.data = True
                pub.publish(msg)
            else:
                pub.publish(Twist())
            rogue["last_publish_monotonic"] = now
            rogue["publish_count"] += 1

    def destroy_rogue_publishers(self) -> None:
        for rogue in self._rogue_publishers:
            self.destroy_publisher(rogue["publisher"])
        count = len(self._rogue_publishers)
        self._rogue_publishers.clear()
        self.metrics["events"].append({"monotonic_ns": self.mono_ns(), "rogue_publishers_destroyed": count})

    def spin_with_rogues(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publish_rogues()
            rclpy.spin_once(self, timeout_sec=0.02)

    def current_publishers_by_topic(self, topic: str) -> dict[str, str]:
        publishers = {}
        for info in self.get_publishers_info_by_topic(topic):
            gid = endpoint_gid_text(getattr(info, "endpoint_gid", None))
            node = f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            publishers[gid] = node
        return publishers

    def summarize_common(self) -> None:
        vehicle_nonzero = [sample for sample in self.vehicle_samples if sample.nonzero]
        vehicle_zero = [sample for sample in self.vehicle_samples if not sample.nonzero]
        gate_samples = [sample for sample in self.vehicle_samples if sample.is_gate_output]
        gate_zero = [sample for sample in gate_samples if not sample.nonzero]
        gate_nonzero = [sample for sample in gate_samples if sample.nonzero]
        rogue_output_samples = [sample for sample in self.vehicle_samples if sample.is_rogue_output]
        rogue_output_zero = [sample for sample in rogue_output_samples if not sample.nonzero]
        rogue_output_nonzero = [sample for sample in rogue_output_samples if sample.nonzero]
        gate_zero_after_fault_plus_reaction = [
            sample
            for sample in gate_zero
            if self.first_fault_ns is not None and sample.monotonic_ns >= self.first_fault_ns + 100_000_000
        ]
        gate_nonzero_after_fault_plus_reaction = [
            sample
            for sample in gate_nonzero
            if self.first_fault_ns is not None and sample.monotonic_ns >= self.first_fault_ns + 100_000_000
        ]
        gate_zero_rate_hz = sample_rate_hz(gate_zero_after_fault_plus_reaction)
        rogue_output_rate_hz = sample_rate_hz(rogue_output_samples)
        self.metrics.update(
            {
                "safe_sample_count": len(self.safe_samples),
                "vehicle_sample_count": len(self.vehicle_samples),
                "collision_valid_sample_count": len(self.collision_valid_samples),
                "state_sample_count": len(self.state_events),
                "vehicle_nonzero_count": len(vehicle_nonzero),
                "vehicle_zero_count": len(vehicle_zero),
                "all_vehicle_finite": all(sample.finite for sample in self.vehicle_samples),
                "all_safe_finite": all(finite(v) for _, v in self.safe_samples),
                "first_fault_latency_from_false_collision_sec": None
                if self.first_fault_ns is None or self.last_false_collision_ns is None
                else (self.first_fault_ns - self.last_false_collision_ns) / 1.0e9,
                "first_zero_latency_from_fault_sec": None
                if self.first_zero_after_fault_ns is None or self.first_fault_ns is None
                else (self.first_zero_after_fault_ns - self.first_fault_ns) / 1.0e9,
                "first_gate_zero_latency_from_fault_sec": None
                if self.first_gate_zero_after_fault_ns is None or self.first_fault_ns is None
                else (self.first_gate_zero_after_fault_ns - self.first_fault_ns) / 1.0e9,
                "gate_owned_vehicle_sample_count": len(gate_samples),
                "gate_owned_zero_count": len(gate_zero),
                "gate_owned_nonzero_count": len(gate_nonzero),
                "gate_owned_nonzero_after_fault_plus_0p10_count": len(gate_nonzero_after_fault_plus_reaction),
                "gate_owned_zero_rate_hz_after_fault_plus_0p10": gate_zero_rate_hz,
                "gate_owned_zero_rate_window_sec_after_fault_plus_0p10": sample_duration_sec(gate_zero_after_fault_plus_reaction),
                "rogue_output_sample_count": len(rogue_output_samples),
                "rogue_output_zero_count": len(rogue_output_zero),
                "rogue_output_nonzero_count": len(rogue_output_nonzero),
                "rogue_output_rate_hz": rogue_output_rate_hz,
                "rogue_output_rate_window_sec": sample_duration_sec(rogue_output_samples),
                "rogue_publishers": [
                    {
                        "topic": rogue["topic"],
                        "publish_count": rogue["publish_count"],
                        "published_nonzero_count": rogue["published_nonzero_count"],
                    }
                    for rogue in self._rogue_publishers
                ],
            }
        )

    def write_metrics(self, status: str, error: str | None = None) -> dict[str, Any]:
        self.summarize_common()
        self.metrics["status"] = status
        self.metrics["error"] = error
        with (self.out_dir / "terminal_metrics.json").open("w", encoding="utf-8") as handle:
            json.dump(self.metrics, handle, indent=2, sort_keys=True)
        return self.metrics


def run_gate_preflight(node: P4CRuntimeRunner) -> str:
    ok, reason = node.arm(True)
    node.metrics["early_arm_reason"] = reason
    if ok:
        raise RuntimeError(f"unexpected early arm result: {ok} {reason}")
    node.spin_for(1.3)
    ok, reason = node.arm(True)
    if not ok:
        raise RuntimeError(f"arm failed after stable prerequisites: {reason}")
    node.wait_vehicle_nonzero()
    node.set_safe(linear_x=0.30, angular_z=0.70)
    node.spin_for(0.5)
    node.set_safe(linear_x=0.02, angular_z=0.01)
    node.spin_for(0.3)
    before = node.mono_ns()
    ok, reason = node.arm(False)
    if not ok:
        raise RuntimeError(f"disarm failed: {reason}")
    node.spin_for(2.1)
    first_zero = next(
        (sample.monotonic_ns for sample in node.vehicle_samples if sample.monotonic_ns >= before and not sample.nonzero),
        None,
    )
    node.metrics.update(
        {
            "disarm_first_zero_latency_sec": None if first_zero is None else (first_zero - before) / 1.0e9,
            "max_linear_output": max((abs(sample.values[0]) for sample in node.vehicle_samples), default=0.0),
            "max_angular_output": max((abs(sample.values[5]) for sample in node.vehicle_samples), default=0.0),
        }
    )
    return "P4C_GATE_PREFLIGHT_PASS"


FAULT_CASES = {
    "safe_stale": lambda n: n.set_safe(mode="SILENT"),
    "localization_false": lambda n: n.set_permission(localization_valid=False),
    "localization_stale": lambda n: n.set_permission(publish_localization=False),
    "controller_false": lambda n: n.set_permission(controller_valid=False),
    "controller_stale": lambda n: n.set_permission(publish_controller=False),
    "collision_false": lambda n: n.set_permission(collision_valid=False),
    "collision_stale": lambda n: n.set_permission(publish_collision=False),
    "nan": lambda n: n.set_safe(mode="NAN"),
    "inf": lambda n: n.set_safe(mode="INF"),
    "unsupported_axes": lambda n: n.set_safe(mode="UNSUPPORTED_AXIS"),
    "reverse": lambda n: n.set_safe(mode="REVERSE"),
    "in_place_rotation": lambda n: n.set_safe(mode="IN_PLACE"),
    "duplicate_safe_input": lambda n: n.create_rogue_publisher("/cmd_vel_nav_safe"),
    "duplicate_output": lambda n: n.create_rogue_publisher("/vehicle_cmd_safe"),
    "duplicate_permission": lambda n: n.create_rogue_publisher("/system/localization_valid"),
    "duplicate_localization_permission": lambda n: n.create_rogue_publisher("/system/localization_valid"),
    "duplicate_controller_permission": lambda n: n.create_rogue_publisher("/system/controller_valid"),
    "duplicate_collision_permission": lambda n: n.create_rogue_publisher("/system/collision_monitor_valid"),
}


def diagnostic_status(
    *,
    level: int,
    state: str,
    reason: str,
    latched: bool,
    name: str = GATE_DIAGNOSTIC_NAME,
) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.name = name
    status.level = bytes([level])
    status.message = reason
    for key, value in {
        "state": state,
        "reason_code": reason,
        "fault_latched": str(latched).lower(),
    }.items():
        item = KeyValue()
        item.key = key
        item.value = value
        status.values.append(item)
    return status


def run_diagnostic_preflight(node: P4CRuntimeRunner) -> str:
    state_pub = node.create_publisher(DiagnosticStatus, "/vehicle_cmd_safety/state", 10)
    diag_pub = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
    node.spin_for(0.2)

    ok_status = diagnostic_status(level=0, state="ARMED", reason="ARMED_COMMAND", latched=False)
    warn_status = diagnostic_status(level=1, state="DISARMED", reason="DISARMED_ZERO", latched=False)
    unrelated_fault = diagnostic_status(
        level=2,
        state="FAULT",
        reason="SAFE_TWIST_STALE",
        latched=True,
        name="vehicle_cmd_safety/unrelated_status",
    )
    stale_fault = diagnostic_status(level=3, state="FAULT", reason="SAFE_TWIST_STALE", latched=True)
    error_fault = diagnostic_status(level=2, state="FAULT", reason="SAFE_TWIST_STALE", latched=True)

    state_pub.publish(ok_status)
    state_pub.publish(warn_status)
    array = DiagnosticArray()
    array.status.extend([unrelated_fault, ok_status, error_fault])
    diag_pub.publish(array)
    node.spin_for(0.2)
    state_pub.publish(stale_fault)
    node.spin_for(1.2)

    selected = select_authoritative_gate_fault(node.state_events, node.diagnostic_status_events)
    if selected is None:
        raise RuntimeError("diagnostic preflight did not recognize fault")
    if selected["values"].get("reason_code") != "SAFE_TWIST_STALE":
        raise RuntimeError(f"unexpected diagnostic reason: {selected}")
    if int(selected["level"]) not in {2, 3}:
        raise RuntimeError(f"unexpected diagnostic level: {selected}")
    if selected["name"] != GATE_DIAGNOSTIC_NAME:
        raise RuntimeError(f"unexpected diagnostic name: {selected}")
    node.metrics.update(
        {
            "diagnostic_preflight_selected_level": int(selected["level"]),
            "diagnostic_preflight_selected_reason": selected["values"].get("reason_code"),
            "diagnostic_preflight_state_events": len(node.state_events),
            "diagnostic_preflight_diagnostic_events": len(node.diagnostic_status_events),
            "diagnostic_preflight_fault_monotonic_ns": int(selected["monotonic_ns"]),
        }
    )
    return "P4C_DIAGNOSTIC_RUNNER_PREFLIGHT_PASS"


def run_fault_case(node: P4CRuntimeRunner, case: str) -> str:
    if case == "duplicate_output":
        return run_duplicate_output_fault_case(node)
    node.spin_for(2.5)
    ok, reason = node.arm(True)
    if not ok:
        raise RuntimeError(f"arm failed before fault {case}: {reason}")
    node.wait_vehicle_nonzero()
    event_ns = node.mono_ns()
    FAULT_CASES[case](node)
    node.spin_with_rogues(2.2)
    node._refresh_first_fault_from_evidence()
    if node.first_fault_ns is None:
        raise RuntimeError(f"no fault observed for {case}")
    if node.first_zero_after_fault_ns is None:
        raise RuntimeError(f"no zero after fault for {case}")
    ok, _ = node.arm(True)
    if ok:
        raise RuntimeError("arm=true succeeded while faulted")
    node.metrics.update(
        {
            "fault_case": case,
            "fault_event_to_fault_sec": (node.first_fault_ns - event_ns) / 1.0e9,
            "fault_to_first_zero_sec": (node.first_zero_after_fault_ns - node.first_fault_ns) / 1.0e9,
            "fault_reason": node.state_events[-1]["message"] if node.state_events else "",
        }
    )
    return "P4C_FAULT_MATRIX_PREFLIGHT_PASS"


def assert_duplicate_output_attribution(node: P4CRuntimeRunner) -> dict[str, Any]:
    publishers = node.current_publishers_by_topic("/vehicle_cmd_safe")
    if len(publishers) != 2:
        raise RuntimeError(f"expected two /vehicle_cmd_safe publishers, observed {publishers}")
    gate_gids = [gid for gid, name in publishers.items() if name == "/guarded_vehicle_cmd_gate"]
    rogue_gids = [gid for gid, name in publishers.items() if name == "/phase4_p4c_runtime_runner"]
    if len(gate_gids) != 1 or len(rogue_gids) != 1:
        raise RuntimeError(f"could not independently identify gate and rogue publishers: {publishers}")
    gate_samples = [sample for sample in node.vehicle_samples if sample.publisher_gid == gate_gids[0]]
    rogue_samples = [sample for sample in node.vehicle_samples if sample.publisher_gid == rogue_gids[0]]
    if not gate_samples:
        raise RuntimeError("no gate-owned /vehicle_cmd_safe samples attributed")
    if not rogue_samples:
        raise RuntimeError("no rogue-owned /vehicle_cmd_safe samples attributed")
    if any(sample.frame_id != ROGUE_OUTPUT_FRAME_ID for sample in rogue_samples):
        raise RuntimeError("rogue output frame attribution was not deterministic")
    if any(sample.nonzero for sample in rogue_samples):
        raise RuntimeError("rogue output was not zero-only")
    return {
        "aggregate_authority_classification": "AUTHORITY_INVALID",
        "output_publishers": publishers,
        "gate_output_gid": gate_gids[0],
        "rogue_output_gid": rogue_gids[0],
        "gate_attributed_sample_count": len(gate_samples),
        "rogue_attributed_sample_count": len(rogue_samples),
    }


def gate_zero_rate_after_fault(node: P4CRuntimeRunner) -> tuple[float | None, float | None]:
    if node.first_fault_ns is None:
        return None, None
    gate_zero = [
        sample
        for sample in node.vehicle_samples
        if sample.is_gate_output and not sample.nonzero and sample.monotonic_ns >= node.first_fault_ns + 100_000_000
    ]
    return sample_rate_hz(gate_zero), sample_duration_sec(gate_zero)


def assert_duplicate_output_gate_behavior(node: P4CRuntimeRunner) -> dict[str, Any]:
    if node.first_fault_ns is None:
        raise RuntimeError("duplicate-output fault was not observed")
    if node.first_gate_zero_after_fault_ns is None:
        raise RuntimeError("no gate-owned zero output after duplicate-output fault")
    first_gate_zero_sec = (node.first_gate_zero_after_fault_ns - node.first_fault_ns) / 1.0e9
    if first_gate_zero_sec > 0.10:
        raise RuntimeError(f"gate-owned zero latency exceeded 0.10s: {first_gate_zero_sec}")
    late_nonzero = [
        sample
        for sample in node.vehicle_samples
        if sample.is_gate_output and sample.nonzero and sample.monotonic_ns >= node.first_fault_ns + 100_000_000
    ]
    if late_nonzero:
        raise RuntimeError(f"gate-owned nonzero after FAULT+0.10s: {late_nonzero[:3]}")
    rate_hz, window_sec = gate_zero_rate_after_fault(node)
    if rate_hz is None or window_sec is None or window_sec < 2.0:
        raise RuntimeError(f"insufficient gate-owned zero-rate window: rate={rate_hz} window={window_sec}")
    if not 18.0 <= rate_hz <= 22.0:
        raise RuntimeError(f"gate-owned zero heartbeat outside 18-22 Hz: {rate_hz}")
    return {
        "first_gate_zero_latency_from_fault_sec": first_gate_zero_sec,
        "gate_owned_zero_rate_hz": rate_hz,
        "gate_owned_zero_rate_window_sec": window_sec,
        "gate_owned_nonzero_after_fault_plus_0p10_count": len(late_nonzero),
    }


def run_duplicate_output_attribution_preflight(node: P4CRuntimeRunner) -> str:
    node.spin_for(1.5)
    node.create_rogue_publisher("/vehicle_cmd_safe")
    node.spin_with_rogues(2.6)
    attribution = assert_duplicate_output_attribution(node)
    gate_zero = [sample for sample in node.vehicle_samples if sample.is_gate_output and not sample.nonzero]
    rate_hz = sample_rate_hz(gate_zero)
    window_sec = sample_duration_sec(gate_zero)
    if rate_hz is None or window_sec is None or window_sec < 2.0:
        raise RuntimeError(f"insufficient gate-owned preflight zero-rate window: rate={rate_hz} window={window_sec}")
    if not 18.0 <= rate_hz <= 22.0:
        raise RuntimeError(f"gate-owned preflight rate outside 18-22 Hz: {rate_hz}")
    node.metrics.update(
        {
            **attribution,
            "preflight_gate_owned_zero_rate_hz": rate_hz,
            "preflight_gate_owned_zero_rate_window_sec": window_sec,
            "preflight_rogue_zero_only": True,
            "delayed_prefault_message_classification": "publisher_gid_and_frame_id",
        }
    )
    return "P4C_DUPLICATE_OUTPUT_ATTRIBUTION_PREFLIGHT_PASS"


def run_duplicate_output_fault_case(node: P4CRuntimeRunner) -> str:
    node.spin_for(2.5)
    ok, reason = node.arm(True)
    if not ok:
        raise RuntimeError(f"arm failed before fault duplicate_output: {reason}")
    node.wait_vehicle_nonzero()
    event_ns = node.mono_ns()
    node.create_rogue_publisher("/vehicle_cmd_safe")
    node.spin_with_rogues(3.2)
    node._refresh_first_fault_from_evidence()
    attribution = assert_duplicate_output_attribution(node)
    gate_behavior = assert_duplicate_output_gate_behavior(node)
    ok, _ = node.arm(True)
    if ok:
        raise RuntimeError("arm=true succeeded while duplicate-output faulted")
    node.destroy_rogue_publishers()
    node.spin_for(0.8)
    late_after_remove = [
        sample
        for sample in node.vehicle_samples
        if sample.monotonic_ns >= node.metrics["events"][-1]["monotonic_ns"] and sample.is_gate_output and sample.nonzero
    ]
    if late_after_remove:
        raise RuntimeError("gate motion resumed after rogue removal without explicit recovery")
    ok, _ = node.arm(True)
    if ok:
        raise RuntimeError("arm=true succeeded after rogue removal without explicit recovery")
    ok, reason = node.arm(False)
    if not ok:
        raise RuntimeError(f"arm=false did not clear duplicate-output fault after cause removal: {reason}")
    node.spin_for(1.2)
    ok, reason = node.arm(True)
    if not ok:
        raise RuntimeError(f"re-arm failed after explicit duplicate-output recovery: {reason}")
    node.wait_vehicle_nonzero()
    node.metrics.update(
        {
            **attribution,
            **gate_behavior,
            "fault_case": "duplicate_output",
            "fault_event_to_fault_sec": (node.first_fault_ns - event_ns) / 1.0e9 if node.first_fault_ns is not None else None,
            "fault_to_first_zero_sec": (node.first_zero_after_fault_ns - node.first_fault_ns) / 1.0e9
            if node.first_zero_after_fault_ns is not None and node.first_fault_ns is not None
            else None,
            "fault_to_first_gate_zero_sec": gate_behavior["first_gate_zero_latency_from_fault_sec"],
            "fault_reason": node.state_events[-1]["message"] if node.state_events else "",
            "rogue_removed_without_motion": True,
            "explicit_recovery_required": True,
        }
    )
    return "P4C_FAULT_MATRIX_PREFLIGHT_PASS"


def run_fault_recovery(node: P4CRuntimeRunner) -> str:
    node.spin_for(2.5)
    ok, reason = node.arm(True)
    if not ok:
        raise RuntimeError(f"arm failed before recovery fault: {reason}")
    node.wait_vehicle_nonzero()
    node.set_permission(collision_valid=False)
    node.spin_for(0.5)
    if node.first_fault_ns is None:
        raise RuntimeError("no recovery fault observed")
    ok, _ = node.arm(False)
    if ok:
        raise RuntimeError("arm=false cleared while cause remained")
    node.set_permission(collision_valid=True)
    node.spin_for(0.6)
    ok, reason = node.arm(False)
    if not ok:
        raise RuntimeError(f"arm=false did not clear removed fault: {reason}")
    node.spin_for(1.2)
    ok, reason = node.arm(True)
    if not ok:
        raise RuntimeError(f"re-arm failed after recovery: {reason}")
    node.wait_vehicle_nonzero()
    return "P4C_FAULT_MATRIX_PREFLIGHT_PASS"


def run_validity_preflight(node: P4CRuntimeRunner, mode: str) -> str:
    true_ns = node.wait_for_collision_valid(True, timeout_sec=6.0)
    node.set_obstacle_mode("SILENT")
    silent_ns = node.mono_ns()
    node.spin_for(1.2)
    false_after = next((t for t, value in node.collision_valid_samples if t >= silent_ns and not value), None)
    if false_after is None:
        raise RuntimeError("validity did not become false after SILENT")
    node.set_obstacle_mode("CLEAR")
    recovery_ns = node.mono_ns()
    true_after = None
    start = time.monotonic()
    while time.monotonic() - start < 6.0:
        true_after = next((t for t, value in node.collision_valid_samples if t >= recovery_ns and value), None)
        if true_after is not None:
            break
        node.spin_for(0.05)
    if true_after is None:
        raise RuntimeError("validity did not recover true")
    node.metrics.update(
        {
            "validity_mode": mode,
            "healthy_true_latency_sec": (true_ns - node.collision_valid_samples[0][0]) / 1.0e9
            if node.collision_valid_samples
            else None,
            "silent_false_latency_sec": (false_after - silent_ns) / 1.0e9,
            "recovery_true_latency_sec": (true_after - recovery_ns) / 1.0e9,
        }
    )
    return "P4C_COLLISION_VALIDITY_PREFLIGHT_PASS"


def run_validity_mismatch(node: P4CRuntimeRunner, case: str) -> str:
    node.wait_for_collision_valid(False, timeout_sec=6.0)
    if case == "wrong_frame":
        node.set_validity_params(expected_frame="laser_frame")
    elif case == "lifecycle_inactive":
        node.set_validity_params(collision_monitor_node_name="missing_collision_monitor")
    elif case == "source_mismatch":
        node.set_validity_params(expected_observation_source_topic="/phase4/other_source")
    else:
        raise RuntimeError(f"unsupported validity mismatch case {case}")
    node.spin_for(1.2)
    if not any(not value for _, value in node.collision_valid_samples):
        raise RuntimeError(f"validity did not remain false for {case}")
    return "P4C_COLLISION_VALIDITY_PREFLIGHT_PASS"


def run_e2e_stale(node: P4CRuntimeRunner) -> str:
    node.spin_for(2.5)
    if not any(value for _, value in node.collision_valid_samples):
        raise RuntimeError("collision validity never became true")
    ok, reason = node.arm(True)
    if not ok:
        raise RuntimeError(f"arm failed in e2e stale scenario: {reason}")
    node.wait_vehicle_nonzero()
    node.set_obstacle_mode("SILENT")
    last_observation_ns = node.mono_ns()
    node.spin_for(2.4)
    false_ns = next((t for t, value in node.collision_valid_samples if t >= last_observation_ns and not value), None)
    if false_ns is None:
        raise RuntimeError("collision validity did not become false")
    first_zero = next(
        (sample.monotonic_ns for sample in node.vehicle_samples if sample.monotonic_ns >= false_ns and not sample.nonzero),
        None,
    )
    if first_zero is None:
        raise RuntimeError("no vehicle zero after false validity")
    later_nonzero = any(sample.monotonic_ns > first_zero and sample.nonzero for sample in node.vehicle_samples)
    if later_nonzero:
        raise RuntimeError("nonzero vehicle output observed after fault zero")
    safe_nonzero_after_false = any(t >= false_ns and nonzero(values) for t, values in node.safe_samples)
    node.set_obstacle_mode("CLEAR")
    node.spin_for(1.2)
    ok, _ = node.arm(True)
    if ok:
        raise RuntimeError("arm=true succeeded without clearing fault")
    ok, reason = node.arm(False)
    if not ok:
        raise RuntimeError(f"arm=false failed after cause removed: {reason}")
    node.spin_for(1.2)
    ok, reason = node.arm(True)
    if not ok:
        raise RuntimeError(f"new arm=true failed after clear: {reason}")
    node.wait_vehicle_nonzero()
    node.metrics.update(
        {
            "valid_false_latency_sec": (false_ns - last_observation_ns) / 1.0e9,
            "false_to_first_zero_sec": (first_zero - false_ns) / 1.0e9,
            "last_observation_to_zero_sec": (first_zero - last_observation_ns) / 1.0e9,
            "safe_nonzero_after_false": safe_nonzero_after_false,
        }
    )
    return "P4C_E2E_STALE_OBSERVATION_GATE_PASS"


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--scenario", required=True)
    args = parser.parse_args(argv)
    rclpy.init()
    node = P4CRuntimeRunner(args.out_dir, args.scenario)
    error = None
    status = "UNKNOWN"
    try:
        if args.scenario == "gate_preflight":
            status = run_gate_preflight(node)
        elif args.scenario == "diagnostic_preflight":
            status = run_diagnostic_preflight(node)
        elif args.scenario == "duplicate_output_attribution_preflight":
            status = run_duplicate_output_attribution_preflight(node)
        elif args.scenario.startswith("fault_"):
            case = args.scenario.removeprefix("fault_")
            status = run_fault_recovery(node) if case == "recovery" else run_fault_case(node, case)
        elif args.scenario in {"validity_scan", "validity_points"}:
            status = run_validity_preflight(node, args.scenario)
        elif args.scenario in {"validity_wrong_frame", "validity_lifecycle_inactive", "validity_source_mismatch"}:
            case = args.scenario.removeprefix("validity_")
            status = run_validity_mismatch(node, case)
        elif args.scenario == "e2e_stale":
            status = run_e2e_stale(node)
        else:
            raise RuntimeError(f"unsupported scenario {args.scenario}")
    except Exception as exc:  # noqa: BLE001 - preserve failed scenario evidence.
        error = repr(exc)
        status = "FAILED"
    finally:
        metrics = node.write_metrics(status, error)
        print(json.dumps(metrics, indent=2, sort_keys=True))
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if error is None else 1


def main(argv: list[str] | None = None) -> None:
    raise SystemExit(run(argv))


if __name__ == "__main__":
    main()
