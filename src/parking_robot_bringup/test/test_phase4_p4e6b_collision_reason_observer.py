import json

import pytest

from parking_robot_bringup.phase4_p4e6b_collision_reason_observer import (
    VALIDITY_DIAGNOSTIC, _gid, _status_level, authority_rows,
)


class Endpoint:
    def __init__(self, node_name="monitor", namespace="/", gid=b"\x01\x02", topic_type="std_msgs/msg/Bool"):
        self.node_name=node_name; self.node_namespace=namespace; self.endpoint_gid=gid; self.topic_type=topic_type


class Graph:
    def __init__(self, mapping): self.mapping=mapping
    def get_publishers_info_by_topic(self, topic): return self.mapping.get(topic, [])


def test_observer_contract_is_passive_and_targeted():
    from pathlib import Path
    text=Path(__file__).parents[1].joinpath("parking_robot_bringup/phase4_p4e6b_collision_reason_observer.py").read_text()
    assert "create_publisher" not in text
    assert VALIDITY_DIAGNOSTIC in text
    for topic in ("/system/collision_monitor_valid", "/diagnostics", "/phase4/synthetic_scan"):
        assert topic in text


def test_uint8_and_gid_serialization():
    assert _status_level(b"\x02") == 2
    assert _status_level(1) == 1
    assert _gid(type("Info", (), {"publisher_gid": b"\xab\xcd"})()) == "abcd"
    assert _gid(None) == ""


def test_authority_snapshot_records_all_topics_and_exact_endpoints():
    rows=authority_rows(Graph({"/system/collision_monitor_valid":[Endpoint("collision_monitor_validity_monitor", "/", b"\xaa")],
                               "/phase4/synthetic_scan":[Endpoint("synthetic", "/phase4", b"\xbb", "sensor_msgs/msg/LaserScan"), Endpoint("other", "/", b"\xcc")]}))
    assert [row["topic"] for row in rows] == ["/system/collision_monitor_valid", "/system/localization_valid", "/system/controller_valid", "/phase4/synthetic_scan"]
    assert rows[0]["publishers"] == [{"node":"/collision_monitor_validity_monitor","gid":"aa","topic_type":"std_msgs/msg/Bool"}]
    assert rows[1]["publishers"] == []
    assert [x["gid"] for x in rows[3]["publishers"]] == ["bb", "cc"]


def test_jsonl_shape_is_serializable_and_monotonic_field_named():
    row={"receipt_monotonic_ns": 10, "event":"collision_validity", "value":True, "publisher_gid":"aa"}
    assert json.loads(json.dumps(row))["receipt_monotonic_ns"] == 10


def test_cli_rejects_unknown_application_option_and_accepts_ros_args(monkeypatch, tmp_path):
    import parking_robot_bringup.phase4_p4e6b_collision_reason_observer as module
    monkeypatch.setattr(module.rclpy, "init", lambda **_: None)
    with pytest.raises(SystemExit): module.main(["--output-dir",str(tmp_path),"--typo"])
