from pathlib import Path
import json
import math
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.pathfinder import build_graph, concat_trajectory_segments, find_path
from core.topology import TopologyManager
from core.topology_identity import (
    EdgeRecord,
    SCHEMA_VERSION,
    build_manifest,
    build_sparse_route_spec,
    calculate_topology_version,
    load_topology,
    migrate_apply,
    migration_report,
    parse_edges,
    parse_nodes,
    resolve_traversal_edges,
)


def write_nodes(work_dir: Path, rows=None):
    rows = rows or [
        "1, WP-01, note one, 0.0000, 0.0000, 0.0000, 0.00, 1.000, 0",
        "2, WP-02, note two, 1.0000, 0.0000, 0.0000, 0.00, 2.000, 1",
        "3, WP-03, note three, 2.0000, 0.0000, 0.0000, 0.00, 3.000, 2",
    ]
    (work_dir / "nodes.txt").write_text(
        "# node_id, label, annotation, x, y, z, yaw_deg, timestamp_unix, traj_idx\n"
        + "\n".join(rows)
        + "\n",
        encoding="utf-8",
    )


def write_edges(work_dir: Path, text: str):
    (work_dir / "edges.txt").write_text(text, encoding="utf-8")


def write_trajs(work_dir: Path):
    for idx in range(1, 5):
        (work_dir / f"edge_{idx}_traj.txt").write_text(
            "# x, y, z, yaw_rad, timestamp_unix\n"
            f"{idx - 1}.0000, 0.0000, 0.0000, 0.0000, 1.000\n"
            f"{idx}.0000, 0.0000, 0.0000, 0.0000, 2.000\n",
            encoding="utf-8",
        )


def sample_legacy_topology(tmp_path: Path) -> Path:
    write_nodes(tmp_path)
    write_edges(
        tmp_path,
        "# from_id, to_id, length_m, direction, traj_file\n"
        "1, 2, 1.0, uni, edge_1_traj.txt\n"
        "2, 3, 1.0, bi, edge_2_traj.txt\n",
    )
    write_trajs(tmp_path)
    return tmp_path


def test_legacy_four_and_five_column_edges_load_in_check_mode(tmp_path):
    write_nodes(tmp_path)
    write_edges(tmp_path, "1, 2, 1.0, uni\n2, 3, 1.0, bi, edge_2_traj.txt\n")
    write_trajs(tmp_path)
    report = migration_report(tmp_path)
    assert report["legacy_format"] is True
    assert report["edge_id_mapping"][0]["edge_id"] == "edge-000001"
    assert report["edge_id_mapping"][1]["edge_id"] == "edge-000002"
    assert not report["errors"]


def test_version_two_six_column_edges_load(tmp_path):
    write_nodes(tmp_path)
    write_edges(
        tmp_path,
        "# edge_id, from_id, to_id, length_m, direction, traj_file\n"
        "edge-000010, 1, 2, 1.0, uni, edge_1_traj.txt\n",
    )
    write_trajs(tmp_path)
    edges, legacy = parse_edges(tmp_path / "edges.txt")
    assert legacy is False
    assert edges[0].edge_id == "edge-000010"


def test_deterministic_migration_and_idempotence(tmp_path):
    sample_legacy_topology(tmp_path)
    check = migration_report(tmp_path)
    assert check["edge_id_mapping"][0]["edge_id"] == "edge-000001"
    assert migrate_apply(tmp_path)["errors"] == []
    first_edges = (tmp_path / "edges.txt").read_text()
    first_manifest = (tmp_path / "topology_manifest.json").read_text()
    assert migrate_apply(tmp_path)["errors"] == []
    assert (tmp_path / "edges.txt").read_text() == first_edges
    assert (tmp_path / "topology_manifest.json").read_text() == first_manifest


def test_duplicate_edge_id_and_missing_endpoint_rejected(tmp_path):
    write_nodes(tmp_path)
    write_edges(
        tmp_path,
        "edge-000001, 1, 2, 1.0, uni, edge_1_traj.txt\n"
        "edge-000001, 2, 99, 1.0, uni, edge_2_traj.txt\n",
    )
    write_trajs(tmp_path)
    report = migration_report(tmp_path)
    assert "DUPLICATE_EDGE_ID" in report["errors"]
    assert any(item.startswith("MISSING_ENDPOINT") for item in report["errors"])


def test_new_edge_unique_id_deleted_id_not_reused_and_toggle_preserves_id(tmp_path):
    sample_legacy_topology(tmp_path)
    migrate_apply(tmp_path)
    topo = TopologyManager(str(tmp_path))
    topo.load_all()
    topo.add_edge(1, 3, "uni")
    new_edge_id = topo.edges[-1]["edge_id"]
    assert new_edge_id == "edge-000003"
    topo.remove_edge(1, 3)
    topo.add_edge(1, 3, "uni")
    assert topo.edges[-1]["edge_id"] == "edge-000004"
    topo.toggle_direction(1, 2)
    assert next(e for e in topo.edges if e["from_id"] == 1 and e["to_id"] == 2)["edge_id"] == "edge-000001"


def test_atomic_write_failure_preserves_old_file(monkeypatch, tmp_path):
    sample_legacy_topology(tmp_path)
    old = (tmp_path / "edges.txt").read_text()
    import core.topology_identity as identity

    def fail_replace(src, dst):
        raise OSError("forced")

    monkeypatch.setattr(identity.os, "replace", fail_replace)
    with pytest.raises(OSError):
        identity.write_edges_atomic(tmp_path, [EdgeRecord("edge-000001", 1, 2, 1.0, "uni", "edge_1_traj.txt")])
    assert (tmp_path / "edges.txt").read_text() == old


def test_topology_version_stability_and_navigation_changes(tmp_path):
    sample_legacy_topology(tmp_path)
    migrate_apply(tmp_path)
    nodes, edges, _ = load_topology(tmp_path)
    v1 = calculate_topology_version(nodes, edges, tmp_path)
    v2 = calculate_topology_version(nodes, edges, tmp_path)
    assert v1 == v2
    write_nodes(tmp_path, [
        "1, WP-01, changed annotation, 0.0000, 0.0000, 0.0000, 0.00, 1.000, 0",
        "2, WP-02, note two, 1.0000, 0.0000, 0.0000, 0.00, 2.000, 1",
        "3, WP-03, note three, 2.0000, 0.0000, 0.0000, 0.00, 3.000, 2",
    ])
    assert calculate_topology_version(parse_nodes(tmp_path / "nodes.txt"), edges, tmp_path) == v1
    write_nodes(tmp_path, [
        "1, WP-01, changed annotation, 0.1000, 0.0000, 0.0000, 0.00, 1.000, 0",
        "2, WP-02, note two, 1.0000, 0.0000, 0.0000, 0.00, 2.000, 1",
        "3, WP-03, note three, 2.0000, 0.0000, 0.0000, 0.00, 3.000, 2",
    ])
    assert calculate_topology_version(parse_nodes(tmp_path / "nodes.txt"), edges, tmp_path) != v1

    sample_legacy_topology(tmp_path)
    migrate_apply(tmp_path)
    nodes, edges, _ = load_topology(tmp_path)
    v1 = calculate_topology_version(nodes, edges, tmp_path)
    write_edges(tmp_path, "edge-000001, 1, 2, 1.0, bi, edge_1_traj.txt\nedge-000002, 2, 3, 1.0, bi, edge_2_traj.txt\n")
    assert calculate_topology_version(nodes, parse_edges(tmp_path / "edges.txt")[0], tmp_path) != v1
    sample_legacy_topology(tmp_path)
    migrate_apply(tmp_path)
    nodes, edges, _ = load_topology(tmp_path)
    v1 = calculate_topology_version(nodes, edges, tmp_path)
    (tmp_path / "edge_1_traj.txt").write_text("changed\n", encoding="utf-8")
    assert calculate_topology_version(nodes, edges, tmp_path) != v1


def test_traversal_resolution_rules():
    edges = [
        EdgeRecord("edge-a", 1, 2, 1.0, "uni", ""),
        EdgeRecord("edge-b", 2, 3, 1.0, "bi", ""),
    ]
    assert resolve_traversal_edges([1, 2], edges)[3:] == (["edge-a"], [1])
    assert resolve_traversal_edges([3, 2], edges)[3:] == (["edge-b"], [-1])
    ok, reason, *_ = resolve_traversal_edges([2, 1], edges)
    assert not ok and reason == "ROUTE_EDGE_NOT_FOUND"
    ok, reason, *_ = resolve_traversal_edges([1, 2], edges + [EdgeRecord("edge-c", 1, 2, 1.1, "uni", "")])
    assert not ok and reason == "AMBIGUOUS_ROUTE_EDGE"


def test_sparse_route_spec_invariants_route_id_and_spacing(tmp_path):
    sample_legacy_topology(tmp_path)
    migrate_apply(tmp_path)
    nodes, edges, _ = load_topology(tmp_path)
    manifest = json.loads((tmp_path / "topology_manifest.json").read_text())
    result = build_sparse_route_spec(
        mission_id="mission-1",
        ordered_node_ids=[1, 2, 3],
        nodes=nodes,
        edges=edges,
        topology_manifest=manifest,
    )
    assert result.valid
    route = result.route
    assert route.header_frame_id == "map"
    assert route.node_ids == ["1", "2", "3"]
    assert route.edge_ids == ["edge-000001", "edge-000002"]
    assert route.edge_directions == [1, 1]
    assert route.route_id.startswith("route-sha256:")
    again = build_sparse_route_spec(mission_id="different", ordered_node_ids=[1, 2, 3], nodes=nodes, edges=edges, topology_manifest=manifest)
    assert again.route.route_id == route.route_id
    assert len(route.poses) == len(route.node_ids)
    assert len(route.edge_ids) == len(route.node_ids) - 1

    write_nodes(tmp_path, [
        "1, WP-01, , 0.0000, 0.0000, 0.0000, 0.00, 1.000, 0",
        "2, WP-02, , 0.1000, 0.0000, 0.0000, 0.00, 2.000, 1",
    ])
    write_edges(tmp_path, "edge-000001, 1, 2, 0.1, uni, edge_1_traj.txt\n")
    nodes, edges, _ = load_topology(tmp_path)
    manifest = build_manifest(tmp_path, nodes, edges)
    result = build_sparse_route_spec(mission_id="m", ordered_node_ids=[1, 2], nodes=nodes, edges=edges, topology_manifest=manifest)
    assert not result.valid
    assert result.reason_code == "CONSECUTIVE_WAYPOINTS_TOO_CLOSE"


def test_existing_dijkstra_and_dense_path_regression(tmp_path):
    sample_legacy_topology(tmp_path)
    topo = TopologyManager(str(tmp_path))
    topo.load_all()
    before_graph = build_graph(topo.waypoints, topo.edges)
    before_path, before_len = find_path(before_graph, 1, 3)
    wp = {item["id"]: item for item in topo.waypoints}
    before_dense = concat_trajectory_segments([wp[i] for i in before_path], topo.edges, topo)

    migrate_apply(tmp_path)
    topo = TopologyManager(str(tmp_path))
    topo.load_all()
    after_graph = build_graph(topo.waypoints, topo.edges)
    after_path, after_len = find_path(after_graph, 1, 3)
    wp = {item["id"]: item for item in topo.waypoints}
    after_dense = concat_trajectory_segments([wp[i] for i in after_path], topo.edges, topo)

    assert after_path == before_path
    assert after_len == before_len
    assert after_dense == before_dense


def test_migration_tool_cli_check_and_apply(tmp_path):
    sample_legacy_topology(tmp_path)
    tool = ROOT / "tools" / "migrate_topology_identity.py"
    check_report = tmp_path / "check.json"
    apply_report = tmp_path / "apply.json"
    subprocess.run([sys.executable, str(tool), "--check", "--work-dir", str(tmp_path), "--report", str(check_report)], check=True)
    assert json.loads(check_report.read_text())["legacy_format"] is True
    subprocess.run([sys.executable, str(tool), "--apply", "--work-dir", str(tmp_path), "--report", str(apply_report)], check=True)
    report = json.loads(apply_report.read_text())
    assert report["schema_version_after_migration"] == SCHEMA_VERSION
    assert (tmp_path / "topology_manifest.json").exists()
