import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.topology_identity import load_topology
from tools.reconcile_rtab_topology import reconcile, transform_xy


def _write_fixture(tmp_path: Path):
    source = tmp_path / "source"
    source.mkdir(parents=True)
    (source / "nodes.txt").write_text(
        "# node_id, label, annotation, x, y, z, yaw_deg, timestamp_unix, traj_idx\n"
        "1, WP-01, , 0.0000, 0.0000, 0.0000, 0.00, 1.000, 0\n"
        "2, WP-02, , 2.0000, 0.0000, 0.0000, 0.00, 2.000, 1\n"
        "3, WP-03, , 4.0000, 0.0000, 0.0000, 0.00, 3.000, 2\n"
        "4, WP-04, , -2.0000, 0.0000, 0.0000, 0.00, 0.000, 0\n",
        encoding="utf-8",
    )
    (source / "edges.txt").write_text(
        "# from_id, to_id, length_m, direction, traj_file\n"
        "4, 1, 2.0, uni, edge_1_traj.txt\n"
        "1, 2, 2.0, uni, edge_2_traj.txt\n",
        encoding="utf-8",
    )
    for name, x0, x1 in (("edge_1_traj.txt", -2.0, 0.0), ("edge_2_traj.txt", 0.0, 2.0)):
        (source / name).write_text(
            "# x, y, z, yaw_rad, timestamp_unix\n"
            f"{x0}, 0.0, 0.0, 0.0, 1.0\n{x1}, 0.0, 0.0, 0.0, 2.0\n",
            encoding="utf-8",
        )
    source_db = tmp_path / "source.db"
    target_db = tmp_path / "target.db"
    source_db.write_bytes(b"same immutable database")
    target_db.write_bytes(source_db.read_bytes())
    digest = hashlib.sha256(target_db.read_bytes()).hexdigest()
    return source, source_db, target_db, digest


def _args(tmp_path: Path):
    source, source_db, target_db, digest = _write_fixture(tmp_path)
    return argparse.Namespace(
        source_workspace=str(source), source_database=str(source_db),
        target_workspace=str(tmp_path / "target"), target_database=str(target_db),
        target_database_sha256=digest, theta_rad=math.pi / 2.0,
        tx_m=1.0, ty_m=-2.0, node_correspondences=4,
        node_rms_residual_m=0.01, node_max_residual_m=0.02,
        trajectory_correspondences=4, trajectory_rms_residual_m=0.01,
        trajectory_max_residual_m=0.02,
    )


def test_reconciliation_preserves_source_and_persists_identity(tmp_path):
    args = _args(tmp_path)
    source_nodes_before = (Path(args.source_workspace) / "nodes.txt").read_bytes()
    source_edges_before = (Path(args.source_workspace) / "edges.txt").read_bytes()
    result = reconcile(args)
    assert (Path(args.source_workspace) / "nodes.txt").read_bytes() == source_nodes_before
    assert (Path(args.source_workspace) / "edges.txt").read_bytes() == source_edges_before
    nodes, edges, legacy = load_topology(Path(args.target_workspace))
    assert not legacy
    assert [node.node_id for node in nodes] == [1, 2, 3, 4]
    assert [edge.edge_id for edge in edges] == ["edge-000001", "edge-000002"]
    assert result["offline_route_probe"]["frame"] == "map"
    assert result["offline_route_probe"]["node_ids"] == ["4", "1", "2"]


def test_reconciliation_transforms_nodes_and_trajectory_without_scale(tmp_path):
    args = _args(tmp_path)
    reconcile(args)
    nodes, edges, _ = load_topology(Path(args.target_workspace))
    by_id = {node.node_id: node for node in nodes}
    assert math.isclose(by_id[1].x, 1.0, abs_tol=1e-4)
    assert math.isclose(by_id[1].y, -2.0, abs_tol=1e-4)
    assert math.isclose(by_id[2].x, 1.0, abs_tol=1e-4)
    assert math.isclose(by_id[2].y, 0.0, abs_tol=1e-4)
    assert [edge.length_m for edge in edges] == [2.0, 2.0]
    data = (Path(args.target_workspace) / "edge_2_traj.txt").read_text()
    assert "1.0000, 0.0000" in data


def test_reconciliation_refuses_hash_mismatch_and_overwrite(tmp_path):
    args = _args(tmp_path)
    args.target_database_sha256 = "0" * 64
    try:
        reconcile(args)
    except ValueError as exc:
        assert "hash mismatch" in str(exc)
    else:
        raise AssertionError("hash mismatch was accepted")

    args = _args(tmp_path / "second")
    Path(args.target_workspace).mkdir()
    try:
        reconcile(args)
    except FileExistsError:
        pass
    else:
        raise AssertionError("existing target was overwritten")


def test_transform_geometry_is_rigid():
    a = transform_xy(0.0, 0.0, 0.37, -2.0, 4.0)
    b = transform_xy(3.0, 4.0, 0.37, -2.0, 4.0)
    assert math.isclose(math.dist(a, b), 5.0, abs_tol=1e-12)
