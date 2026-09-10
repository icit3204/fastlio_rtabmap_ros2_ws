#!/usr/bin/env python3
"""Create a map-frame topology copy using one audited rigid SE(2) transform.

This tool never edits either RTAB database or the source topology.  It refuses
to overwrite an existing target workspace and records all provenance needed to
reproduce the conversion.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.topology_identity import (  # noqa: E402
    build_manifest,
    build_sparse_route_spec,
    load_topology,
    render_edges_v2,
)


TOOL_VERSION = "p5a-mk2h1r-se2-v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def transform_xy(x: float, y: float, theta_rad: float, tx_m: float, ty_m: float) -> tuple[float, float]:
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return c * x - s * y + tx_m, s * x + c * y + ty_m


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def render_transformed_nodes(source: Path, theta_rad: float, tx_m: float, ty_m: float) -> str:
    lines = ["# node_id, label, annotation, x, y, z, yaw_deg, timestamp_unix, traj_idx"]
    for line in source.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = [item.strip() for item in line.split(",")]
        if len(fields) < 9:
            raise ValueError("canonical conversion requires the nine-column node schema")
        x, y = transform_xy(float(fields[3]), float(fields[4]), theta_rad, tx_m, ty_m)
        yaw_deg = math.degrees(normalize_angle(math.radians(float(fields[6])) + theta_rad))
        lines.append(
            f"{int(fields[0])}, {fields[1]}, {fields[2]}, {x:.4f}, {y:.4f}, "
            f"{float(fields[5]):.4f}, {yaw_deg:.2f}, {float(fields[7]):.3f}, {int(fields[8])}"
        )
    return "\n".join(lines) + "\n"


def render_transformed_trajectory(source: Path, theta_rad: float, tx_m: float, ty_m: float) -> str:
    lines = ["# x, y, z, yaw_rad, timestamp_unix"]
    for line in source.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = [item.strip() for item in line.split(",")]
        if len(fields) < 5:
            raise ValueError(f"malformed trajectory row in {source}")
        x, y = transform_xy(float(fields[0]), float(fields[1]), theta_rad, tx_m, ty_m)
        yaw = normalize_angle(float(fields[3]) + theta_rad)
        lines.append(
            f"{x:.4f}, {y:.4f}, {float(fields[2]):.4f}, {yaw:.4f}, {float(fields[4]):.3f}"
        )
    return "\n".join(lines) + "\n"


def reconcile(args: argparse.Namespace) -> dict:
    source = Path(args.source_workspace).resolve()
    target = Path(args.target_workspace).resolve()
    source_db = Path(args.source_database).resolve()
    target_db = Path(args.target_database).resolve()
    if target.exists():
        raise FileExistsError(f"refusing to overwrite existing target workspace: {target}")
    actual_target_hash = sha256_file(target_db)
    if actual_target_hash != args.target_database_sha256:
        raise ValueError(f"target database hash mismatch: {actual_target_hash}")
    actual_source_hash = sha256_file(source_db)
    if actual_source_hash != actual_target_hash:
        raise ValueError("source topology database is not byte-identical to target canonical database")

    source_nodes, source_edges, legacy = load_topology(source)
    if not source_nodes or not source_edges:
        raise ValueError("source topology must contain nodes and edges")
    for edge in source_edges:
        if edge.traj_file and not (source / edge.traj_file).is_file():
            raise FileNotFoundError(source / edge.traj_file)

    target.mkdir(parents=True)
    try:
        (target / "nodes.txt").write_text(
            render_transformed_nodes(source / "nodes.txt", args.theta_rad, args.tx_m, args.ty_m),
            encoding="utf-8",
        )
        # parse_edges() deterministically assigns legacy edge IDs by persisted
        # record order.  Persist those IDs in the new copy without touching the
        # historical source file.
        (target / "edges.txt").write_text(render_edges_v2(source_edges), encoding="utf-8")
        for edge in source_edges:
            if edge.traj_file:
                (target / edge.traj_file).write_text(
                    render_transformed_trajectory(
                        source / edge.traj_file, args.theta_rad, args.tx_m, args.ty_m
                    ),
                    encoding="utf-8",
                )

        target_nodes, target_edges, target_legacy = load_topology(target)
        if target_legacy:
            raise AssertionError("target edge IDs were not persisted")
        manifest = build_manifest(
            target,
            target_nodes,
            target_edges,
            generated_at="reconciliation:p5a-mk2h1r",
        )
        (target / "topology_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        route_probe = build_sparse_route_spec(
            mission_id="mk2h1r-offline-route-probe",
            ordered_node_ids=[4, 1, 2],
            nodes=target_nodes,
            edges=target_edges,
            topology_manifest=manifest,
        )
        if not route_probe.valid:
            raise ValueError(f"offline route probe failed: {route_probe.reason_code}")

        provenance = {
            "schema_version": 1,
            "tool_version": TOOL_VERSION,
            "disposition": "CREATE_CANONICAL_TRANSFORMED_TOPOLOGY_COPY",
            "source_workspace": str(source),
            "source_database": str(source_db),
            "source_database_sha256": actual_source_hash,
            "source_nodes_sha256": sha256_file(source / "nodes.txt"),
            "source_edges_sha256": sha256_file(source / "edges.txt"),
            "source_edge_format": "legacy-positional" if legacy else "persisted-v2",
            "target_workspace": str(target),
            "target_database": str(target_db),
            "target_database_sha256": actual_target_hash,
            "coordinate_semantics": {
                "source": "RTAB Node.pose raw odometry trajectory",
                "target": "RTAB optimized graph map frame via audited rigid SE(2) approximation",
                "z": "unchanged",
                "yaw": "source yaw plus SE(2) yaw, normalized",
                "edge_lengths": "unchanged under rigid transform",
            },
            "se2": {
                "theta_rad": args.theta_rad,
                "theta_deg": math.degrees(args.theta_rad),
                "tx_m": args.tx_m,
                "ty_m": args.ty_m,
            },
            "fit_evidence": {
                "node_correspondences": args.node_correspondences,
                "node_rms_residual_m": args.node_rms_residual_m,
                "node_max_residual_m": args.node_max_residual_m,
                "trajectory_correspondences": args.trajectory_correspondences,
                "trajectory_rms_residual_m": args.trajectory_rms_residual_m,
                "trajectory_max_residual_m": args.trajectory_max_residual_m,
            },
            "identity": {
                "node_ids_preserved": True,
                "legacy_edge_records_persisted_as": [edge.edge_id for edge in target_edges],
                "topology_id": manifest["topology_id"],
                "topology_version": manifest["topology_version"],
            },
            "offline_route_probe": {
                "frame": route_probe.route.header_frame_id,
                "node_ids": route_probe.route.node_ids,
                "edge_ids": route_probe.route.edge_ids,
                "edge_directions": route_probe.route.edge_directions,
                "result": route_probe.reason_code,
            },
        }
        (target / "reconciliation_manifest.json").write_text(
            json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return provenance
    except Exception:
        shutil.rmtree(target)
        raise


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--source-workspace", required=True)
    result.add_argument("--source-database", required=True)
    result.add_argument("--target-workspace", required=True)
    result.add_argument("--target-database", required=True)
    result.add_argument("--target-database-sha256", required=True)
    result.add_argument("--theta-rad", required=True, type=float)
    result.add_argument("--tx-m", required=True, type=float)
    result.add_argument("--ty-m", required=True, type=float)
    result.add_argument("--node-correspondences", required=True, type=int)
    result.add_argument("--node-rms-residual-m", required=True, type=float)
    result.add_argument("--node-max-residual-m", required=True, type=float)
    result.add_argument("--trajectory-correspondences", required=True, type=int)
    result.add_argument("--trajectory-rms-residual-m", required=True, type=float)
    result.add_argument("--trajectory-max-residual-m", required=True, type=float)
    return result


def main() -> int:
    args = parser().parse_args()
    print(json.dumps(reconcile(args), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
