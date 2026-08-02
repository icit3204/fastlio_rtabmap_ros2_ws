"""Topology identity, migration, and sparse route helpers for plan_nav.

This module is intentionally ROS-independent. P3-B.1 establishes persistent
topology identity only; it does not publish RouteMission or connect to the
Mission Manager.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable


SCHEMA_VERSION = 2
TOOL_VERSION = "p3b1-topology-identity-v1"
MANIFEST_NAME = "topology_manifest.json"
EDGE_ID_PREFIX = "edge-"
MAP_FRAME = "map"
DEFAULT_MIN_WAYPOINT_SEPARATION_M = 0.55


@dataclass(frozen=True)
class EdgeRecord:
    edge_id: str
    from_id: int
    to_id: int
    length_m: float
    direction: str
    traj_file: str = ""


@dataclass(frozen=True)
class TopologyNode:
    node_id: int
    label: str
    x: float
    y: float
    z: float
    yaw: float
    timestamp: float
    traj_idx: int


@dataclass(frozen=True)
class RouteSpec:
    header_frame_id: str
    mission_id: str
    route_id: str
    topology_version: str
    node_ids: list[str]
    edge_ids: list[str]
    edge_directions: list[int]
    poses: list[dict]


@dataclass(frozen=True)
class RouteBuildResult:
    valid: bool
    reason_code: str
    detail: str
    route: RouteSpec | None = None


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_json(data) -> bytes:
    return json.dumps(data, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    existing_mode = path.stat().st_mode if path.exists() else None
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    tmp = Path(tmp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        if existing_mode is not None:
            os.chmod(tmp, existing_mode)
        os.replace(tmp, path)
    except Exception:
        try:
            tmp.unlink()
        except FileNotFoundError:
            pass
        raise


def parse_nodes(path: Path) -> list[TopologyNode]:
    nodes: list[TopologyNode] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 9:
            nodes.append(
                TopologyNode(
                    node_id=int(parts[0]),
                    label=parts[1],
                    x=float(parts[3]),
                    y=float(parts[4]),
                    z=float(parts[5]),
                    yaw=math.radians(float(parts[6])),
                    timestamp=float(parts[7]),
                    traj_idx=int(parts[8]),
                )
            )
        elif len(parts) == 8:
            nodes.append(
                TopologyNode(
                    node_id=int(parts[0]),
                    label=parts[1],
                    x=float(parts[2]),
                    y=float(parts[3]),
                    z=float(parts[4]),
                    yaw=math.radians(float(parts[5])),
                    timestamp=float(parts[6]),
                    traj_idx=int(parts[7]),
                )
            )
        elif len(parts) >= 7:
            nodes.append(
                TopologyNode(
                    node_id=int(parts[0]),
                    label=parts[1],
                    x=float(parts[2]),
                    y=float(parts[3]),
                    z=float(parts[4]),
                    yaw=math.radians(float(parts[5])),
                    timestamp=float(parts[6]),
                    traj_idx=-1,
                )
            )
        else:
            raise ValueError(f"malformed node row: {line}")
    return nodes


def parse_edges(path: Path) -> tuple[list[EdgeRecord], bool]:
    edges: list[EdgeRecord] = []
    legacy = False
    record_index = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        record_index += 1
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 6 and parts[0].startswith(EDGE_ID_PREFIX):
            edge = EdgeRecord(
                edge_id=parts[0],
                from_id=int(parts[1]),
                to_id=int(parts[2]),
                length_m=float(parts[3]),
                direction=parts[4],
                traj_file=parts[5] if len(parts) >= 6 else "",
            )
        elif len(parts) >= 4:
            legacy = True
            edge = EdgeRecord(
                edge_id=format_edge_id(record_index),
                from_id=int(parts[0]),
                to_id=int(parts[1]),
                length_m=float(parts[2]),
                direction=parts[3],
                traj_file=parts[4] if len(parts) >= 5 else "",
            )
        else:
            raise ValueError(f"malformed edge row: {line}")
        edges.append(edge)
    return edges, legacy


def format_edge_id(index: int) -> str:
    return f"{EDGE_ID_PREFIX}{index:06d}"


def edge_to_topology_dict(edge) -> dict:
    return {
        "edge_id": str(edge.get("edge_id", "")),
        "from_id": int(edge["from_id"]),
        "to_id": int(edge["to_id"]),
        "length_m": float(edge["length"]),
        "direction": str(edge["direction"]),
        "traj_file": str(edge.get("traj_file", "") or ""),
    }


def next_edge_id(edges: Iterable[dict | EdgeRecord]) -> str:
    max_id = 0
    for edge in edges:
        edge_id = edge.edge_id if isinstance(edge, EdgeRecord) else str(edge.get("edge_id", ""))
        if edge_id.startswith(EDGE_ID_PREFIX):
            suffix = edge_id[len(EDGE_ID_PREFIX):]
            if suffix.isdigit():
                max_id = max(max_id, int(suffix))
    return format_edge_id(max_id + 1)


def edge_id_index(edge_id: str) -> int:
    if edge_id.startswith(EDGE_ID_PREFIX):
        suffix = edge_id[len(EDGE_ID_PREFIX):]
        if suffix.isdigit():
            return int(suffix)
    return 0


def allocate_edge_id(work_dir: Path, edges: Iterable[dict | EdgeRecord]) -> str:
    existing = read_manifest(work_dir)
    if existing and int(existing.get("edge_id_next_index", 0)) > 0:
        return format_edge_id(int(existing["edge_id_next_index"]))
    return next_edge_id(edges)


def validate_topology(nodes: list[TopologyNode], edges: list[EdgeRecord], work_dir: Path) -> list[str]:
    errors: list[str] = []
    node_ids = [node.node_id for node in nodes]
    node_set = set(node_ids)
    if len(node_ids) != len(node_set):
        errors.append("DUPLICATE_NODE_ID")
    edge_ids = [edge.edge_id for edge in edges]
    if any(not edge_id for edge_id in edge_ids):
        errors.append("EMPTY_EDGE_ID")
    if len(edge_ids) != len(set(edge_ids)):
        errors.append("DUPLICATE_EDGE_ID")
    traj_files = [edge.traj_file for edge in edges if edge.traj_file]
    if len(traj_files) != len(set(traj_files)):
        errors.append("DUPLICATE_TRAJECTORY_FILE")
    for edge in edges:
        if edge.from_id not in node_set or edge.to_id not in node_set:
            errors.append(f"MISSING_ENDPOINT:{edge.edge_id}")
        if edge.direction not in {"uni", "bi"}:
            errors.append(f"INVALID_DIRECTION:{edge.edge_id}")
        if edge.traj_file and not (work_dir / edge.traj_file).exists():
            errors.append(f"MISSING_TRAJECTORY:{edge.edge_id}:{edge.traj_file}")
    return errors


def trajectory_sha256(work_dir: Path, traj_file: str) -> str:
    if not traj_file:
        return ""
    return _sha256_bytes((work_dir / traj_file).read_bytes())


def calculate_topology_version(nodes: list[TopologyNode], edges: list[EdgeRecord], work_dir: Path) -> str:
    data = {
        "nodes": [
            {
                "node_id": n.node_id,
                "x": round(n.x, 6),
                "y": round(n.y, 6),
                "z": round(n.z, 6),
                "yaw": round(n.yaw, 9),
            }
            for n in sorted(nodes, key=lambda item: item.node_id)
        ],
        "edges": [
            {
                "edge_id": e.edge_id,
                "from_id": e.from_id,
                "to_id": e.to_id,
                "length_m": round(e.length_m, 6),
                "direction": e.direction,
                "traj_file": e.traj_file,
                "traj_sha256": trajectory_sha256(work_dir, e.traj_file),
            }
            for e in sorted(edges, key=lambda item: item.edge_id)
        ],
    }
    return "sha256:" + _sha256_bytes(_canonical_json(data))


def calculate_trajectory_manifest_sha256(edges: list[EdgeRecord], work_dir: Path) -> str:
    entries = [
        {"traj_file": edge.traj_file, "sha256": trajectory_sha256(work_dir, edge.traj_file)}
        for edge in sorted(edges, key=lambda item: item.edge_id)
        if edge.traj_file
    ]
    return _sha256_bytes(_canonical_json(entries))


def topology_id_from_legacy(nodes: list[TopologyNode], edges: list[EdgeRecord]) -> str:
    data = {
        "node_ids": [node.node_id for node in sorted(nodes, key=lambda item: item.node_id)],
        "legacy_edges": [
            {
                "record": i,
                "from_id": edge.from_id,
                "to_id": edge.to_id,
                "direction": edge.direction,
                "traj_file": edge.traj_file,
            }
            for i, edge in enumerate(edges, start=1)
        ],
    }
    return "topology-sha256:" + _sha256_bytes(_canonical_json(data))


def read_manifest(work_dir: Path) -> dict | None:
    path = work_dir / MANIFEST_NAME
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def build_manifest(work_dir: Path, nodes: list[TopologyNode], edges: list[EdgeRecord], topology_id: str | None = None, generated_at: str | None = None) -> dict:
    existing = read_manifest(work_dir)
    if topology_id is None:
        topology_id = existing.get("topology_id") if existing else topology_id_from_legacy(nodes, edges)
    if generated_at is None:
        generated_at = existing.get("generated_at") if existing else "migration:p3b1"
    nodes_content_sha = _sha256_bytes((work_dir / "nodes.txt").read_bytes())
    edges_text = render_edges_v2(edges)
    max_current_edge_index = max([edge_id_index(edge.edge_id) for edge in edges] or [0]) + 1
    edge_id_next_index = max(int(existing.get("edge_id_next_index", 0)), max_current_edge_index) if existing else max_current_edge_index
    return {
        "schema_version": SCHEMA_VERSION,
        "topology_id": topology_id,
        "topology_version": calculate_topology_version(nodes, edges, work_dir),
        "node_count": len(nodes),
        "edge_count": len(edges),
        "node_identity_policy": "persisted numeric node IDs from nodes.txt; legacy delete may reindex and changes topology_version",
        "edge_identity_policy": "persisted edge_id; deleted edge IDs are not reused within this topology identity",
        "nodes_content_sha256": nodes_content_sha,
        "edges_content_sha256": _sha256_bytes(edges_text.encode("utf-8")),
        "trajectory_manifest_sha256": calculate_trajectory_manifest_sha256(edges, work_dir),
        "generated_at": generated_at,
        "generation_tool_version": TOOL_VERSION,
        "edge_id_next_index": edge_id_next_index,
    }


def render_edges_v2(edges: list[EdgeRecord]) -> str:
    lines = ["# edge_id, from_id, to_id, length_m, direction, traj_file"]
    for edge in edges:
        lines.append(
            f"{edge.edge_id}, {edge.from_id}, {edge.to_id}, "
            f"{edge.length_m}, {edge.direction}, {edge.traj_file}"
        )
    return "\n".join(lines) + "\n"


def write_manifest_atomic(work_dir: Path, manifest: dict) -> None:
    atomic_write_text(work_dir / MANIFEST_NAME, json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def write_edges_atomic(work_dir: Path, edges: list[EdgeRecord]) -> None:
    atomic_write_text(work_dir / "edges.txt", render_edges_v2(edges))


def load_topology(work_dir: Path) -> tuple[list[TopologyNode], list[EdgeRecord], bool]:
    nodes = parse_nodes(work_dir / "nodes.txt")
    edges, legacy = parse_edges(work_dir / "edges.txt")
    return nodes, edges, legacy


def migration_report(work_dir: Path) -> dict:
    nodes, edges, legacy = load_topology(work_dir)
    errors = validate_topology(nodes, edges, work_dir)
    manifest = build_manifest(work_dir, nodes, edges)
    pair_counts: dict[tuple[int, int], int] = {}
    for edge in edges:
        pair_counts[(edge.from_id, edge.to_id)] = pair_counts.get((edge.from_id, edge.to_id), 0) + 1
    ambiguous_pairs = sorted([f"{a}->{b}" for (a, b), count in pair_counts.items() if count > 1])
    close_pairs = []
    node_by_id = {node.node_id: node for node in nodes}
    for edge in edges:
        a = node_by_id.get(edge.from_id)
        b = node_by_id.get(edge.to_id)
        if a and b:
            d = math.hypot(a.x - b.x, a.y - b.y)
            if d < DEFAULT_MIN_WAYPOINT_SEPARATION_M:
                close_pairs.append({"edge_id": edge.edge_id, "from_id": edge.from_id, "to_id": edge.to_id, "distance_m": d})
    return {
        "work_dir": str(work_dir),
        "legacy_format": legacy,
        "schema_version_after_migration": SCHEMA_VERSION,
        "proposed_topology_id": manifest["topology_id"],
        "proposed_topology_version": manifest["topology_version"],
        "edge_id_mapping": [
            {"record_index": i, "edge_id": edge.edge_id, "from_id": edge.from_id, "to_id": edge.to_id}
            for i, edge in enumerate(edges, start=1)
        ],
        "errors": errors,
        "ambiguous_directed_pairs": ambiguous_pairs,
        "close_waypoint_transitions": close_pairs,
        "node_count": len(nodes),
        "edge_count": len(edges),
        "manifest": manifest,
    }


def migrate_apply(work_dir: Path) -> dict:
    report = migration_report(work_dir)
    if report["errors"]:
        return report
    nodes, edges, _ = load_topology(work_dir)
    write_edges_atomic(work_dir, edges)
    manifest = build_manifest(work_dir, nodes, edges, topology_id=report["proposed_topology_id"])
    write_manifest_atomic(work_dir, manifest)
    return migration_report(work_dir)


def resolve_traversal_edges(path_node_ids: list[int], edges: list[EdgeRecord]) -> tuple[bool, str, str, list[str], list[int]]:
    edge_ids: list[str] = []
    directions: list[int] = []
    for a, b in zip(path_node_ids, path_node_ids[1:]):
        candidates: list[tuple[EdgeRecord, int]] = []
        for edge in edges:
            if edge.from_id == a and edge.to_id == b:
                candidates.append((edge, 1))
            elif edge.from_id == b and edge.to_id == a and edge.direction == "bi":
                candidates.append((edge, -1))
        if not candidates:
            return False, "ROUTE_EDGE_NOT_FOUND", f"no eligible edge for {a}->{b}", [], []
        if len(candidates) > 1:
            ids = ",".join(edge.edge_id for edge, _ in candidates)
            return False, "AMBIGUOUS_ROUTE_EDGE", f"multiple eligible edges for {a}->{b}: {ids}", [], []
        edge, direction = candidates[0]
        edge_ids.append(edge.edge_id)
        directions.append(direction)
    return True, "ROUTE_EDGES_RESOLVED", "route edges resolved", edge_ids, directions


def _yaw_to_quaternion(yaw: float) -> dict:
    half = yaw / 2.0
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def build_sparse_route_spec(
    *,
    mission_id: str,
    ordered_node_ids: list[int],
    nodes: list[TopologyNode],
    edges: list[EdgeRecord],
    topology_manifest: dict,
    min_waypoint_separation_m: float = DEFAULT_MIN_WAYPOINT_SEPARATION_M,
) -> RouteBuildResult:
    if not mission_id:
        return RouteBuildResult(False, "EMPTY_MISSION_ID", "mission_id is required")
    if not ordered_node_ids:
        return RouteBuildResult(False, "ZERO_ROUTE_NODES", "route must contain at least one node")
    node_by_id = {node.node_id: node for node in nodes}
    missing = [node_id for node_id in ordered_node_ids if node_id not in node_by_id]
    if missing:
        return RouteBuildResult(False, "ROUTE_NODE_NOT_FOUND", f"missing node IDs: {missing}")
    for a_id, b_id in zip(ordered_node_ids, ordered_node_ids[1:]):
        a = node_by_id[a_id]
        b = node_by_id[b_id]
        if math.hypot(a.x - b.x, a.y - b.y) < min_waypoint_separation_m:
            return RouteBuildResult(False, "CONSECUTIVE_WAYPOINTS_TOO_CLOSE", f"{a_id}->{b_id} below {min_waypoint_separation_m} m")
    ok, reason, detail, edge_ids, edge_directions = resolve_traversal_edges(ordered_node_ids, edges)
    if not ok:
        return RouteBuildResult(False, reason, detail)
    topology_version = str(topology_manifest["topology_version"])
    route_hash = _sha256_bytes(_canonical_json({
        "topology_version": topology_version,
        "node_ids": [str(node_id) for node_id in ordered_node_ids],
        "edge_ids": edge_ids,
        "edge_directions": edge_directions,
    }))
    poses = []
    for node_id in ordered_node_ids:
        node = node_by_id[node_id]
        q = _yaw_to_quaternion(node.yaw)
        poses.append({
            "frame_id": MAP_FRAME,
            "position": {"x": node.x, "y": node.y, "z": node.z},
            "orientation": q,
        })
    route = RouteSpec(
        header_frame_id=MAP_FRAME,
        mission_id=mission_id,
        route_id=f"route-sha256:{route_hash}",
        topology_version=topology_version,
        node_ids=[str(node_id) for node_id in ordered_node_ids],
        edge_ids=edge_ids,
        edge_directions=edge_directions,
        poses=poses,
    )
    return RouteBuildResult(True, "ROUTE_SPEC_BUILT", "route specification built", route)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check or apply plan_nav topology identity migration")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--check", action="store_true")
    group.add_argument("--apply", action="store_true")
    parser.add_argument("--work-dir", required=True)
    parser.add_argument("--report", required=False)
    args = parser.parse_args(argv)
    work_dir = Path(args.work_dir)
    report = migrate_apply(work_dir) if args.apply else migration_report(work_dir)
    output = json.dumps(report, indent=2, sort_keys=True)
    if args.report:
        atomic_write_text(Path(args.report), output + "\n")
    else:
        print(output)
    return 1 if report.get("errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
