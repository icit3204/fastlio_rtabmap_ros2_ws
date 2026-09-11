import csv
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import zlib

import numpy as np
import pytest

from core.db_loader import (
    OPTIMIZED_IMPORT_MODE,
    load_db,
    load_optimized_map_trajectory,
)
from core.optimized_workspace import (
    finalize_optimized_workspace,
    prepare_optimized_workspace,
)
from core.mission_bridge import prepare_route_spec_from_topology


REPO = Path(__file__).resolve().parents[2]
CURRENT_DB = REPO / 'map/current_room_sessions/20260911_194712_current_room/rtabmap_2d.db'
REFERENCE = Path(__file__).parent / 'data/current_room_optimized_reference.csv'
CURRENT_DB_SHA = '550227ccf65470946d78317f28f200f3dcc904dfa053291f69f5f06b67fdd127'
CURRENT_WORKSPACE = CURRENT_DB.parent / 'plannav'
CURRENT_TOPOLOGY_VERSION = 'sha256:948217a0129dbf785b4364cdff94fe5424563719495a6aeb1bfc2780814d1a2e'


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _angle_delta(left, right):
    return math.atan2(math.sin(left - right), math.cos(left - right))


def _compressed_mat(values, rows, cols, cv_type, dtype):
    raw = np.asarray(values, dtype=dtype).reshape(rows, cols).tobytes()
    return zlib.compress(raw) + struct.pack('<iii', rows, cols, cv_type)


def _make_db(path):
    transform = np.array([
        [0.0, -1.0, 0.0, 1.25],
        [1.0, 0.0, 0.0, -2.5],
        [0.0, 0.0, 1.0, 0.75],
    ], dtype=np.float32)
    raw_pose = np.array([
        [1.0, 0.0, 0.0, 99.0],
        [0.0, 1.0, 0.0, 88.0],
        [0.0, 0.0, 1.0, 77.0],
    ], dtype=np.float32)
    conn = sqlite3.connect(path)
    conn.execute('CREATE TABLE Node(id INTEGER, stamp REAL, pose BLOB)')
    conn.execute('CREATE TABLE Link(from_id INTEGER, to_id INTEGER, type INTEGER)')
    conn.execute(
        'CREATE TABLE Admin(version TEXT, opt_ids BLOB, opt_poses BLOB, '
        'opt_map BLOB, opt_map_x_min REAL, opt_map_y_min REAL, opt_map_resolution REAL)'
    )
    conn.execute('INSERT INTO Node VALUES(7, 123.5, ?)', (raw_pose.tobytes(),))
    conn.execute(
        'INSERT INTO Admin VALUES(?, ?, ?, NULL, NULL, NULL, NULL)',
        (
            '0.23.4',
            _compressed_mat([7], 1, 1, 4, np.int32),
            _compressed_mat(transform.reshape(-1), 1, 12, 5, np.float32),
        ),
    )
    conn.commit()
    conn.close()


def test_optimized_import_is_explicit_and_legacy_raw_import_is_preserved(tmp_path):
    db = tmp_path / 'sample.db'
    _make_db(db)
    optimized = load_optimized_map_trajectory(db)
    legacy = load_db(db)
    assert optimized['import_mode'] == OPTIMIZED_IMPORT_MODE
    assert optimized['frame_id'] == 'map'
    assert optimized['coordinate_semantic'] == 'OPTIMIZED_RTAB_GRAPH'
    assert optimized['map_meta']['resolution'] == pytest.approx(0.01)
    assert optimized['map_meta']['resolution_semantic'] == 'TRAJECTORY_DISPLAY_SCALE_ONLY'
    assert optimized['nodes'][0]['id'] == 7
    assert optimized['nodes'][0]['x'] == pytest.approx(1.25)
    assert optimized['nodes'][0]['y'] == pytest.approx(-2.5)
    assert optimized['nodes'][0]['yaw'] == pytest.approx(math.pi / 2)
    assert legacy['coordinate_semantic'] == 'RAW_RTAB_NODE_POSE'
    assert legacy['nodes'][0]['x'] == pytest.approx(99.0)


def test_current_room_all_poses_match_independent_native_rtab_reference():
    assert _sha(CURRENT_DB) == CURRENT_DB_SHA
    before = _sha(CURRENT_DB)
    imported = load_optimized_map_trajectory(CURRENT_DB)
    reference = list(csv.DictReader(REFERENCE.open(encoding='utf-8')))
    assert len(imported['nodes']) == len(reference) == 38
    for actual, expected in zip(imported['nodes'], reference):
        assert actual['id'] == int(expected['node_id'])
        translation_error = math.sqrt(sum(
            (actual[key] - float(expected[key])) ** 2 for key in ('x', 'y', 'z')
        ))
        assert translation_error < 1e-8
        for key in ('roll', 'pitch', 'yaw'):
            assert abs(_angle_delta(actual[key], float(expected[key]))) < 2e-7
    assert _sha(CURRENT_DB) == before


def test_workspace_is_hash_and_map_frame_bound_and_rejects_rebinding(tmp_path):
    db = tmp_path / 'room.db'
    _make_db(db)
    workspace = tmp_path / 'plannav'
    first = prepare_optimized_workspace(db, workspace, 'session-a')
    second = prepare_optimized_workspace(db, workspace, 'session-a')
    assert second == first
    assert first['database_sha256'] == _sha(db)
    assert first['coordinate_semantic'] == 'OPTIMIZED_RTAB_GRAPH'
    assert first['frame'] == 'map'
    with pytest.raises(ValueError, match='mapping_session_id'):
        prepare_optimized_workspace(db, workspace, 'different-session')


def test_workspace_finalization_records_existing_topology_identity(tmp_path):
    db = tmp_path / 'room.db'
    _make_db(db)
    workspace = tmp_path / 'plannav'
    prepare_optimized_workspace(db, workspace, 'session-a')
    (workspace / 'topology_manifest.json').write_text(
        '{"topology_id":"room-topology","topology_version":"sha256:abc",'
        '"node_count":3,"edge_count":2}\n',
        encoding='utf-8',
    )
    finalized = finalize_optimized_workspace(workspace)
    assert finalized['topology_status'] == 'OPERATOR_APPROVED'
    assert finalized['topology_version'] == 'sha256:abc'
    assert finalized['node_count'] == 3
    assert finalized['edge_count'] == 2


def test_approved_current_room_workspace_binding_and_route_contract():
    workspace_manifest = json.loads(
        (CURRENT_WORKSPACE / 'plannav_workspace_manifest.json').read_text(encoding='utf-8')
    )
    topology_manifest = json.loads(
        (CURRENT_WORKSPACE / 'topology_manifest.json').read_text(encoding='utf-8')
    )
    assert workspace_manifest['database_sha256'] == CURRENT_DB_SHA
    assert workspace_manifest['frame'] == 'map'
    assert workspace_manifest['coordinate_semantic'] == 'OPTIMIZED_RTAB_GRAPH'
    assert workspace_manifest['topology_status'] == 'OPERATOR_APPROVED'
    assert workspace_manifest['topology_version'] == CURRENT_TOPOLOGY_VERSION
    assert topology_manifest['topology_version'] == CURRENT_TOPOLOGY_VERSION
    result = prepare_route_spec_from_topology(
        work_dir=CURRENT_WORKSPACE,
        start_node_id=1,
        end_node_id=3,
        mission_id='current-room-offline-test',
    )
    assert result.valid
    assert result.path_node_ids == [1, 2, 3]
    assert result.route.header_frame_id == 'map'
    assert result.route.edge_ids == ['edge-000001', 'edge-000002']
    assert result.route.edge_directions == [1, 1]
