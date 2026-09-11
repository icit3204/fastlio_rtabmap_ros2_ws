# [DONE] F-1.1 导入 .db 文件并解析 RTAB-Map 数据库
# [DONE] F-1.3 按时间戳顺序加载轨迹位姿

import sqlite3
import numpy as np
import math
import os
import struct
import zlib
from pathlib import Path
from urllib.parse import quote


OPTIMIZED_IMPORT_MODE = 'OPTIMIZED_GRAPH_MAP_FRAME'
OPTIMIZED_IMPORTER_VERSION = 'p5a-mk2h1m-r1-v1'
CV_32SC1 = 4
CV_32FC1 = 5
TRAJECTORY_DISPLAY_RESOLUTION_M_PER_PIXEL = 0.01


def load_db(db_path: str, progress_callback=None) -> dict:
    """
    解析 RTAB-Map .db 文件，返回完整数据结构。

    返回:
    {
      'nodes': [{'id': int, 'timestamp': float, 'x': float, 'y': float,
                 'z': float, 'yaw': float, 'mat': np.ndarray}],
      'links': [{'from_id': int, 'to_id': int, 'type': int}],
      'map_blob': bytes | None,     # opt_map 原始 blob
      'map_info_blob': bytes | None, # opt_map_info 原始 blob
    }
    """
    conn = sqlite3.connect(db_path)

    # --- 读取位姿节点 (F-1.3) ---
    nodes = []
    rows = conn.execute(
        "SELECT id, stamp, pose FROM Node ORDER BY stamp ASC"
    ).fetchall()
    total = len(rows)

    for i, (node_id, stamp, pose_blob) in enumerate(rows):
        mat = _parse_pose(pose_blob)  # 4×4 变换矩阵
        x, y, z = float(mat[0, 3]), float(mat[1, 3]), float(mat[2, 3])
        yaw = math.atan2(mat[1, 0], mat[0, 0])  # Z轴旋转角
        nodes.append({
            'id': node_id,
            'timestamp': stamp,
            'x': x, 'y': y, 'z': z,
            'yaw': yaw,
            'mat': mat,
        })
        if progress_callback:
            progress_callback(int((i + 1) / total * 50))  # 前半段进度

    # --- 读取约束边 ---
    links = []
    for row in conn.execute("SELECT from_id, to_id, type FROM Link"):
        links.append({'from_id': row[0], 'to_id': row[1], 'type': row[2]})

    # --- 读取 2D 占用栅格与元数据 (F-1.2 数据源) ---
    row = conn.execute(
        "SELECT opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution "
        "FROM Admin"
    ).fetchone()
    map_blob = row[0] if row else None
    map_meta = {
        'origin_x': row[1] if row and row[1] is not None else -3.34,
        'origin_y': row[2] if row and row[2] is not None else -85.93,
        'resolution': row[3] if row and row[3] is not None else 0.05,
    }

    if progress_callback:
        progress_callback(100)

    conn.close()
    return {
        'nodes': nodes,
        'links': links,
        'map_blob': map_blob,
        'map_meta': map_meta,
        'import_mode': 'LEGACY_RAW_NODE_POSE',
        'coordinate_semantic': 'RAW_RTAB_NODE_POSE',
    }


def load_optimized_map_trajectory(db_path: str, progress_callback=None) -> dict:
    """Load RTAB-Map's saved optimized graph in canonical ROS ``map`` coordinates.

    RTAB-Map stores ``Admin.opt_ids`` as a compressed ``CV_32SC1`` matrix and
    ``Admin.opt_poses`` as a compressed ``CV_32FC1`` matrix containing one
    row-major 3x4 transform per ID.  This function only decodes that saved
    result; it never runs graph optimization and opens SQLite in read-only
    mode so importing cannot mutate the database.

    ``load_db()`` intentionally remains the legacy raw ``Node.pose`` path.
    Callers must opt in to this API when authoring map-frame topology.
    """
    db_path = str(Path(db_path).resolve())
    uri = f'file:{quote(db_path)}?mode=ro'
    conn = sqlite3.connect(uri, uri=True)
    try:
        admin = conn.execute(
            'SELECT version, opt_ids, opt_poses, opt_map, '
            'opt_map_x_min, opt_map_y_min, opt_map_resolution FROM Admin'
        ).fetchone()
        if not admin:
            raise ValueError('RTAB database has no Admin row')
        version, ids_blob, poses_blob, map_blob, origin_x, origin_y, resolution = admin
        ids = _decode_compressed_cv_mat(ids_blob, expected_type=CV_32SC1)
        pose_values = _decode_compressed_cv_mat(poses_blob, expected_type=CV_32FC1)
        ids = ids.reshape(-1)
        pose_values = pose_values.reshape(-1)
        if pose_values.size != ids.size * 12:
            raise ValueError(
                f'optimized graph cardinality mismatch: {ids.size} IDs, '
                f'{pose_values.size} pose floats'
            )

        stamps = {
            int(node_id): float(stamp)
            for node_id, stamp in conn.execute('SELECT id, stamp FROM Node')
        }
        missing = [int(node_id) for node_id in ids if int(node_id) not in stamps]
        if missing:
            raise ValueError(f'optimized graph references missing Node IDs: {missing}')

        nodes = []
        total = int(ids.size)
        for index, node_id_value in enumerate(ids):
            node_id = int(node_id_value)
            mat = np.eye(4, dtype=np.float64)
            mat[:3, :4] = pose_values[index * 12:(index + 1) * 12].reshape(3, 4)
            roll, pitch, yaw = _rotation_matrix_to_rpy(mat[:3, :3])
            nodes.append({
                'id': node_id,
                'timestamp': stamps[node_id],
                'x': float(mat[0, 3]),
                'y': float(mat[1, 3]),
                'z': float(mat[2, 3]),
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'mat': mat,
            })
            if progress_callback:
                progress_callback(int((index + 1) / total * 75))

        links = [
            {'from_id': int(from_id), 'to_id': int(to_id), 'type': int(link_type)}
            for from_id, to_id, link_type in conn.execute(
                'SELECT from_id, to_id, type FROM Link'
            )
        ]
        map_meta = {
            'origin_x': float(origin_x) if origin_x is not None else min(n['x'] for n in nodes),
            'origin_y': float(origin_y) if origin_y is not None else min(n['y'] for n in nodes),
            # With no stored occupancy map, resolution is a view scale only.
            # Keep enough pixels/metre for waypoint and edge-ID readability;
            # world coordinates are converted back with the same scale and
            # are never modified in the optimized node records.
            'resolution': (
                float(resolution)
                if resolution
                else TRAJECTORY_DISPLAY_RESOLUTION_M_PER_PIXEL
            ),
            'resolution_semantic': (
                'RTAB_OPT_MAP_RESOLUTION'
                if resolution
                else 'TRAJECTORY_DISPLAY_SCALE_ONLY'
            ),
        }
        if progress_callback:
            progress_callback(100)
        return {
            'nodes': nodes,
            'links': links,
            'map_blob': map_blob,
            'map_meta': map_meta,
            'database_version': str(version),
            'import_mode': OPTIMIZED_IMPORT_MODE,
            'importer_version': OPTIMIZED_IMPORTER_VERSION,
            'frame_id': 'map',
            'coordinate_semantic': 'OPTIMIZED_RTAB_GRAPH',
        }
    finally:
        conn.close()


def _decode_compressed_cv_mat(blob: bytes, expected_type: int) -> np.ndarray:
    """Decode the ``compressData2()`` cv::Mat representation used by RTAB-Map."""
    if blob is None or len(blob) < 12:
        raise ValueError('optimized graph matrix is absent or truncated')
    rows, cols, cv_type = struct.unpack_from('<iii', blob, len(blob) - 12)
    if rows <= 0 or cols <= 0:
        raise ValueError(f'invalid optimized graph matrix dimensions: {rows}x{cols}')
    if cv_type != expected_type:
        raise ValueError(
            f'unexpected optimized graph matrix type: {cv_type}, expected {expected_type}'
        )
    dtype = np.int32 if cv_type == CV_32SC1 else np.float32
    try:
        raw = zlib.decompress(blob[:-12])
    except zlib.error as exc:
        raise ValueError(f'invalid optimized graph compression: {exc}') from exc
    expected_bytes = rows * cols * np.dtype(dtype).itemsize
    if len(raw) != expected_bytes:
        raise ValueError(
            f'optimized graph matrix payload is {len(raw)} bytes, expected {expected_bytes}'
        )
    return np.frombuffer(raw, dtype=dtype).reshape(rows, cols)


def _rotation_matrix_to_rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    """Return ROS fixed-axis roll, pitch, yaw from a 3x3 rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2, 0]))))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
        yaw = 0.0
    return roll, pitch, yaw


def _parse_pose(blob: bytes) -> np.ndarray:
    """RTAB-Map 存储位姿为行优先 float32 × 12 (3×4)，补齐为 4×4 矩阵"""
    arr = np.frombuffer(blob, dtype=np.float32)
    mat = np.eye(4, dtype=np.float32)
    mat[:3, :4] = arr[:12].reshape(3, 4)
    return mat
