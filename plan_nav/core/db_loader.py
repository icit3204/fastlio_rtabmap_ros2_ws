# [DONE] F-1.1 导入 .db 文件并解析 RTAB-Map 数据库
# [DONE] F-1.3 按时间戳顺序加载轨迹位姿

import sqlite3
import numpy as np
import math
import os


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
    }


def _parse_pose(blob: bytes) -> np.ndarray:
    """RTAB-Map 存储位姿为行优先 float32 × 12 (3×4)，补齐为 4×4 矩阵"""
    arr = np.frombuffer(blob, dtype=np.float32)
    mat = np.eye(4, dtype=np.float32)
    mat[:3, :4] = arr[:12].reshape(3, 4)
    return mat
