#!/usr/bin/python3.10
"""Extract robot path from RTAB-Map database and save as YAML waypoints.

Usage:
  python3 extract_path_yaml.py <database.db> [output.yaml] [--step N]
"""
import sqlite3
import struct
import math
import sys
import os
import yaml
import numpy as np


def get_poses(db_path):
    """Get optimized node poses ordered by ID."""
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
    poses = []
    for row in cur.fetchall():
        node_id = row[0]
        pose_blob = row[1]
        if pose_blob and len(pose_blob) >= 48:
            vals = struct.unpack(f'<{len(pose_blob)//4}f', pose_blob)
            T = np.eye(4)
            T[:3, :] = np.array(vals[:12]).reshape(3, 4)
            x, y, z = T[0, 3], T[1, 3], T[2, 3]
            # Extract yaw from rotation matrix
            yaw = math.atan2(T[1, 0], T[0, 0])
            poses.append({'node_id': node_id, 'x': float(x), 'y': float(y),
                         'z': float(z), 'yaw': float(yaw)})
    conn.close()
    return poses


def downsample_poses(poses, spacing=1.0):
    """Downsample poses to roughly `spacing` meters apart."""
    if len(poses) < 2:
        return poses
    result = [poses[0]]
    for p in poses[1:]:
        last = result[-1]
        dx = p['x'] - last['x']
        dy = p['y'] - last['y']
        if dx * dx + dy * dy >= spacing * spacing:
            result.append(p)
    if result[-1]['node_id'] != poses[-1]['node_id']:
        result.append(poses[-1])
    return result


def main():
    db_path = sys.argv[1] if len(sys.argv) > 1 else '/data/maps/db/first_version_0514.db'
    output = sys.argv[2] if len(sys.argv) > 2 else None
    step = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

    if output is None:
        output = os.path.join(os.path.dirname(db_path),
                             f"path_waypoints_{os.path.basename(db_path).replace('.db', '')}.yaml")

    poses = get_poses(db_path)
    print(f"[INFO] Extracted {len(poses)} poses from {db_path}")

    filtered = downsample_poses(poses, spacing=step)
    print(f"[INFO] Downsampled to {len(filtered)} waypoints (spacing={step}m)")

    # Remove z and node_id for YAML cleanliness
    waypoints = [{'x': p['x'], 'y': p['y'], 'yaw': p['yaw']} for p in filtered]

    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
    with open(output, 'w') as f:
        yaml.dump({'waypoints': waypoints}, f, default_flow_style=False)
    print(f"[OK] Saved {len(waypoints)} waypoints to {output}")


if __name__ == '__main__':
    main()
