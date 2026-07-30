#!/usr/bin/python3.10
"""Generate offline Nav2 assets directly from an RTAB-Map database.

Outputs, in one shared map frame:
  - path_waypoints.yaml
  - clean_map.yaml / clean_map.pgm
  - obstacle_map.yaml / obstacle_map.pgm
  - obstacles.yaml

The source database has several large pose jumps near the end. For navigation
we use the first continuous trajectory segment from the map origin. The map is
rebuilt from RTAB-Map scan data so the environment remains visible. Test
obstacles are placed directly on that trajectory.
"""
import math
import os
import sqlite3
import struct
import sys
import zlib

import numpy as np
import yaml


FREE = 254
UNKNOWN = 205
OCCUPIED = 0

CV_8U = 0
CV_8S = 1
CV_16U = 2
CV_16S = 3
CV_32S = 4
CV_32F = 5
CV_64F = 6

CV_TYPE_TO_DTYPE = {
    CV_8U: np.uint8,
    CV_8S: np.int8,
    CV_16U: np.uint16,
    CV_16S: np.int16,
    CV_32S: np.int32,
    CV_32F: np.float32,
    CV_64F: np.float64,
}


def yaw_to_pi(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def read_db_poses(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
    poses = []
    for node_id, blob in cur.fetchall():
        if not blob or len(blob) < 48:
            continue
        vals = struct.unpack(f'<{len(blob) // 4}f', blob)
        transform = np.array(vals[:12], dtype=np.float64).reshape(3, 4)
        x = float(transform[0, 3])
        y = float(transform[1, 3])
        yaw = yaw_to_pi(math.atan2(transform[1, 0], transform[0, 0]))
        poses.append({'node_id': int(node_id), 'x': x, 'y': y, 'yaw': yaw})
    conn.close()
    return poses


def read_db_pose_transforms(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
    poses = {}
    for node_id, blob in cur.fetchall():
        if blob and len(blob) >= 48:
            vals = struct.unpack(f'<{len(blob) // 4}f', blob)
            transform = np.eye(4, dtype=np.float64)
            transform[:3, :] = np.array(vals[:12], dtype=np.float64).reshape(3, 4)
            poses[int(node_id)] = transform
    conn.close()
    return poses


def decompress_cv_mat(blob):
    if blob is None or len(blob) < 12:
        return None

    rows = struct.unpack_from('<i', blob, len(blob) - 12)[0]
    cols = struct.unpack_from('<i', blob, len(blob) - 8)[0]
    cv_type = struct.unpack_from('<i', blob, len(blob) - 4)[0]

    depth = cv_type & 7
    channels = (cv_type >> 3) + 1
    dtype = CV_TYPE_TO_DTYPE.get(depth)
    if dtype is None or rows <= 0 or cols <= 0 or channels <= 0:
        return None

    try:
        raw = zlib.decompress(blob[:len(blob) - 12])
    except zlib.error:
        return None

    expected = rows * cols * channels * np.dtype(dtype).itemsize
    if len(raw) != expected:
        return None

    return np.frombuffer(raw, dtype=dtype).reshape(rows * cols, channels)


def parse_scan_info(blob):
    if blob is None or len(blob) < 76:
        return None
    vals = struct.unpack('<19f', blob[:76])
    return np.array(vals[7:19], dtype=np.float64).reshape(3, 4)


def build_environment_grid_from_db(db_path, resolution, z_min, z_max):
    """Project RTAB-Map scan hits into a 2D occupancy image."""
    pose_transforms = read_db_pose_transforms(db_path)
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, scan_info, scan FROM Data WHERE scan IS NOT NULL ORDER BY id")

    xy_chunks = []
    scanned = 0
    used = 0
    raw_points = 0
    kept_points = 0
    for node_id, scan_info_blob, scan_blob in cur.fetchall():
        scanned += 1
        points = decompress_cv_mat(scan_blob)
        local_transform = parse_scan_info(scan_info_blob)
        pose = pose_transforms.get(int(node_id))
        if points is None or local_transform is None or pose is None or points.shape[1] < 3:
            continue

        xyz = points[:, :3].astype(np.float64, copy=False)
        local_xyz = (local_transform[:3, :3] @ xyz.T).T + local_transform[:3, 3]
        world_xyz = (pose[:3, :3] @ local_xyz.T).T + pose[:3, 3]
        raw_points += world_xyz.shape[0]
        z_mask = (world_xyz[:, 2] >= z_min) & (world_xyz[:, 2] <= z_max)
        world_xyz = world_xyz[z_mask]
        kept_points += world_xyz.shape[0]
        if world_xyz.size == 0:
            continue
        xy_chunks.append(world_xyz[:, :2].astype(np.float32))
        used += 1

    conn.close()
    if not xy_chunks:
        return None, None

    xy = np.vstack(xy_chunks)
    # The database contains a few far X outliers that make the image much wider
    # than the RTAB-Map export. Clip only the extreme tails so the visible
    # environment keeps the same scale and bounds as the original map.
    x_low, x_high = np.percentile(xy[:, 0], [0.01, 99.997])
    clip_mask = (xy[:, 0] >= x_low) & (xy[:, 0] <= x_high)
    xy = xy[clip_mask]
    x_min = float(np.floor(xy[:, 0].min() / resolution) * resolution)
    y_min = float(np.floor(xy[:, 1].min() / resolution) * resolution)
    x_max = float(np.ceil(xy[:, 0].max() / resolution) * resolution)
    y_max = float(np.ceil(xy[:, 1].max() / resolution) * resolution)

    cols = max(1, int(math.ceil((x_max - x_min) / resolution)))
    rows = max(1, int(math.ceil((y_max - y_min) / resolution)))
    hit_map = np.zeros((rows, cols), dtype=np.uint16)

    col_idx = np.clip(((xy[:, 0] - x_min) / resolution).astype(np.int32), 0, cols - 1)
    row_idx = np.clip(((y_max - xy[:, 1]) / resolution).astype(np.int32), 0, rows - 1)
    np.add.at(hit_map, (row_idx, col_idx), 1)

    grid = np.full((rows, cols), UNKNOWN, dtype=np.uint8)
    grid[hit_map > 0] = OCCUPIED
    meta = {
        'origin_x': x_min,
        'origin_y': y_min,
        'scanned_nodes': scanned,
        'used_nodes': used,
        'raw_points': raw_points,
        'kept_points': kept_points,
        'z_min': z_min,
        'z_max': z_max,
    }
    return grid, meta


def first_continuous_segment(poses, max_step):
    if len(poses) < 2:
        return poses
    for i in range(1, len(poses)):
        dx = poses[i]['x'] - poses[i - 1]['x']
        dy = poses[i]['y'] - poses[i - 1]['y']
        if math.hypot(dx, dy) > max_step:
            return poses[:i]
    return poses


def downsample_path(poses, spacing):
    if len(poses) < 2:
        return poses
    result = [poses[0]]
    for pose in poses[1:]:
        last = result[-1]
        if math.hypot(pose['x'] - last['x'], pose['y'] - last['y']) >= spacing:
            result.append(pose)
    if result[-1]['node_id'] != poses[-1]['node_id']:
        result.append(poses[-1])
    return result


def path_length(path):
    return sum(
        math.hypot(path[i]['x'] - path[i - 1]['x'], path[i]['y'] - path[i - 1]['y'])
        for i in range(1, len(path)))


def pose_at_distance(path, distance):
    if not path:
        raise ValueError('empty path')
    if distance <= 0:
        return path[0]

    covered = 0.0
    for i in range(1, len(path)):
        a = path[i - 1]
        b = path[i]
        seg = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        if seg <= 1e-6:
            continue
        if covered + seg >= distance:
            t = (distance - covered) / seg
            return {
                'x': a['x'] + (b['x'] - a['x']) * t,
                'y': a['y'] + (b['y'] - a['y']) * t,
                'yaw': math.atan2(b['y'] - a['y'], b['x'] - a['x']),
            }
        covered += seg
    return path[-1]


def make_obstacles(path):
    total = path_length(path)
    fractions = [0.18, 0.34, 0.50, 0.66, 0.82]
    obstacles = []
    for idx, fraction in enumerate(fractions):
        pose = pose_at_distance(path, total * fraction)
        obstacles.append({
            'cx': round(float(pose['x']), 3),
            'cy': round(float(pose['y']), 3),
            'width': 2.2 if idx % 2 == 0 else 2.6,
            'height': 2.2 if idx % 2 == 0 else 2.6,
        })
    return obstacles


def world_to_pixel(x, y, origin_x, y_top, resolution):
    col = int((x - origin_x) / resolution)
    row = int((y_top - y) / resolution)
    return row, col


def fill_disk(grid, row, col, radius_cells, value):
    rows, cols = grid.shape
    r0 = max(0, row - radius_cells)
    r1 = min(rows - 1, row + radius_cells)
    c0 = max(0, col - radius_cells)
    c1 = min(cols - 1, col + radius_cells)
    if r0 > r1 or c0 > c1:
        return
    yy, xx = np.ogrid[r0:r1 + 1, c0:c1 + 1]
    mask = (yy - row) * (yy - row) + (xx - col) * (xx - col) <= radius_cells * radius_cells
    grid[r0:r1 + 1, c0:c1 + 1][mask] = value


def carve_corridor(grid, path, origin_x, origin_y, resolution, corridor_radius):
    if corridor_radius <= 0.0:
        return
    y_top = origin_y + grid.shape[0] * resolution
    radius_cells = max(1, int(math.ceil(corridor_radius / resolution)))
    last = None
    for pose in path:
        row, col = world_to_pixel(pose['x'], pose['y'], origin_x, y_top, resolution)
        if last is None:
            fill_disk(grid, row, col, radius_cells, FREE)
        else:
            last_row, last_col = last
            dr = row - last_row
            dc = col - last_col
            steps = max(abs(dr), abs(dc), 1)
            for step in range(steps + 1):
                t = step / steps
                rr = int(round(last_row + dr * t))
                cc = int(round(last_col + dc * t))
                fill_disk(grid, rr, cc, radius_cells, FREE)
        last = (row, col)


def draw_obstacle(grid, obstacle, origin_x, origin_y, resolution):
    y_top = origin_y + grid.shape[0] * resolution
    rows, cols = grid.shape
    cx = float(obstacle['cx'])
    cy = float(obstacle['cy'])
    width = float(obstacle['width'])
    height = float(obstacle['height'])
    x_min = cx - width / 2.0
    x_max = cx + width / 2.0
    y_min = cy - height / 2.0
    y_max = cy + height / 2.0
    c_min = max(0, int((x_min - origin_x) / resolution))
    c_max = min(cols - 1, int((x_max - origin_x) / resolution))
    r_min = max(0, int((y_top - y_max) / resolution))
    r_max = min(rows - 1, int((y_top - y_min) / resolution))
    if c_min <= c_max and r_min <= r_max:
        grid[r_min:r_max + 1, c_min:c_max + 1] = OCCUPIED


def write_pgm(path, grid):
    h, w = grid.shape
    if os.path.islink(path):
        os.unlink(path)
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(grid.astype(np.uint8).tobytes())


def write_map_yaml(path, image, origin_x, origin_y, resolution):
    content = (
        f'image: {image}\n'
        'mode: trinary\n'
        f'resolution: {resolution:.4f}\n'
        f'origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0000]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    )
    if os.path.islink(path):
        os.unlink(path)
    with open(path, 'w') as f:
        f.write(content)


def write_yaml(path, data):
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def main():
    if len(sys.argv) < 3:
        print('Usage: generate_offline_nav_assets.py <database.db> <output_dir> '
              '[resolution=0.05] [spacing=1.0] [corridor_radius=5.0] '
              '[z_min=-0.3] [z_max=1.5]')
        sys.exit(1)

    db_path = sys.argv[1]
    output_dir = sys.argv[2]
    resolution = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    spacing = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    corridor_radius = float(sys.argv[5]) if len(sys.argv) > 5 else 5.0
    z_min = float(sys.argv[6]) if len(sys.argv) > 6 else -0.3
    z_max = float(sys.argv[7]) if len(sys.argv) > 7 else 1.5
    if z_min > z_max:
        raise ValueError(f'z_min must be <= z_max, got {z_min} > {z_max}')

    poses = read_db_poses(db_path)
    if not poses:
        raise RuntimeError(f'No poses found in {db_path}')

    continuous = first_continuous_segment(poses, max_step=8.0)
    waypoints = downsample_path(continuous, spacing)
    obstacles = make_obstacles(waypoints)

    clean, map_meta = build_environment_grid_from_db(db_path, resolution, z_min, z_max)
    if clean is None:
        xs = [p['x'] for p in continuous]
        ys = [p['y'] for p in continuous]
        margin = corridor_radius + 6.0
        origin_x = math.floor((min(xs) - margin) / resolution) * resolution
        origin_y = math.floor((min(ys) - margin) / resolution) * resolution
        x_max = math.ceil((max(xs) + margin) / resolution) * resolution
        y_max = math.ceil((max(ys) + margin) / resolution) * resolution
        cols = max(1, int(math.ceil((x_max - origin_x) / resolution)))
        rows = max(1, int(math.ceil((y_max - origin_y) / resolution)))
        clean = np.full((rows, cols), UNKNOWN, dtype=np.uint8)
        map_meta = {
            'origin_x': origin_x,
            'origin_y': origin_y,
            'scanned_nodes': 0,
            'used_nodes': 0,
            'raw_points': 0,
            'kept_points': 0,
            'z_min': z_min,
            'z_max': z_max,
        }

    origin_x = float(map_meta['origin_x'])
    origin_y = float(map_meta['origin_y'])
    carve_corridor(clean, continuous, origin_x, origin_y, resolution, corridor_radius)

    obstacle_map = clean.copy()
    for obstacle in obstacles:
        draw_obstacle(obstacle_map, obstacle, origin_x, origin_y, resolution)

    os.makedirs(output_dir, exist_ok=True)
    write_yaml(os.path.join(output_dir, 'path_waypoints.yaml'), {
        'source_database': db_path,
        'frame_id': 'map',
        'waypoints': [{'x': p['x'], 'y': p['y'], 'yaw': p['yaw']} for p in waypoints],
    })
    write_yaml(os.path.join(output_dir, 'obstacles.yaml'), {'obstacles': obstacles})
    write_pgm(os.path.join(output_dir, 'clean_map.pgm'), clean)
    write_map_yaml(os.path.join(output_dir, 'clean_map.yaml'), 'clean_map.pgm',
                   origin_x, origin_y, resolution)
    write_pgm(os.path.join(output_dir, 'obstacle_map.pgm'), obstacle_map)
    write_map_yaml(os.path.join(output_dir, 'obstacle_map.yaml'), 'obstacle_map.pgm',
                   origin_x, origin_y, resolution)

    print(f'[OK] Database: {db_path}')
    print(f'[OK] Poses: {len(poses)} raw, {len(continuous)} continuous, {len(waypoints)} waypoints')
    rows, cols = clean.shape
    print(f'[OK] Scans: {map_meta["used_nodes"]}/{map_meta["scanned_nodes"]} nodes used for environment map')
    print(f'[OK] Z filter: {map_meta["z_min"]:.2f}m <= z <= {map_meta["z_max"]:.2f}m, '
          f'points kept: {map_meta["kept_points"]}/{map_meta["raw_points"]}')
    print(f'[OK] Map: {cols}x{rows} @ {resolution:.3f}m, origin=({origin_x:.2f},{origin_y:.2f})')
    print(f'[OK] Wrote assets to {output_dir}')


if __name__ == '__main__':
    main()
