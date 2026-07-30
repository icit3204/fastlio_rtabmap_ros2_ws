#!/usr/bin/env python3
"""Extract 3D point cloud and 2D occupancy grid from RTAB-Map database.

Reads compressed laser scan data from the SQLite database, decompresses (zlib),
assembles with optimized poses, and writes PCD/PLY (3D) + PGM/YAML (2D map).
"""
import sqlite3
import struct
import zlib
import sys
import os
import numpy as np
from pathlib import Path
from datetime import datetime

# OpenCV Mat type constants
CV_8U  = 0
CV_8S  = 1
CV_16U = 2
CV_16S = 3
CV_32S = 4
CV_32F = 5
CV_64F = 6

CV_TYPE_TO_DTYPE = {
    CV_8U:  np.uint8,
    CV_8S:  np.int8,
    CV_16U: np.uint16,
    CV_16S: np.int16,
    CV_32S: np.int32,
    CV_32F: np.float32,
    CV_64F: np.float64,
}

SCAN_FORMAT_NAMES = {
    0: "kUnknown",     1: "kXY",        2: "kXYZ",
    3: "kXYI",         4: "kXYZI",      5: "kXYZRGB",
    6: "kXYNormal",    7: "kXYZNormal", 8: "kXYINormal",
    9: "kXYZINormal",  10: "kXYZRGBNormal",
}

SCAN_FORMAT_CHANNELS = {
    0: 0, 1: 2, 2: 3, 3: 3, 4: 4, 5: 4, 6: 5, 7: 6, 8: 6, 9: 7, 10: 7,
}


def decompress_cv_mat(blob):
    """Decompress a RTAB-Map compressed cv::Mat blob."""
    if blob is None or len(blob) < 12:
        return None, 0, 0

    rows = struct.unpack_from('<i', blob, len(blob) - 12)[0]
    cols = struct.unpack_from('<i', blob, len(blob) - 8)[0]
    cv_type = struct.unpack_from('<i', blob, len(blob) - 4)[0]

    depth = cv_type & 7
    channels = (cv_type >> 3) + 1
    dtype = CV_TYPE_TO_DTYPE.get(depth, np.float32)

    compressed = blob[: len(blob) - 12]
    try:
        raw = zlib.decompress(compressed)
    except zlib.error as e:
        print(f"  [WARN] zlib decompress failed: {e}")
        return None, 0, 0

    n_channels = channels
    n_points = cols

    expected = rows * cols * channels * np.dtype(dtype).itemsize
    if len(raw) != expected:
        print(f"  [WARN] size mismatch: got {len(raw)}, expected {expected}")
        return None, 0, 0

    arr = np.frombuffer(raw, dtype=dtype).reshape(rows * cols, channels)
    return arr, n_points, n_channels


def parse_scan_info(blob):
    """Parse scan_info blob (version >= 0.18.0): 7 floats header + 12 floats localTransform."""
    if blob is None or len(blob) < 76:
        return None
    vals = struct.unpack('<19f', blob)
    return {
        'format': int(vals[0]),
        'min_range': vals[1],
        'max_range': vals[2],
        'angle_min': vals[3],
        'angle_max': vals[4],
        'angle_inc': vals[5],
        'max_pts': int(vals[6]),
        'local_transform': np.array(vals[7:19]).reshape(3, 4),
    }


def get_optimized_poses(db_path):
    """Get node poses from the database."""
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
    poses = {}
    for row in cur.fetchall():
        node_id = row[0]
        pose_blob = row[1]
        if pose_blob and len(pose_blob) >= 48:
            vals = struct.unpack(f'<{len(pose_blob)//4}f', pose_blob)
            T = np.eye(4)
            T[:3, :] = np.array(vals[:12]).reshape(3, 4)
            poses[node_id] = T
    conn.close()
    return poses


def pack_rgb(r, g, b):
    """Pack R,G,B bytes into a single float32 (PCL XYZRGB convention)."""
    import struct
    return struct.unpack('f', struct.pack('BBBB', r, g, b, 255))[0]

def write_pcd_binary(filename, points, fields=None):
    """Write points to PCD format (binary)."""
    n = points.shape[0]
    dim = points.shape[1]
    if fields is None:
        fields = ['x', 'y', 'z', 'intensity', 'normal_x', 'normal_y', 'normal_z'][:dim]

    sizes = ' '.join(['4'] * dim)
    types = ' '.join(['F'] * dim)
    counts = ' '.join(['1'] * dim)

    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )
    with open(filename, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype(np.float32).tobytes())
    print(f"[OK] Wrote {n} points to {filename}")

# <修改 version1 增加轨迹导出到XYZRGB PCD>
def build_trajectory_points(poses, step=0.05):
    """Build trajectory points with RGB color from ordered poses.

    Interpolates between consecutive poses at `step` spacing (meters).
    Returns Nx6 float32 array: x, y, z, r, g, b (rgb packed as float32).
    """
    if len(poses) < 2:
        return np.empty((0, 6), dtype=np.float32)

    # Sort poses by node ID (assumes sequential traversal)
    sorted_ids = sorted(poses.keys())
    traj_pts = []
    green_rgb = pack_rgb(0, 255, 0)

    for i in range(len(sorted_ids) - 1):
        id_a = sorted_ids[i]
        id_b = sorted_ids[i + 1]
        T_a = poses[id_a]
        T_b = poses[id_b]
        p_a = T_a[:3, 3]
        p_b = T_b[:3, 3]
        dist = np.linalg.norm(p_b - p_a)
        if dist < 0.01:
            continue
        n_steps = max(2, int(dist / step))
        for s in range(n_steps):
            t = s / float(n_steps - 1)
            pt = p_a + t * (p_b - p_a)
            traj_pts.append([pt[0], pt[1], pt[2], green_rgb])

    if not traj_pts:
        return np.empty((0, 6), dtype=np.float32)
    return np.array(traj_pts, dtype=np.float32)

# <修改 version2 增加2D PGM轨迹绘制>
def get_trajectory_xy(poses):
    """Extract ordered XY trajectory positions from poses."""
    if len(poses) < 2:
        return np.empty((0, 2), dtype=np.float32)
    sorted_ids = sorted(poses.keys())
    traj = []
    for nid in sorted_ids:
        t = poses[nid][:3, 3]
        traj.append([t[0], t[1]])
    return np.array(traj, dtype=np.float32)


def draw_trajectory_on_pgm(pgm, traj_xy, origin_x, origin_y, resolution, y_max, value=128):
    """Draw trajectory lines onto a PGM grid using Bresenham algorithm.

    Args:
        pgm: 2D numpy uint8 occupancy grid (row 0 = y_max, flipped for ROS)
        traj_xy: Nx2 world-frame trajectory positions
        origin_x, origin_y: world coordinates of bottom-left cell center
        resolution: m/cell
        y_max: max world y (top of grid)
        value: grayscale value for trajectory (0-255, default 128=gray)
    """
    if traj_xy.shape[0] < 2:
        return pgm

    rows, cols = pgm.shape

    def world_to_pixel(x, y):
        """Convert world coordinates to PGM pixel indices (row, col)."""
        c = int((x - origin_x) / resolution)
        r = int((y_max - y) / resolution)
        return r, c

    def in_bounds(r, c):
        return 0 <= r < rows and 0 <= c < cols

    for i in range(traj_xy.shape[0] - 1):
        x0, y0 = traj_xy[i, 0], traj_xy[i, 1]
        x1, y1 = traj_xy[i + 1, 0], traj_xy[i + 1, 1]

        r0, c0 = world_to_pixel(x0, y0)
        r1, c1 = world_to_pixel(x1, y1)

        # Bresenham line algorithm
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dr - dc

        r, c = r0, c0
        while True:
            if in_bounds(r, c):
                pgm[r, c] = value
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

    print(f"[INFO] Trajectory drawn on PGM ({traj_xy.shape[0]} poses, value={value})")
    return pgm

def write_pcd_xyzrgb(filename, cloud_xyzrgb, traj_xyzrgb):
    """Write combined cloud and trajectory as XYZRGB PCD."""
    n_cloud = cloud_xyzrgb.shape[0]
    n_traj = traj_xyzrgb.shape[0]
    total = n_cloud + n_traj

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {total}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {total}\n"
        "DATA binary\n"
    )
    with open(filename, 'wb') as f:
        f.write(header.encode('ascii'))
        if n_cloud > 0:
            f.write(cloud_xyzrgb.astype(np.float32).tobytes())
        if n_traj > 0:
            f.write(traj_xyzrgb.astype(np.float32).tobytes())
    print(f"[OK] Wrote {total} points (cloud={n_cloud}, traj={n_traj}) to {filename}")


def write_ply_ascii(filename, points):
    """Write Nx3+ points to PLY ascii format."""
    n = points.shape[0]
    dim = points.shape[1]
    has_intensity = dim >= 4
    has_normals = dim >= 7

    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        if has_intensity:
            f.write("property float intensity\n")
        if has_normals:
            f.write("property float nx\n")
            f.write("property float ny\n")
            f.write("property float nz\n")
        f.write("end_header\n")
        for i in range(n):
            vals = ' '.join(f'{v:.6f}' for v in points[i, :min(dim, 7)])
            f.write(vals + '\n')
    print(f"[OK] Wrote {n} points to {filename}")


def build_occupancy_grid(xy_world, resolution=0.05, z_min=-1.0, z_max=1.0,
                          occupied_thresh=1, free_thresh=-1):
    """Build a 2D occupancy grid from world-frame XY points.

    Points with z outside [z_min, z_max] are treated as ceiling/floor and removed.

    Returns:
        pgm_data: 2D numpy uint8 array (0=occupied, 255=free)
        origin_x, origin_y: world coordinates of the bottom-left cell center
        resolution: cell size in meters
    """
    if xy_world.shape[0] == 0:
        return None, 0, 0, resolution

    x = xy_world[:, 0]
    y = xy_world[:, 1]

    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    width_m = x_max - x_min
    height_m = y_max - y_min

    cols = max(1, int(np.ceil(width_m / resolution)))
    rows = max(1, int(np.ceil(height_m / resolution)))

    # Count hits per cell. Row 0 = top (y_max), image flipped for ROS convention
    hit_map = np.zeros((rows, cols), dtype=np.int32)
    col_idx = np.clip(((x - x_min) / resolution).astype(np.int32), 0, cols - 1)
    row_idx = np.clip(((y_max - y) / resolution).astype(np.int32), 0, rows - 1)

    np.add.at(hit_map, (row_idx, col_idx), 1)

    # 0 = occupied (black), 255 = free (white), 205 = unknown
    pgm = np.full((rows, cols), 205, dtype=np.uint8)
    pgm[hit_map >= occupied_thresh] = 0
    pgm[hit_map <= free_thresh] = 255

    origin_x = x_min
    origin_y = y_min

    return pgm, origin_x, origin_y, resolution


def write_pgm(filename, pgm_data):
    """Write PGM P5 binary image."""
    h, w = pgm_data.shape
    header = f"P5\n{w} {h}\n255\n"
    with open(filename, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(pgm_data.tobytes())
    print(f"[OK] Wrote {w}x{h} map to {filename}")


def write_map_yaml(yaml_path, pgm_name, origin_x, origin_y, resolution,
                   occupied_thresh=0.65, free_thresh=0.196):
    """Write nav2_map_server compatible YAML metadata."""
    yaml_content = (
        f"image: {pgm_name}\n"
        f"mode: trinary\n"
        f"resolution: {resolution:.4f}\n"
        f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0000]\n"
        f"negate: 0\n"
        f"occupied_thresh: {occupied_thresh}\n"
        f"free_thresh: {free_thresh}\n"
    )
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"[OK] Wrote YAML to {yaml_path}")


def main():
    db_path = sys.argv[1] if len(sys.argv) > 1 else "/data/maps/site_a/rtabmap.db"
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cloud_map")

    # 2D map parameters
    map_resolution = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    z_min = float(sys.argv[4]) if len(sys.argv) > 4 else -100.0
    z_max = float(sys.argv[5]) if len(sys.argv) > 5 else 100.0

    os.makedirs(output_dir, exist_ok=True)

    if not os.path.isfile(db_path):
        print(f"[ERROR] Database not found: {db_path}")
        sys.exit(1)

    print(f"[INFO] Database: {db_path}")
    print(f"[INFO] Output dir: {output_dir}")
    print(f"[INFO] 2D map resolution: {map_resolution}m")
    if z_min > -100 or z_max < 100:
        print(f"[INFO] Z filter: [{z_min}, {z_max}]m")

    # Get poses
    poses = get_optimized_poses(db_path)
    print(f"[INFO] Loaded {len(poses)} poses")

    # Read scan data
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, scan_info, scan FROM Data WHERE scan IS NOT NULL ORDER BY id")

    all_points = []
    all_xy_world = []  # for 2D grid
    total_nodes = 0
    scan_format_name = "unknown"

    for row in cur.fetchall():
        node_id, scan_info_blob, scan_blob = row
        total_nodes += 1

        info = parse_scan_info(scan_info_blob)
        if info is None:
            print(f"  [WARN] Node {node_id}: no scan info")
            continue

        fmt = info['format']
        n_channels = SCAN_FORMAT_CHANNELS.get(fmt, 0)
        scan_format_name = SCAN_FORMAT_NAMES.get(fmt, f"unknown({fmt})")

        if total_nodes == 1:
            print(f"[INFO] Scan format: {scan_format_name} ({n_channels} channels)")

        points, n_points, actual_channels = decompress_cv_mat(scan_blob)
        if points is None or n_points == 0:
            print(f"  [WARN] Node {node_id}: decompression failed")
            continue
        if points.shape[1] < 3:
            continue

        xyz = points[:, :3]

        # Apply local transform from scan_info
        lt = info['local_transform']
        R_local = lt[:3, :3]
        t_local = lt[:3, 3]
        xyz_local = (R_local @ xyz.T).T + t_local

        # Apply node pose
        if node_id in poses:
            T = poses[node_id]
            R = T[:3, :3]
            t = T[:3, 3]
            xyz_world = (R @ xyz_local.T).T + t
        else:
            print(f"  [WARN] Node {node_id}: no pose, using local frame")
            xyz_world = xyz_local

        # full point cloud with channels
        out = np.copy(points)
        out[:, :3] = xyz_world
        all_points.append(out)

        # 2D projection points (filtered by z range)
        mask = (xyz_world[:, 2] >= z_min) & (xyz_world[:, 2] <= z_max)
        if mask.any():
            all_xy_world.append(xyz_world[mask])

    conn.close()

    if not all_points:
        print("[ERROR] No point cloud data found in database!")
        sys.exit(1)

    # --- 3D point cloud ---
    cloud = np.vstack(all_points)
    print(f"[INFO] Total assembled 3D points: {cloud.shape[0]} (channels: {cloud.shape[1]})")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    basename = f"rtabmap_{ts}"

    # <修改 version1 输出XYZRGB PCD含轨迹>
    # Convert scan points to XYZRGB (intensity -> grayscale)
    scan_xyz = cloud[:, :3].astype(np.float32)
    if cloud.shape[1] >= 4:
        intensity = cloud[:, 3]
        i_norm = np.clip(intensity / 255.0, 0.0, 1.0)
        r = (i_norm * 255).astype(np.uint8)
        g = (i_norm * 255).astype(np.uint8)
        b = (i_norm * 255).astype(np.uint8)
    else:
        r = np.full(cloud.shape[0], 200, dtype=np.uint8)
        g = np.full(cloud.shape[0], 200, dtype=np.uint8)
        b = np.full(cloud.shape[0], 200, dtype=np.uint8)
    rgb_packed = np.array([pack_rgb(int(r[i]), int(g[i]), int(b[i])) for i in range(cloud.shape[0])], dtype=np.float32)
    cloud_xyzrgb = np.column_stack([scan_xyz, rgb_packed])

    # Build trajectory points
    traj_xyzrgb = build_trajectory_points(poses, step=0.05)
    print(f"[INFO] Trajectory points: {traj_xyzrgb.shape[0]} (from {len(poses)} poses)")

    pcd_path = os.path.join(output_dir, f"{basename}_cloud.pcd")
    write_pcd_xyzrgb(pcd_path, cloud_xyzrgb, traj_xyzrgb)

    ply_path = os.path.join(output_dir, f"{basename}_cloud.ply")
    write_ply_ascii(ply_path, cloud[:, :min(cloud.shape[1], 7)])

    # --- 2D occupancy grid ---
    if all_xy_world:
        xy = np.vstack(all_xy_world)
        y_max = float(xy[:, 1].max())  # <修改 version2 用于轨迹绘制>
        pgm, origin_x, origin_y, resolution = build_occupancy_grid(
            xy[:, :2], resolution=map_resolution)

        if pgm is not None:
            # <修改 version2 在PGM上绘制轨迹>
            traj_xy = get_trajectory_xy(poses)
            if traj_xy.shape[0] >= 2:
                # <原版-trajectory value=128灰色>
                # <修改 version2 value=0黑色 在RViz中高对比显示>
                draw_trajectory_on_pgm(pgm, traj_xy, origin_x, origin_y,
                                       resolution, y_max, value=0)

            pgm_name = f"{basename}_map.pgm"
            pgm_path = os.path.join(output_dir, pgm_name)
            write_pgm(pgm_path, pgm)

            yaml_path = os.path.join(output_dir, f"{basename}_map.yaml")
            write_map_yaml(yaml_path, pgm_name, origin_x, origin_y,
                           resolution)

            h, w = pgm.shape
            print(f"[INFO] 2D Map: {w}x{h} cells @ {resolution:.3f}m/cell")
            print(f"[INFO] 2D Map bounds: x=[{origin_x:.2f}, {origin_x + w*resolution:.2f}], y=[{origin_y:.2f}, {origin_y + h*resolution:.2f}]")
        else:
            print("[WARN] 2D map generation returned empty grid")
    else:
        print("[WARN] No XY points for 2D map (check z filter or scan data)")

    print(f"\n[DONE] Exported to {output_dir}/")
    print(f"  3D PCD : {os.path.basename(pcd_path)}")
    print(f"  3D PLY : {os.path.basename(ply_path)}")
    if all_xy_world:
        print(f"  2D PGM : {basename}_map.pgm")
        print(f"  2D YAML: {basename}_map.yaml")


if __name__ == "__main__":
    main()
