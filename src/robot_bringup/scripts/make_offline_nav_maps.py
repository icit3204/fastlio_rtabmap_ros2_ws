#!/usr/bin/python3.10
"""Build Nav2-friendly offline maps from a raw RTAB-Map PGM.

The exported RTAB-Map point projection used in this project marks scan hits
and the drawn trajectory as black pixels. Nav2 interprets black pixels as
occupied, so the recorded path becomes an obstacle. This script keeps the raw
map size/origin but opens a free corridor along the extracted path, then
optionally bakes rectangular test obstacles into that corridor.
"""
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml


FREE = 254
UNKNOWN = 205
OCCUPIED = 0


def read_pgm(pgm_path):
    with open(pgm_path, 'rb') as f:
        magic = f.readline().strip()
        if magic not in (b'P5', b'P2'):
            raise ValueError(f'Unsupported PGM format: {magic}')

        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()

        width, height = map(int, line.split())
        maxval = int(f.readline().strip())

        if magic == b'P5':
            data = np.frombuffer(f.read(), dtype=np.uint8 if maxval < 256 else np.uint16).copy()
        else:
            data = np.fromfile(f, dtype=np.uint8, sep=' ')

    return data.reshape(height, width), maxval


def write_pgm(pgm_path, data, maxval=255):
    h, w = data.shape
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n{maxval}\n'.encode('ascii'))
        f.write(data.astype(np.uint8).tobytes())
    print(f'[OK] Wrote {w}x{h} PGM to {pgm_path}')


def write_yaml(yaml_path, pgm_name, meta):
    content = (
        f'image: {pgm_name}\n'
        'mode: trinary\n'
        f"resolution: {float(meta['resolution']):.4f}\n"
        f"origin: [{float(meta['origin'][0]):.4f}, {float(meta['origin'][1]):.4f}, 0.0000]\n"
        'negate: 0\n'
        f"occupied_thresh: {float(meta.get('occupied_thresh', 0.65))}\n"
        f"free_thresh: {float(meta.get('free_thresh', 0.196))}\n"
    )
    with open(yaml_path, 'w') as f:
        f.write(content)
    print(f'[OK] Wrote YAML to {yaml_path}')


def load_waypoints(path_yaml):
    with open(path_yaml, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('waypoints', [])


def load_obstacles(obstacles_yaml):
    if not obstacles_yaml or not os.path.exists(obstacles_yaml):
        return []
    with open(obstacles_yaml, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('obstacles', [])


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


def carve_path_corridor(grid, waypoints, meta, corridor_radius_m):
    resolution = float(meta['resolution'])
    origin_x = float(meta['origin'][0])
    origin_y = float(meta['origin'][1])
    y_top = origin_y + grid.shape[0] * resolution
    radius_cells = max(1, int(math.ceil(corridor_radius_m / resolution)))

    last_row = None
    last_col = None
    max_segment_m = 8.0
    max_segment_cells = int(max_segment_m / resolution)

    for wp in waypoints:
        row, col = world_to_pixel(float(wp['x']), float(wp['y']), origin_x, y_top, resolution)
        if last_row is None:
            fill_disk(grid, row, col, radius_cells, FREE)
        else:
            dr = row - last_row
            dc = col - last_col
            steps = max(abs(dr), abs(dc))
            if steps <= max_segment_cells:
                for step in range(steps + 1):
                    t = step / max(1, steps)
                    rr = int(round(last_row + dr * t))
                    cc = int(round(last_col + dc * t))
                    fill_disk(grid, rr, cc, radius_cells, FREE)
            else:
                fill_disk(grid, row, col, radius_cells, FREE)
        last_row = row
        last_col = col

    print(f'[OK] Carved free corridor: {len(waypoints)} waypoints, radius={corridor_radius_m:.2f}m')


def draw_obstacle(grid, obs, meta):
    resolution = float(meta['resolution'])
    origin_x = float(meta['origin'][0])
    origin_y = float(meta['origin'][1])
    y_top = origin_y + grid.shape[0] * resolution
    rows, cols = grid.shape

    cx = float(obs['cx'])
    cy = float(obs['cy'])
    width = float(obs['width'])
    height = float(obs['height'])

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
        print(f'  Obstacle: world({cx:.1f},{cy:.1f}) size({width}x{height})m '
              f'pixels[{r_min}:{r_max},{c_min}:{c_max}]')
    else:
        print(f'  [WARN] Obstacle out of bounds: world({cx:.1f},{cy:.1f})')


def main():
    if len(sys.argv) < 4:
        print('Usage: make_offline_nav_maps.py <raw_map.yaml> <path_waypoints.yaml> <output_dir> '
              '[obstacles.yaml] [corridor_radius_m]')
        sys.exit(1)

    raw_yaml = sys.argv[1]
    path_yaml = sys.argv[2]
    output_dir = sys.argv[3]
    obstacles_yaml = sys.argv[4] if len(sys.argv) > 4 else None
    corridor_radius_m = float(sys.argv[5]) if len(sys.argv) > 5 else 5.0

    with open(raw_yaml, 'r') as f:
        meta = yaml.safe_load(f)

    raw_dir = Path(raw_yaml).resolve().parent
    raw_pgm = raw_dir / meta['image']
    if not raw_pgm.exists():
        fallback = raw_dir / (Path(raw_yaml).stem + '.pgm')
        if fallback.exists():
            raw_pgm = fallback
        else:
            raise FileNotFoundError(f'PGM not found: {raw_pgm}')
    raw, _ = read_pgm(raw_pgm)
    waypoints = load_waypoints(path_yaml)
    obstacles = load_obstacles(obstacles_yaml)

    if not waypoints:
        raise RuntimeError(f'No waypoints found in {path_yaml}')

    os.makedirs(output_dir, exist_ok=True)

    clean = np.full(raw.shape, UNKNOWN, dtype=np.uint8)
    clean[raw == OCCUPIED] = OCCUPIED
    carve_path_corridor(clean, waypoints, meta, corridor_radius_m)

    clean_pgm = 'clean_map_0518.pgm'
    clean_yaml = 'clean_map_0518.yaml'
    write_pgm(os.path.join(output_dir, clean_pgm), clean)
    write_yaml(os.path.join(output_dir, clean_yaml), clean_pgm, meta)

    obstacle_map = clean.copy()
    for obs in obstacles:
        draw_obstacle(obstacle_map, obs, meta)

    obstacle_pgm = 'obstacle_map.pgm'
    obstacle_yaml = 'obstacle_map.yaml'
    write_pgm(os.path.join(output_dir, obstacle_pgm), obstacle_map)
    write_yaml(os.path.join(output_dir, obstacle_yaml), obstacle_pgm, meta)

    print(f'[DONE] Generated clean and obstacle Nav2 maps in {output_dir}')


if __name__ == '__main__':
    main()
