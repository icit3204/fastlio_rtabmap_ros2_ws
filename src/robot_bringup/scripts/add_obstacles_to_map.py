#!/usr/bin/python3.10
"""Add obstacle rectangles to a PGM map.

Reads a clean PGM+YAML, draws obstacle blocks at specified positions,
and writes the modified map files.

Usage:
  python3 add_obstacles_to_map.py <clean_map.yaml> [obstacles.yaml] [output_dir]
"""
import sys
import os
import yaml
import numpy as np
from pathlib import Path


def read_pgm(pgm_path):
    """Read PGM P5 or P2 file, return numpy array and maxval."""
    with open(pgm_path, 'rb') as f:
        # Read magic number
        magic = f.readline().strip()
        if magic not in (b'P5', b'P2'):
            raise ValueError(f'Unsupported PGM format: {magic}')

        # Skip comments
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
    """Write numpy array as PGM P5 binary."""
    h, w = data.shape
    header = f"P5\n{w} {h}\n{maxval}\n"
    with open(pgm_path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(data.astype(np.uint8).tobytes())
    print(f"[OK] Wrote {w}x{h} PGM to {pgm_path}")


def draw_rectangle(pgm, origin_x, origin_y, resolution, y_max,
                   cx, cy, width_m, height_m, fill_value=0):
    """Draw a filled rectangle obstacle on the PGM.

    Args:
        pgm: 2D numpy uint8 array (image coord: row 0 = y_max)
        origin_x, origin_y: bottom-left corner world coords
        resolution: m/cell
        y_max: max world y (top of image)
        cx, cy: rectangle center world coords
        width_m, height_m: rectangle size in meters
        fill_value: grayscale fill value (0=black/occupied)
    """
    rows, cols = pgm.shape

    # World to pixel
    x_min_w = cx - width_m / 2
    x_max_w = cx + width_m / 2
    y_min_w = cy - height_m / 2
    y_max_w = cy + height_m / 2

    c_min = int((x_min_w - origin_x) / resolution)
    c_max = int((x_max_w - origin_x) / resolution)
    r_max = int((y_max - y_min_w) / resolution)  # top row (smaller y in world)
    r_min = int((y_max - y_max_w) / resolution)  # bottom row

    # Clamp
    c_min = max(0, c_min)
    c_max = min(cols - 1, c_max)
    r_min = max(0, r_min)
    r_max = min(rows - 1, r_max)

    if c_min < c_max and r_min < r_max:
        pgm[r_min:r_max + 1, c_min:c_max + 1] = fill_value
        print(f"  Obstacle: world({cx:.1f},{cy:.1f}) size({width_m}x{height_m})m "
              f"→ pixels[{r_min}:{r_max},{c_min}:{c_max}]")
    else:
        print(f"  [WARN] Obstacle out of bounds: world({cx:.1f},{cy:.1f})")


def main():
    if len(sys.argv) < 2:
        print("Usage: add_obstacles_to_map.py <clean_map.yaml> [obstacles.yaml] [output_dir]")
        sys.exit(1)

    yaml_path = sys.argv[1]
    obstacles_yaml = sys.argv[2] if len(sys.argv) > 2 else None
    output_dir = sys.argv[3] if len(sys.argv) > 3 else os.path.dirname(yaml_path)

    # Read map metadata
    with open(yaml_path, 'r') as f:
        map_meta = yaml.safe_load(f)

    pgm_name = map_meta['image']
    resolution = float(map_meta['resolution'])
    origin_x = float(map_meta['origin'][0])
    origin_y = float(map_meta['origin'][1])
    occupied_thresh = float(map_meta.get('occupied_thresh', 0.65))
    free_thresh = float(map_meta.get('free_thresh', 0.196))

    pgm_dir = os.path.dirname(os.path.abspath(yaml_path))
    pgm_path = os.path.join(pgm_dir, pgm_name)

    # Read PGM
    data, maxval = read_pgm(pgm_path)
    rows, cols = data.shape
    y_max = origin_y + rows * resolution
    print(f"[INFO] Map: {cols}x{rows} cells @ {resolution}m/cell")
    print(f"[INFO] World bounds: x=[{origin_x:.1f},{origin_x + cols * resolution:.1f}], "
          f"y=[{origin_y:.1f},{y_max:.1f}]")

    # Load obstacle definitions
    if obstacles_yaml and os.path.exists(obstacles_yaml):
        with open(obstacles_yaml, 'r') as f:
            obstacle_list = yaml.safe_load(f)['obstacles']
        print(f"[INFO] Loaded {len(obstacle_list)} obstacles from {obstacles_yaml}")
    else:
        # Default obstacles: placed at various positions along the expected path
        # The path area is roughly from world coords
        obstacle_list = [
            # Format: {cx, cy, width, height}
            {'cx': 10.0, 'cy': -10.0, 'width': 2.0, 'height': 2.0},
            {'cx': 20.0, 'cy': -20.0, 'width': 1.5, 'height': 3.0},
            {'cx': 35.0, 'cy': -35.0, 'width': 2.5, 'height': 2.5},
            {'cx': 50.0, 'cy': -50.0, 'width': 2.0, 'height': 2.0},
            {'cx': 60.0, 'cy': -65.0, 'width': 3.0, 'height': 1.5},
            {'cx': 75.0, 'cy': -55.0, 'width': 2.0, 'height': 2.0},
        ]
        print(f"[INFO] Using {len(obstacle_list)} default obstacles")

    # Draw obstacles
    for obs in obstacle_list:
        draw_rectangle(data, origin_x, origin_y, resolution, y_max,
                      float(obs['cx']), float(obs['cy']),
                      float(obs['width']), float(obs['height']),
                      fill_value=0)

    # Write output
    os.makedirs(output_dir, exist_ok=True)

    out_pgm_name = 'obstacle_map.pgm'
    out_pgm_path = os.path.join(output_dir, out_pgm_name)
    write_pgm(out_pgm_path, data)

    out_yaml_path = os.path.join(output_dir, 'obstacle_map.yaml')
    yaml_content = (
        f"image: {out_pgm_name}\n"
        f"mode: trinary\n"
        f"resolution: {resolution:.4f}\n"
        f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0000]\n"
        f"negate: 0\n"
        f"occupied_thresh: {occupied_thresh}\n"
        f"free_thresh: {free_thresh}\n"
    )
    with open(out_yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"[OK] Wrote YAML to {out_yaml_path}")
    print(f"[DONE] Obstacle map created in {output_dir}/")


if __name__ == '__main__':
    main()
