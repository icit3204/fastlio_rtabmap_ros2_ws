# [DONE] F-1.2 从 Admin 表提取 2D 占用栅格，zlib解压并解码为三色底图

import numpy as np
import zlib

# 默认地图元数据（fallback 值）
DEFAULT_ORIGIN_X = -3.34
DEFAULT_ORIGIN_Y = -85.93
DEFAULT_RESOLUTION = 0.05


def decode_map(map_blob: bytes, map_meta: dict = None) -> dict:
    """
    解码 opt_map (zlib压缩的 CV_8SC1) 为可渲染的 RGB 底图。

    参数:
      map_blob: opt_map 原始 zlib 压缩 BLOB
      map_meta: {'origin_x', 'origin_y', 'resolution'} 从 Admin 表读取

    返回:
    {
      'image_rgb': np.ndarray (H×W×3, uint8),
      'origin_x': float,
      'origin_y': float,
      'resolution': float,
      'width': int,
      'height': int,
    }
    """
    # zlib 解压
    raw = np.frombuffer(zlib.decompress(map_blob), dtype=np.int8)
    total_pixels = len(raw)

    if map_meta:
        origin_x = map_meta.get('origin_x', DEFAULT_ORIGIN_X)
        origin_y = map_meta.get('origin_y', DEFAULT_ORIGIN_Y)
        resolution = map_meta.get('resolution', DEFAULT_RESOLUTION)
    else:
        origin_x = DEFAULT_ORIGIN_X
        origin_y = DEFAULT_ORIGIN_Y
        resolution = DEFAULT_RESOLUTION

    # 根据像素总数推断地图尺寸：优先使用较大值作为高度（Y轴跨度通常更大）
    h, w = _infer_dimensions(total_pixels)
    raw = raw.reshape(h, w)

    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[raw == -1] = [220, 218, 210]   # 未知区域: 浅灰 #dcdad2
    rgb[raw == 0]  = [244, 242, 237]   # 空地: 米白 #f4f2ed
    rgb[raw == 100]= [80,  82,  75]    # 障碍物: 深灰 #50524b

    return {
        'image_rgb': rgb,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'resolution': resolution,
        'width': w,
        'height': h,
    }


def _infer_dimensions(total: int) -> tuple:
    """根据像素总数推断地图 (高度, 宽度)，Y轴跨度通常更大故高度>宽度"""
    best_h, best_w = 1, total
    min_diff = float('inf')
    target_ratio = 2.5  # Y/X 典型比例（地下狭长空间）

    for h in range(2, int(total ** 0.5) + 1):
        if total % h == 0:
            w = total // h
            # 检查 (h, w) —— h < w
            diff = abs(h / w - target_ratio)
            if diff < min_diff:
                min_diff = diff
                best_h, best_w = h, w
            # 检查 (w, h) —— w > h，即高度>宽度的情况
            diff = abs(w / h - target_ratio)
            if diff < min_diff:
                min_diff = diff
                best_h, best_w = w, h

    return best_h, best_w


def world_to_pixel(wx: float, wy: float, map_meta: dict) -> tuple:
    """世界坐标(米) → 像素坐标"""
    px = int((wx - map_meta['origin_x']) / map_meta['resolution'])
    py = int((wy - map_meta['origin_y']) / map_meta['resolution'])
    return px, py


def pixel_to_world(px: int, py: int, map_meta: dict) -> tuple:
    """像素坐标 → 世界坐标(米)"""
    wx = px * map_meta['resolution'] + map_meta['origin_x']
    wy = py * map_meta['resolution'] + map_meta['origin_y']
    return wx, wy
