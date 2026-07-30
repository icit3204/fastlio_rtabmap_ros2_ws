# [IMPL] F-13.5 / F-14.4 共用的时间戳就近匹配工具函数
# F-13.5: 视频帧与轨迹姿态时间戳对齐（最近邻匹配）
# F-14.4: 图片帧与轨迹姿态时间戳对齐（最近邻匹配）

import re
import bisect

FNAME_RE = re.compile(
    r'^(\d+)_t([\d.]+)_x([+-][\d.]+)_y([+-][\d.]+)\.jpg$'
)


def parse_frame_timestamp(filename: str) -> float | None:
    """从图片文件名解析时间戳 t；解析失败返回 None"""
    m = FNAME_RE.match(filename)
    return float(m.group(2)) if m else None


def find_nearest_by_timestamp(target_ts: float, items: list, ts_key) -> int:
    """
    在 items 中按时间戳就近查找，返回下标。
    items: 任意有序/无序列表；ts_key: 取出每项时间戳的函数
    使用二分近邻查找。
    """
    if not items:
        return -1
    sorted_idx = sorted(range(len(items)), key=lambda i: ts_key(items[i]))
    ts_list = [ts_key(items[i]) for i in sorted_idx]
    pos = bisect.bisect_left(ts_list, target_ts)
    if pos == 0:
        return sorted_idx[0]
    if pos == len(ts_list):
        return sorted_idx[-1]
    before, after = ts_list[pos - 1], ts_list[pos]
    closer = sorted_idx[pos - 1] if (target_ts - before) <= (after - target_ts) else sorted_idx[pos]
    return closer
