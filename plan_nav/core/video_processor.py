# [DONE] F-9.2 视频源选择 (最新 mtime → 并列取最长时长)
# [DONE] F-9.3 通用视频格式支持
# [DONE] F-9.8 输出目录清空策略
# [DONE] F-9.4 时间戳对齐抽帧
# [DONE] F-9.5 终止条件 (视频先结束/轨迹先结束)
# [DONE] F-9.6 720p 仅尺寸缩放
# [DONE] F-9.7 文件命名规则 (x±X.XXXX_y±Y.YYYY_tT.TTT.jpg)
# [DONE] F-9.10 进度回调反馈

"""视频抽帧与图像导出模块 — Underground Map Editor F-9.x"""

import os
import glob
import cv2
import numpy as np

# F-9.3 通用视频格式支持：支持的视频扩展名（小写）
SUPPORTED_EXTS = ('.mp4', '.avi', '.mov', '.mkv', '.flv', '.wmv', '.webm')


def _video_duration(path: str) -> float:
    """内部辅助函数：估算视频时长（秒）。

    通过 OpenCV 读取 FPS 与帧数计算时长；打开失败时返回 0.0。
    """
    cap = None
    try:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            return 0.0
        fps = cap.get(cv2.CAP_PROP_FPS)
        cnt = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        # 避免除零：分母至少为 1e-3
        return float(cnt) / max(float(fps), 1e-3)
    except Exception:
        # 任何异常视为无法读取
        return 0.0
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


def pick_video(video_dir: str) -> str | None:
    """F-9.2 视频源选择。

    规则：列出目录下受支持的视频文件，按 mtime 降序；
    取与最大 mtime "同时刻"（差异 < 1e-3 秒）的全部候选；
    若仅 1 个直接返回，否则在并列候选中选取时长最长者。
    无候选返回 None。
    """
    # 列出目录下所有条目
    all_paths = glob.glob(os.path.join(video_dir, '*'))

    # 过滤为受支持的视频文件（扩展名不区分大小写，且必须是文件）
    candidates = []
    for p in all_paths:
        if not os.path.isfile(p):
            continue
        ext = os.path.splitext(p)[1].lower()
        if ext in SUPPORTED_EXTS:
            candidates.append(p)

    # 无候选直接返回 None
    if not candidates:
        return None

    # 按 mtime 降序排列
    candidates.sort(key=lambda x: os.path.getmtime(x), reverse=True)

    # 取最大 mtime
    max_mtime = os.path.getmtime(candidates[0])

    # 找出与最大 mtime "同时刻" 的全部候选
    tied = [p for p in candidates if abs(os.path.getmtime(p) - max_mtime) < 1e-3]

    # 仅 1 个直接返回
    if len(tied) == 1:
        return tied[0]

    # 多个并列：取时长最长者
    best = tied[0]
    best_dur = _video_duration(best)
    for p in tied[1:]:
        dur = _video_duration(p)
        if dur > best_dur:
            best = p
            best_dur = dur
    return best


def clear_pic_dir(pic_dir: str) -> int:
    """F-9.8 清空输出目录。

    确保目录存在，仅删除目录下的文件（保留子目录与目录本身），
    返回删除的文件数量。
    """
    # 确保目录存在
    os.makedirs(pic_dir, exist_ok=True)

    deleted = 0
    # 遍历目录条目
    try:
        entries = os.listdir(pic_dir)
    except OSError:
        # 目录不可访问，视为未删除任何文件
        return 0

    for name in entries:
        full = os.path.join(pic_dir, name)
        # 仅删除文件，不删除子目录
        if os.path.isfile(full):
            try:
                os.remove(full)
                deleted += 1
            except OSError:
                # 删除失败跳过，不计数
                pass
    return deleted


def extract_frames(video_path: str, nodes: list, pic_dir: str,
                   progress_cb=None) -> dict:
    """F-9.4 / F-9.5 / F-9.6 视频按轨迹时间戳对齐抽帧。

    - F-9.4：以 nodes[0]['timestamp'] 为 t0，按 dt = ts - t0 在视频中定位抽帧
    - F-9.5：若 dt 超过视频时长则停止抽帧（视频先结束，后续轨迹丢弃）
    - F-9.6：仅在高度不为 720 时按比例缩放至 720p（不改变其他编码参数）

    返回统计字典：{'total': int, 'saved': int, 'skipped': int, 'reason': str}
    """
    # 打开视频；失败直接返回
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        # 确保释放（OpenCV 中即使未打开也建议 release）
        try:
            cap.release()
        except Exception:
            pass
        return {'total': 0, 'saved': 0, 'skipped': 0, 'reason': '无法打开视频'}

    # 读取视频元数据
    video_fps = cap.get(cv2.CAP_PROP_FPS)
    video_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    # 避免除零：fps 至少为 1e-3
    video_duration = float(video_frames) / max(float(video_fps), 1e-3)

    # 轨迹为空：释放后返回
    if not nodes:
        try:
            cap.release()
        except Exception:
            pass
        return {'total': 0, 'saved': 0, 'skipped': 0, 'reason': '轨迹为空'}

    # 确保输出目录存在
    os.makedirs(pic_dir, exist_ok=True)

    # F-9.4 取首个节点时间戳作为基准 t0
    t0 = nodes[0]['timestamp']

    saved = 0
    skipped = 0
    total = len(nodes)

    # 遍历每个轨迹节点
    for i, node in enumerate(nodes):
        # 相对时间偏移（秒）
        dt = node['timestamp'] - t0

        # 防御：理论不应出现负偏移
        if dt < 0:
            skipped += 1
            if progress_cb is not None:
                try:
                    progress_cb(int((i + 1) / total * 100))
                except Exception:
                    # 回调失败不影响抽帧主流程
                    pass
            continue

        # F-9.5 视频先结束 → 跳出循环（后续轨迹点全部丢弃）
        if dt > video_duration:
            break

        # 在视频中按毫秒定位
        cap.set(cv2.CAP_PROP_POS_MSEC, dt * 1000.0)
        ok, frame = cap.read()
        if not ok or frame is None:
            skipped += 1
            if progress_cb is not None:
                try:
                    progress_cb(int((i + 1) / total * 100))
                except Exception:
                    pass
            continue

        # F-9.6 仅尺寸缩放：若高度不为 720 按比例缩放
        h, w = frame.shape[:2]
        if h != 720:
            new_w = int(round(w * 720 / h))
            frame = cv2.resize(frame, (new_w, 720), interpolation=cv2.INTER_AREA)

        # F-9.7 文件命名：NNNN_tT.TTT_x±X.XXXX_y±Y.YYYY.jpg
        # 序号前缀(4位0填充)保证文件按时间戳顺序排序，时间戳保留供 auto_node 反查 z/yaw
        fname = (f"{i+1:04d}_t{node['timestamp']:.3f}"
                 f"_x{node['x']:+.4f}_y{node['y']:+.4f}.jpg")
        # 使用 imencode + ndarray.tofile 以支持非 ASCII（中文）路径
        try:
            ok_enc, buf = cv2.imencode('.jpg', frame)
            if ok_enc:
                buf.tofile(os.path.join(pic_dir, fname))
                saved += 1
            else:
                skipped += 1
        except Exception:
            # 写盘失败计入跳过
            skipped += 1

        # 进度回调
        if progress_cb is not None:
            try:
                progress_cb(int((i + 1) / total * 100))
            except Exception:
                pass

    # F-9.10 统一保证进度到达 100
    if progress_cb is not None:
        try:
            progress_cb(100)
        except Exception:
            pass

    # 释放视频资源
    try:
        cap.release()
    except Exception:
        pass

    return {'total': total, 'saved': saved, 'skipped': skipped, 'reason': ''}
