# [DONE] F-10.3 Roboflow API 调用封装（重试 + 退避）
# [DONE] F-10.4 yes 判定（置信度 + 对称误差）
# [DONE] F-10.5 文件名解析 + db_nodes 最近邻取 z/yaw
# [DONE] F-10.6 result 清空 + 全图带框保存 + CSV 三件套
# [DONE] F-10.7 ObjectDetectorThread 异步线程 + 信号

"""目标识别核心算法 — Underground Map Editor F-10.x

本模块包含：
- Roboflow API 封装与重试
- yes 框判定逻辑
- 图片文件名解析与轨迹 z/yaw 注入
- result 目录清空、带框可视化、CSV 三件套输出
- ObjectDetectorThread 异步识别线程
"""

import os
import re
import csv
import base64
import time
from collections import Counter

import cv2
import numpy as np
import requests

from PyQt5.QtCore import QThread, pyqtSignal


# ============================================================
# 常量定义
# ============================================================

# Roboflow 推理 URL 模板（{model}/{ver} 由调用方填充）
ROBOFLOW_URL_TMPL = "https://detect.roboflow.com/{model}/{ver}"

# 默认模型与版本号
DEFAULT_MODEL = "carplace-1krak"
DEFAULT_VERSION = 1

# DEFAULT_MODEL = "parking-space-ipm1b-bggyz"
# DEFAULT_VERSION = 1



# 解析阶段使用的最低置信度阈值
DEFAULT_CONF = 0.15

# 网络请求总超时（秒）
TIMEOUT = 60.0

# 最大重试次数与线性退避基础间隔（秒）
RETRIES = 5
RETRY_SLEEP = 3.0

# 支持的图片扩展名
SUPPORTED_EXTS = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')

# 类别名归一化映射表（容错拼写/复数/同义词）
CLASS_ALIASES = {
    'car': 'car', 'cars': 'car', 'vehicle': 'car',
    'empty': 'empty_space', 'empty_space': 'empty_space',
    'empty space': 'empty_space', 'free': 'empty_space', 'vacant': 'empty_space',
    'occupied': 'occupied_space', 'ocupied': 'occupied_space',
    'occupied_space': 'occupied_space', 'occupied space': 'occupied_space',
}

# 各类别在可视化绘制中使用的 BGR 颜色
COLORS = {
    'car':            (0, 180, 255),
    'empty_space':    (0, 220, 0),
    'occupied_space': (0, 60, 255),
}

# F-10.5 文件名格式：NNNN_tT.TTT_x±X.XXXX_y±Y.YYYY.jpg
# 序号(NNNN)仅用于排序，本模块不消费；只提取 (x, y, t) 三元组
FNAME_RE = re.compile(
    r'^\d+_t(\d+\.\d{3})_x([+-]?\d+\.\d{4})_y([+-]?\d+\.\d{4})\.jpg$',
    re.IGNORECASE
)


# ============================================================
# F-10.3  本地 YOLO 推理（替代 Roboflow API）
# ============================================================
# [FIX] 2026-06-29 Roboflow detect.roboflow.com 国内被墙，改用本地训练的 YOLOv11n 模型

_LOCAL_MODEL = None  # 模块级缓存，首次调用时加载


def _get_local_model():
    """懒加载本地 YOLO 模型（模块级单例）"""
    global _LOCAL_MODEL
    if _LOCAL_MODEL is None:
        from ultralytics import YOLO
        import os, sys
        # 优先使用本地训练模型，其次用预训练 yolo11n
        if getattr(sys, 'frozen', False):
            project_root = sys._MEIPASS
        else:
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        local_pt = os.path.join(project_root, 'models', 'carplace_train', 'weights', 'best.pt')
        if os.path.exists(local_pt):
            _LOCAL_MODEL = YOLO(local_pt)
        else:
            _LOCAL_MODEL = YOLO('yolo11n.pt')
    return _LOCAL_MODEL


def roboflow_call(img_path, api_key):
    """使用本地 YOLO 模型推理，返回与 Roboflow API 兼容的 JSON 格式。

    参数:
        img_path (str): 待识别图片的本地路径
        api_key (str):  保留参数（本地推理不使用），维持接口兼容

    返回:
        dict: 与 Roboflow 响应格式一致的 JSON 对象
    """
    model = _get_local_model()
    results = model(img_path, conf=0.15, iou=0.3, verbose=False)

    predictions = []
    if results and len(results) > 0:
        r = results[0]
        if r.boxes is not None:
            cls_map = {0: 'empty_space', 1: 'occupied_space'}
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                w = x2 - x1
                h = y2 - y1
                cx = x1 + w / 2.0
                cy = y1 + h / 2.0
                predictions.append({
                    'class': cls_map.get(cls_id, 'unknown'),
                    'confidence': conf,
                    'x': cx,
                    'y': cy,
                    'width': w,
                    'height': h,
                })

    return {'predictions': predictions}


# ============================================================
# F-10.4  解析 Roboflow 预测结果
# ============================================================

def _normalize_class(name):
    """类别名归一化：小写后查 CLASS_ALIASES，未命中则保留原始 strip 字符串。"""
    if not isinstance(name, str):
        return str(name).strip()
    key = name.strip().lower()
    return CLASS_ALIASES.get(key, name.strip())


def parse_predictions(raw_json):
    """将 Roboflow 返回的 predictions 数组转为内部字典列表。

    输入预测使用中心点 + 宽高（x, y, width, height），
    本函数转换为左上/右下坐标（x1, y1, x2, y2），并过滤低置信度框。

    参数:
        raw_json (dict): Roboflow 响应 JSON

    返回:
        list[dict]: 每项包含 class_name / confidence / x1 / y1 / x2 / y2
    """
    out = []
    if not isinstance(raw_json, dict):
        return out

    preds = raw_json.get('predictions', []) or []
    for p in preds:
        try:
            conf = float(p.get('confidence', 0.0))
        except (TypeError, ValueError):
            continue
        # 低于默认阈值的预测直接跳过
        if conf < DEFAULT_CONF:
            continue

        try:
            cx = float(p.get('x', 0.0))
            cy = float(p.get('y', 0.0))
            w = float(p.get('width', 0.0))
            h = float(p.get('height', 0.0))
        except (TypeError, ValueError):
            continue

        out.append({
            'class_name': _normalize_class(p.get('class', '')),
            'confidence': conf,
            'x1': cx - w / 2.0,
            'y1': cy - h / 2.0,
            'x2': cx + w / 2.0,
            'y2': cy + h / 2.0,
        })
    return out


# ============================================================
# F-10.4  yes 判定（置信度 + 水平轴对称误差）
# ============================================================

def mark_axis_symmetric_yes(img_path, preds, yes_conf, axis_tol):
    """在 preds 中挑选最对称的高置信框，并标记 axis_symmetric_yes=True。

    判定规则：
      1. 对每个框计算水平方向对称误差：abs((x1+x2) - W) / W
      2. 候选条件：confidence > yes_conf  且  axis_symmetry_error <= axis_tol
      3. 候选中按 (对称误差升序, 置信度降序) 选取第一个标 yes

    参数:
        img_path (str):  图片路径（用于读取宽度）
        preds (list):    预测列表（会被原地添加 axis_symmetry_error / axis_symmetric_yes）
        yes_conf (float): yes 判定的置信度下限（严格大于）
        axis_tol (float): 允许的对称误差上限（含等于）

    返回:
        list: 原 preds 列表（已注入两个新字段）
    """
    # 图片读取失败 → 直接返回原列表（无图无法计算宽度）
    img = cv2.imread(img_path)
    if img is None:
        return preds

    image_width = float(img.shape[1])
    if image_width <= 0:
        return preds

    # 初始化所有框的对称误差与默认 yes=False
    candidates = []
    for idx, p in enumerate(preds):
        try:
            err = abs((float(p['x1']) + float(p['x2'])) - image_width) / image_width
        except (KeyError, TypeError, ValueError):
            err = float('inf')
        p['axis_symmetry_error'] = err
        p['axis_symmetric_yes'] = False

        # 同时满足置信度与对称误差阈值才进入候选
        if p.get('confidence', 0.0) > yes_conf and err <= axis_tol:
            candidates.append((err, -float(p['confidence']), idx))

    # 候选按 (误差升序, 置信度降序) 排序，取第一名
    if candidates:
        candidates.sort(key=lambda t: (t[0], t[1]))
        winner_idx = candidates[0][2]
        preds[winner_idx]['axis_symmetric_yes'] = True

    return preds


# ============================================================
# F-10.5  文件名解析 + db_nodes 最近邻取 z/yaw
# ============================================================

def parse_filename_xyt(fname):
    """从形如 0001_t1778851623.092_x+0.8461_y-2.0429.jpg 的文件名提取 (x, y, t)。

    参数:
        fname (str): 文件名（不含路径）

    返回:
        tuple[float, float, float] | None: 匹配失败返回 None
    """
    if not isinstance(fname, str):
        return None
    # 仅取 basename，避免误传完整路径
    base = os.path.basename(fname)
    m = FNAME_RE.match(base)
    if not m:
        return None
    try:
        # 正则捕获顺序：(t, x, y)；返回保持原签名 (x, y, t)
        t = float(m.group(1))
        x = float(m.group(2))
        y = float(m.group(3))
    except (TypeError, ValueError):
        return None
    return (x, y, t)


def lookup_zyaw(db_nodes, t):
    """根据时间戳在 db_nodes 中最近邻查找 z/yaw。

    参数:
        db_nodes (list[dict]): 每项需包含 'timestamp' 字段，可选 'z' / 'yaw'
        t (float):             目标时间戳

    返回:
        tuple[float, float]: (z, yaw)。空列表或 None 时返回 (0.0, 0.0)
    """
    if not db_nodes:
        return (0.0, 0.0)
    try:
        nearest = min(db_nodes, key=lambda n: abs(float(n['timestamp']) - float(t)))
    except (KeyError, TypeError, ValueError):
        return (0.0, 0.0)
    z = float(nearest.get('z', 0.0)) if isinstance(nearest, dict) else 0.0
    yaw = float(nearest.get('yaw', 0.0)) if isinstance(nearest, dict) else 0.0
    return (z, yaw)


# ============================================================
# F-10.6  result 目录清理 + 带框可视化 + CSV 三件套
# ============================================================

def clear_result_dir(result_dir: str) -> int:
    """清空结果目录（保留目录本身与子目录）。

    仿 video_processor.clear_pic_dir 实现：
    - 确保目录存在
    - 仅删除一级文件，不递归子目录
    - 返回成功删除的文件数量
    """
    # 确保目录存在
    os.makedirs(result_dir, exist_ok=True)

    deleted = 0
    # 遍历目录条目
    try:
        entries = os.listdir(result_dir)
    except OSError:
        # 目录不可访问 → 视为未删除任何文件
        return 0

    for name in entries:
        full = os.path.join(result_dir, name)
        # 仅删除文件，保留子目录
        if os.path.isfile(full):
            try:
                os.remove(full)
                deleted += 1
            except OSError:
                # 单个删除失败不计入统计
                pass
    return deleted


def draw_predictions(src_path: str, dst_path: str, preds: list) -> bool:
    """在源图上绘制所有预测框，并将命中 yes 的框中心标注红色 'yes'。

    参数:
        src_path (str): 源图路径（支持中文路径）
        dst_path (str): 写盘目标路径（使用 imencode + tofile 规避中文路径问题）
        preds (list):   预测列表，元素需包含 class_name/confidence/x1/y1/x2/y2，
                        可选 axis_symmetric_yes

    返回:
        bool: 是否成功写盘
    """
    # 读图（OpenCV 自身对中文路径兼容性差 → 使用 imread 直读，
    # 若失败可改 imdecode；此处与设计保持一致直接 imread）
    img = cv2.imread(src_path)
    if img is None:
        # 部分中文路径下 imread 会返回 None，尝试以二进制读后 imdecode
        try:
            with open(src_path, 'rb') as f:
                data = np.frombuffer(f.read(), dtype=np.uint8)
            img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        except Exception:
            img = None
        if img is None:
            return False

    font = cv2.FONT_HERSHEY_SIMPLEX

    for p in preds:
        cls = str(p.get('class_name', ''))
        # 默认颜色为浅蓝色，未知类别仍可被绘制
        color = COLORS.get(cls, (255, 180, 0))

        # 取整数化坐标
        try:
            x1 = int(round(float(p.get('x1', 0))))
            y1 = int(round(float(p.get('y1', 0))))
            x2 = int(round(float(p.get('x2', 0))))
            y2 = int(round(float(p.get('y2', 0))))
        except (TypeError, ValueError):
            continue

        # 画框（粗细 2）
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # 文本标签：类别 + 置信度
        try:
            conf = float(p.get('confidence', 0.0))
        except (TypeError, ValueError):
            conf = 0.0
        label = f"{cls} {conf:.2f}"

        # 计算文本高度，避免标签贴顶时被裁切
        (tw, th), _ = cv2.getTextSize(label, font, 0.5, 1)
        y_text = max(y1, th + 4)
        cv2.putText(img, label, (x1 + 2, y_text - 3),
                    font, 0.5, color, 1, cv2.LINE_AA)

        # 若命中 yes → 在框中心叠加红色 yes 标记（先白底大粗，再红字细描边）
        if p.get('axis_symmetric_yes', False):
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            yes_text = 'yes'
            # 先写白色底（更粗）作为描边
            cv2.putText(img, yes_text, (cx - 20, cy + 8),
                        font, 1.2, (255, 255, 255), 5, cv2.LINE_AA)
            # 再写红色前景（更细）
            cv2.putText(img, yes_text, (cx - 20, cy + 8),
                        font, 1.2, (0, 0, 255), 2, cv2.LINE_AA)

    # 写盘：使用 imencode + tofile 以支持中文路径
    try:
        ok_enc, buf = cv2.imencode('.jpg', img)
        if not ok_enc:
            return False
        buf.tofile(dst_path)
        return True
    except Exception:
        return False


def _write_csv(path: str, header: list, rows: list) -> None:
    """内部辅助：以 utf-8 / newline='' 写入一个 CSV 文件。"""
    # 确保父目录存在
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow(r)


def write_csv_summary(result_dir: str, rows: list) -> str:
    """写入 summary.csv（每张图一行：car / empty / occupied / 占用率）。"""
    path = os.path.join(result_dir, 'summary.csv')
    header = ['frame', 'source', 'car', 'empty_space',
              'occupied_space', 'occupancy_ratio']
    _write_csv(path, header, rows)
    return path


def write_csv_detections(result_dir: str, rows: list) -> str:
    """写入 detections.csv（每条预测一行：坐标 + 对称误差 + yes 标记）。"""
    path = os.path.join(result_dir, 'detections.csv')
    header = ['frame', 'source', 'class_name', 'confidence',
              'x1', 'y1', 'x2', 'y2',
              'axis_symmetry_error', 'axis_symmetric_yes']
    _write_csv(path, header, rows)
    return path


def write_csv_failed(result_dir: str, rows: list) -> str:
    """写入 failed.csv（重试耗尽仍失败的图片记录）。"""
    path = os.path.join(result_dir, 'failed.csv')
    header = ['frame', 'source', 'error']
    _write_csv(path, header, rows)
    return path


# ============================================================
# F-10.7  ObjectDetectorThread 异步识别线程
# ============================================================

class ObjectDetectorThread(QThread):
    """F-10.7 异步识别线程。

    负责：
    1. 清空 result 目录
    2. 列出 pic 目录所有图片，逐张调用 Roboflow API
    3. 解析预测、标 yes、画框写盘
    4. 维护 summary / detections / failed 三个 CSV
    5. 命中 yes 时解析文件名 + 查 z/yaw，发出 yes_detected 信号
    """

    # ---- 信号定义 ----
    # 命中 yes 时发出：x, y, z, yaw, t, fname
    yes_detected = pyqtSignal(float, float, float, float, float, str)
    # 进度信号：当前张数 / 总张数
    progress = pyqtSignal(int, int)
    # 日志信号：消息文本 + 级别 (info/warn/error)
    log_message = pyqtSignal(str, str)
    # 结束信号：{'total','yes','added','failed','reason'}
    finished_with_stats = pyqtSignal(dict)

    def __init__(self, pic_dir, result_dir, db_nodes,
                 api_key, yes_conf, axis_tolerance):
        super().__init__()
        # 输入：图片目录、输出目录、轨迹节点（用于查 z/yaw）
        self.pic_dir = pic_dir
        self.result_dir = result_dir
        self.db_nodes = db_nodes
        # 推理参数
        self.api_key = api_key
        self.yes_conf = yes_conf
        self.axis_tolerance = axis_tolerance
        # 运行标志（供外部 stop 调用）
        self._running = True

    def stop(self):
        """外部调用：请求线程在下一帧前退出循环。"""
        self._running = False

    def run(self):
        # 1. 清空 result 目录
        clear_result_dir(self.result_dir)

        # 2. 列出 pic 目录所有图片（按文件名升序，因命名含 timestamp）
        try:
            files = sorted(
                f for f in os.listdir(self.pic_dir)
                if f.lower().endswith(SUPPORTED_EXTS)
            )
        except OSError as exc:
            # pic 目录无法访问 → 直接结束
            self.finished_with_stats.emit({
                'total': 0, 'yes': 0, 'added': 0, 'failed': 0,
                'reason': f'pic 目录不可访问: {exc}',
            })
            return

        total = len(files)
        if total == 0:
            self.finished_with_stats.emit({
                'total': 0, 'yes': 0, 'added': 0, 'failed': 0,
                'reason': 'pic 目录为空',
            })
            return

        # CSV 行缓存
        summary_rows = []
        detection_rows = []
        failed_rows = []
        yes_cnt = 0
        added_cnt = 0
        failed_cnt = 0

        # 3. 逐张处理
        for i, fname in enumerate(files, start=1):
            # 响应外部停止请求
            if not self._running:
                break

            # 进度信号（按当前/总数）
            self.progress.emit(i, total)
            img_path = os.path.join(self.pic_dir, fname)

            # 3.1 API 调用（含重试）
            try:
                raw = roboflow_call(img_path, self.api_key)
            except Exception as exc:
                failed_cnt += 1
                failed_rows.append([i, img_path, str(exc)])
                self.log_message.emit(
                    f'[{i}/{total}] {fname} 识别失败（已重试 {RETRIES} 次）',
                    'error'
                )
                continue

            # 3.2 解析 + yes 判定
            preds = parse_predictions(raw)
            preds = mark_axis_symmetric_yes(
                img_path, preds, self.yes_conf, self.axis_tolerance
            )
            counts = Counter(p['class_name'] for p in preds)

            # 3.3 画框写盘（result 目录沿用原文件名）
            out_path = os.path.join(self.result_dir, fname)
            draw_predictions(img_path, out_path, preds)

            # 3.4 CSV 行收集
            empty = counts.get('empty_space', 0)
            occupied = counts.get('occupied_space', 0)
            denom = empty + occupied
            occupancy = (occupied / denom) if denom else 0.0
            summary_rows.append([
                i, img_path,
                counts.get('car', 0), empty, occupied, occupancy,
            ])
            for p in preds:
                detection_rows.append([
                    i, img_path,
                    p['class_name'], p['confidence'],
                    p['x1'], p['y1'], p['x2'], p['y2'],
                    p.get('axis_symmetry_error', ''),
                    bool(p.get('axis_symmetric_yes', False)),
                ])

            # 3.5 单图完成日志
            self.log_message.emit(
                f'[{i}/{total}] {fname} → car={counts.get("car", 0)} '
                f'empty={empty} occupied={occupied}',
                'info'
            )

            # 3.6 命中 yes → 文件名解析 → 查 z/yaw → 发信号
            if any(p.get('axis_symmetric_yes') for p in preds):
                yes_cnt += 1
                parsed = parse_filename_xyt(fname)
                if parsed is None:
                    self.log_message.emit(
                        f'[{i}/{total}] yes 命中但文件名格式异常，跳过节点添加',
                        'error'
                    )
                    continue
                x, y, t = parsed
                z, yaw = lookup_zyaw(self.db_nodes, t)
                self.yes_detected.emit(x, y, z, yaw, t, fname)
                added_cnt += 1

        # 4. 写出 CSV 三件套
        write_csv_summary(self.result_dir, summary_rows)
        write_csv_detections(self.result_dir, detection_rows)
        write_csv_failed(self.result_dir, failed_rows)

        # 5. 结束信号
        self.finished_with_stats.emit({
            'total': total,
            'yes': yes_cnt,
            'added': added_cnt,
            'failed': failed_cnt,
            'reason': '',
        })
