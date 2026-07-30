# core/car_indicator.py
# 🚗 车辆位置指示器
# 优先使用系统 Noto Color Emoji 字体渲染🚗，不可用时自动切换为 QPainter 绘制的俯视车形图标。
# 两种方式均支持 yaw 旋转。

import math
import subprocess

from PyQt5.QtWidgets import QGraphicsItem, QGraphicsTextItem
from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import (
    QPainter, QColor, QBrush, QPen, QFont, QFontDatabase,
    QPolygonF, QPainterPath,
)


# ─── 字体检测（模块级，只执行一次）────────────────────────────

def _detect_emoji_support() -> bool:
    """
    检测系统是否安装了 Noto Color Emoji 字体。
    先用 fc-list 快速检测，再用 QFontDatabase 验证 Qt 能否识别。
    """
    # 方法1：fc-list（Linux/Ubuntu 通用）
    try:
        result = subprocess.run(
            ['fc-list', ':family=Noto Color Emoji'],
            capture_output=True, text=True, timeout=3
        )
        if 'Noto Color Emoji' in result.stdout:
            return True
    except Exception:
        pass

    # 方法2：QFontDatabase 直接检查
    db = QFontDatabase()
    families = [f.lower() for f in db.families()]
    if any('noto color emoji' in f for f in families):
        return True
    if any('segoe ui emoji' in f for f in families):  # Windows fallback
        return True

    return False


# 模块加载时执行一次检测，结果缓存
_EMOJI_SUPPORTED: bool | None = None


def emoji_supported() -> bool:
    """返回当前系统是否支持彩色 Emoji 渲染（结果缓存）"""
    global _EMOJI_SUPPORTED
    if _EMOJI_SUPPORTED is None:
        _EMOJI_SUPPORTED = _detect_emoji_support()
    return _EMOJI_SUPPORTED


# ─── 图元基类 ──────────────────────────────────────────────

class CarIndicatorItem(QGraphicsItem):
    """
    车辆指示器图元基类。
    子类实现 boundingRect() 和 paint()。
    外部调用 set_pose(px, py, yaw_deg) 更新位置和朝向。
    """

    def __init__(self):
        super().__init__()
        self.setZValue(20)  # 绘制在所有图层之上

    def set_pose(self, px: float, py: float, yaw_deg: float):
        """更新场景位置和旋转角度（yaw_deg 为 Qt 旋转角，顺时针为正）"""
        self.setPos(px, py)
        self.setRotation(yaw_deg)


# ─── Emoji 实现 ────────────────────────────────────────────

class EmojiCarItem(CarIndicatorItem):
    """
    使用 🚗 emoji 渲染的车辆指示器。
    通过 Noto Color Emoji 字体绘制，支持 yaw 旋转。
    默认车头朝右（+x 方向），与 ROS yaw=0 对应。
    """

    SIZE = 28  # 字体 pt 大小

    def __init__(self):
        super().__init__()
        # 构建专用 emoji 字体
        self._font = QFont('Noto Color Emoji', self.SIZE)
        # Windows 备选
        if 'Noto Color Emoji' not in QFontDatabase().families():
            self._font = QFont('Segoe UI Emoji', self.SIZE)
        self._font.setStyleStrategy(QFont.PreferMatch)
        # 设置变换原点为图元中心（使旋转以车辆中心为轴）
        self.setTransformOriginPoint(0, 0)

    def set_pose(self, px: float, py: float, yaw_deg: float):
        """🚗 emoji 默认车头朝左，需旋转 180° 修正为车头朝右（+x）"""
        super().set_pose(px, py, yaw_deg + 180)

    def boundingRect(self) -> QRectF:
        # 以原点为中心的包围盒（近似值，足够鼠标事件检测）
        half = self.SIZE * 1.2
        return QRectF(-half, -half, half * 2, half * 2)

    def paint(self, painter: QPainter, option, widget=None):
        painter.setRenderHint(QPainter.TextAntialiasing)
        painter.setFont(self._font)
        # 在原点附近绘制 emoji（向左/上偏移使其居中）
        offset = int(self.SIZE * 0.9)
        painter.drawText(-offset, offset // 2, '🚗')


# ─── QPainter 俯视车形实现 ──────────────────────────────────

class DrawnCarItem(CarIndicatorItem):
    """
    无需 emoji 字体，用 QPainter 绘制俯视角车辆图标。
    车头朝右（+x 方向），与 EmojiCarItem 方向一致。
    颜色方案接近 🚗 emoji：黄色车身，蓝色挡风玻璃，红色前保险杠。
    """

    SIZE = 36  # 车辆长度（像素）

    def __init__(self):
        super().__init__()
        self.setTransformOriginPoint(0, 0)

    def boundingRect(self) -> QRectF:
        s = self.SIZE
        return QRectF(-s * 0.7, -s * 0.5, s * 1.4, s)

    def paint(self, painter: QPainter, option, widget=None):
        painter.setRenderHint(QPainter.Antialiasing)
        s = self.SIZE

        # ── 坐标系：车头朝右（+x），以原点为车辆中心 ──
        L = s * 0.65    # 车半长
        W = s * 0.38    # 车半宽

        # 车身（黄色圆角矩形）
        painter.setBrush(QBrush(QColor('#f5c518')))
        painter.setPen(QPen(QColor('#8b6508'), 1.2))
        painter.drawRoundedRect(QRectF(-L, -W, L * 2, W * 2), 4, 4)

        # 前挡风玻璃（车头方向，浅蓝色）
        painter.setBrush(QBrush(QColor('#a8d8ea')))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(
            QRectF(L * 0.05, -W * 0.65, L * 0.45, W * 1.3), 2, 2
        )

        # 后挡风玻璃（车尾方向，深蓝色）
        painter.setBrush(QBrush(QColor('#7ab8d4')))
        painter.drawRoundedRect(
            QRectF(-L * 0.52, -W * 0.55, L * 0.38, W * 1.1), 2, 2
        )

        # 四个车轮（黑色）
        painter.setBrush(QBrush(QColor('#222222')))
        ww, wh = s * 0.14, s * 0.22
        for wx, wy in [
            (L * 0.28,  W * 0.76),
            (L * 0.28, -W * 0.98),
            (-L * 0.62,  W * 0.76),
            (-L * 0.62, -W * 0.98),
        ]:
            painter.drawRoundedRect(QRectF(wx, wy, ww, wh), 2, 2)

        # 前保险杠（红色条，在车头最右侧）
        painter.setBrush(QBrush(QColor('#e24b4a')))
        painter.setPen(Qt.NoPen)
        painter.drawRect(QRectF(L * 0.88, -W * 0.85, s * 0.08, W * 1.7))


# ─── 公共工厂函数 ──────────────────────────────────────────

def create_car_item() -> CarIndicatorItem:
    """
    工厂函数：返回适合当前系统的车辆指示器图元。
    - 有 Noto Color Emoji → EmojiCarItem（🚗）
    - 无 emoji 字体      → DrawnCarItem（QPainter 绘制的等效车形）
    """
    if emoji_supported():
        return EmojiCarItem()
    return DrawnCarItem()


def yaw_to_rotation(yaw_rad: float) -> float:
    """
    将 ROS yaw（弧度，逆时针为正）转换为 Qt setRotation 角度（顺时针为正）。
    车辆图元默认车头朝右（+x），与 ROS yaw=0 一致，无需额外偏移。
    """
    return -math.degrees(yaw_rad)
