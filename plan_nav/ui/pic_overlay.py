# [IMPL] F-13.1 图片回放悬浮窗 — 图片显示、播放控制、进度拖动
# [IMPL] F-13.7 标题栏拖拽移动 — 鼠标按住标题栏区域可拖动窗口
# [IMPL] F-13.8 最小尺寸约束(220×260) + 右下角 QSizeGrip 缩放
# [IMPL] F-13.3 播放/暂停按钮 — 控制图片帧播放状态
# [IMPL] F-13.4 进度滑块 — 拖动跳转到指定帧
# [IMPL] F-13.10 QWidget 作为 map_view 子控件，键盘快捷键直通

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QSlider, QComboBox, QSizeGrip,
)
from PyQt5.QtCore import Qt, pyqtSignal, QPoint
from PyQt5.QtGui import QPixmap, QFont


from core.ui_font import mono_font  # [ADAPT-UBU-02] 跨平台等宽字体
class PicOverlay(QWidget):
    """图片回放悬浮窗：显示当前帧图片、提供播放/速度/进度控制。
    作为 map_view 的子控件，保证 Z/Space 等键盘快捷键透传（F-13.10）。"""

    # ─── 信号定义 ──────────────────────────────────────
    play_toggled = pyqtSignal()          # 播放/暂停按钮点击
    seek_requested = pyqtSignal(int)     # 滑块拖动，参数为帧索引
    speed_changed = pyqtSignal(float)    # 速度下拉框改变，参数为新倍率
    closed = pyqtSignal()               # 关闭按钮点击

    def __init__(self, parent):
        super().__init__(parent)
        # F-13.8 最小尺寸约束
        self.setMinimumSize(220, 260)
        self.resize(300, 320)

        # 外观：白色背景、细边框、圆角
        self.setStyleSheet(
            'PicOverlay {'
            '  background: #ffffff;'
            '  border: 0.5px solid #888780;'
            '  border-radius: 4px;'
            '}'
        )

        # 拖拽状态
        self._drag_pos = None

        self._setup_ui()
        # 初始隐藏
        self.hide()

    # ─── UI 构建 ───────────────────────────────────────

    def _setup_ui(self):
        """构建完整 UI 布局：标题栏 + 图片区 + 帧信息 + 滑块 + 控制栏 + 缩放手柄"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # ── 标题栏 (F-13.7 拖拽区域) ──
        self._title_bar = QWidget()
        self._title_bar.setFixedHeight(26)
        self._title_bar.setStyleSheet(
            'background: #2c2c2a; border: none; border-radius: 4px 4px 0px 0px;'
        )
        title_layout = QHBoxLayout(self._title_bar)
        title_layout.setContentsMargins(8, 0, 2, 0)
        title_layout.setSpacing(0)

        title_label = QLabel('图片回放')
        title_label.setFont(mono_font(9))
        title_label.setStyleSheet('color: #ffffff; background: transparent; border: none;')
        title_layout.addWidget(title_label)
        title_layout.addStretch()

        # 关闭按钮
        self._btn_close = QPushButton('×')  # × 符号
        self._btn_close.setFixedSize(18, 18)
        self._btn_close.setFont(mono_font(10))
        self._btn_close.setStyleSheet(
            'QPushButton {'
            '  background: transparent; color: #ffffff; border: none;'
            '  padding: 0px;'
            '}'
            'QPushButton:hover { color: #e24b4a; }'
        )
        self._btn_close.clicked.connect(self._on_close)
        title_layout.addWidget(self._btn_close)

        layout.addWidget(self._title_bar)

        # ── 图片显示区域 ──
        self._img_label = QLabel('无图片')
        self._img_label.setAlignment(Qt.AlignCenter)
        self._img_label.setStyleSheet(
            'QLabel {'
            '  background: #000000; color: #888780;'
            '  border: none;'
            '}'
        )
        self._img_label.setFont(mono_font(9))
        layout.addWidget(self._img_label, stretch=1)

        # ── 帧信息标签 ──
        self._frame_label = QLabel('帧 -- / --')
        self._frame_label.setAlignment(Qt.AlignCenter)
        self._frame_label.setFont(mono_font(8))
        self._frame_label.setStyleSheet(
            'QLabel {'
            '  color: #2c2c2a; background: #ffffff; border: none;'
            '  padding: 2px 0px;'
            '}'
        )
        layout.addWidget(self._frame_label)

        # ── 进度滑块 (F-13.4) ──
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setStyleSheet(
            'QSlider::groove:horizontal {'
            '  border: none; background: #f4f2ed; height: 4px;'
            '  border-radius: 2px;'
            '}'
            'QSlider::handle:horizontal {'
            '  background: #1d9e75; border: none; width: 10px;'
            '  margin: -4px 0; border-radius: 5px;'
            '}'
        )
        self._slider.sliderMoved.connect(self.seek_requested.emit)
        layout.addWidget(self._slider)

        # ── 控制行：播放/暂停 + 速度下拉框 ──
        control_row = QHBoxLayout()
        control_row.setContentsMargins(4, 4, 4, 4)
        control_row.setSpacing(6)

        # 播放/暂停按钮
        self._btn_play = QPushButton('播放')
        self._btn_play.setFont(mono_font(8))
        self._btn_play.setStyleSheet(
            'QPushButton {'
            '  background: #1d9e75; color: #ffffff;'
            '  border: none; border-radius: 3px;'
            '  padding: 3px 8px;'
            '}'
            'QPushButton:hover { opacity: 0.85; }'
        )
        self._btn_play.clicked.connect(self.play_toggled.emit)
        control_row.addWidget(self._btn_play)

        # 速度下拉框
        self._speed_combo = QComboBox()
        self._speed_combo.setFont(mono_font(8))
        self._speed_combo.setStyleSheet(
            'QComboBox {'
            '  background: #f4f2ed; color: #2c2c2a;'
            '  border: 0.5px solid #d3d1c7; border-radius: 3px;'
            '  padding: 2px 4px;'
            '}'
            'QComboBox::drop-down { border: none; }'
        )
        speed_items = [
            ('0.5x', 0.5),
            ('1.0x', 1.0),
            ('1.5x', 1.5),
            ('2.0x', 2.0),
            ('3.0x', 3.0),
        ]
        for label, val in speed_items:
            self._speed_combo.addItem(label, val)
        self._speed_combo.setCurrentIndex(1)  # 默认 1.0x
        self._speed_combo.currentIndexChanged.connect(self._on_speed_changed)
        control_row.addWidget(self._speed_combo)

        control_row.addStretch()
        layout.addLayout(control_row)

        # ── 右下角缩放手柄 (F-13.8) ──
        grip_row = QHBoxLayout()
        grip_row.setContentsMargins(0, 0, 0, 0)
        grip_row.addStretch()
        grip_row.addWidget(QSizeGrip(self))
        layout.addLayout(grip_row)

    # ─── 拖拽逻辑 (F-13.7) ──────────────────────────────

    def mousePressEvent(self, event):
        """左键点击标题栏区域时记录偏移，准备拖拽"""
        if event.button() == Qt.LeftButton:
            # 只允许在标题栏高度范围内拖拽
            if event.pos().y() <= self._title_bar.height():
                self._drag_pos = event.globalPos() - self.frameGeometry().topLeft()
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """拖拽过程中移动窗口"""
        if self._drag_pos is not None:
            new_pos = event.globalPos() - self._drag_pos
            self.move(new_pos)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        """释放鼠标，结束拖拽"""
        if event.button() == Qt.LeftButton and self._drag_pos is not None:
            self._drag_pos = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    # ─── 公共接口 ───────────────────────────────────────

    def set_frame(self, pixmap: QPixmap, idx: int, total: int):
        """设置当前帧图片、索引和总帧数"""
        # 等比缩放适配当前标签大小
        label_size = self._img_label.size()
        if not label_size.isEmpty() and not pixmap.isNull():
            scaled = pixmap.scaled(
                label_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        else:
            scaled = pixmap
        self._img_label.setPixmap(scaled)

        # 更新帧信息
        self._frame_label.setText(f'帧 {idx + 1} / {total}')

        # 更新滑块范围 (blockSignals 防止触发 seek_requested)
        slider_max = max(0, total - 1)
        self._slider.blockSignals(True)
        self._slider.setMaximum(slider_max)
        self._slider.setValue(idx)
        self._slider.blockSignals(False)

    def set_play_text(self, playing: bool):
        """更新播放按钮文本"""
        self._btn_play.setText('暂停' if playing else '播放')

    # ─── 私有槽函数 ────────────────────────────────────

    def _on_close(self):
        """关闭按钮：隐藏窗口并发射 closed 信号"""
        self.hide()
        self.closed.emit()

    def _on_speed_changed(self, index: int):
        """速度下拉框改变 → 发射 speed_changed 信号"""
        value = self._speed_combo.itemData(index)
        if value is not None:
            self.speed_changed.emit(float(value))
