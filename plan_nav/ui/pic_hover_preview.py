# [IMPL] F-14.1 节点悬停图片预览 — 鼠标悬停语义节点时显示对应图片
# [IMPL] F-14.3 固定尺寸(220×180) + 轻量无控件 — 仅图片+帧号，无按钮无滑块
# [IMPL] F-14.5 不抢焦点(Qt.ToolTip) — 自动隐藏，不影响地图交互

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from core.ui_font import mono_font  # [ADAPT-UBU-02] 跨平台等宽字体
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QFont


class HoverPreview(QWidget):
    """悬停预览浮窗：鼠标悬停在节点上时，显示该节点对应的图片帧。
    极简设计 — 无标题栏、无按钮、无滑块、不可拖拽、不可缩放。"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # 固定尺寸 440×360，不可缩放
        self.setFixedSize(440, 360)

        # Qt.ToolTip 标志：无需标题栏，失去焦点自动隐藏
        self.setWindowFlags(Qt.ToolTip)

        # 不抢夺键盘焦点
        self.setAttribute(Qt.WA_ShowWithoutActivating)

        # 外观：深色边框 + 黑色背景
        self.setStyleSheet(
            'HoverPreview {'
            '  border: 1px solid #555555;'
            '  background: #000000;'
            '}'
        )

        self._setup_ui()
        # 初始隐藏
        self.hide()

    def _setup_ui(self):
        """构建极简布局：图片 + 帧号"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 图片标签
        self._img_label = QLabel()
        self._img_label.setAlignment(Qt.AlignCenter)
        self._img_label.setStyleSheet(
            'QLabel {'
            '  background: #000000; color: #555555;'
            '  border: none;'
            '}'
        )
        self._img_label.setFont(mono_font(8))
        layout.addWidget(self._img_label, stretch=1)

        # 帧号标签
        self._info_label = QLabel()
        self._info_label.setAlignment(Qt.AlignCenter)
        self._info_label.setFont(mono_font(7))
        self._info_label.setStyleSheet(
            'QLabel {'
            '  color: #888780; background: #000000; border: none;'
            '  padding: 1px 0px;'
            '}'
        )
        layout.addWidget(self._info_label)

    def set_frame(self, pixmap: QPixmap, idx: int, total: int):
        """设置当前帧的图片和索引信息。
        图片等比缩放至 420×300 固定显示区域。"""
        if not pixmap.isNull():
            scaled = pixmap.scaled(
                420, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self._img_label.setPixmap(scaled)
        else:
            self._img_label.clear()

        self._info_label.setText(f'帧 {idx + 1} / {total}')
