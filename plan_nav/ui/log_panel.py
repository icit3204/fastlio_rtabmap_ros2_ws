"""PlanNav authoring log and recognition settings panel."""

from datetime import datetime

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit, QVBoxLayout, QWidget

from core.ui_font import mono_font


class LogPanel(QWidget):
    """Right panel without legacy runtime transport or pursuit controls."""

    lock_requested = pyqtSignal()
    detector_config_changed = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(200)
        self.setStyleSheet('background: #ffffff; border-left: 0.5px solid #d3d1c7;')
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 12)
        layout.setSpacing(8)
        font_title = mono_font(10)
        font_title.setBold(True)
        font_mono = mono_font(8)

        title = QLabel('日志')
        title.setFont(font_title)
        title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(title)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(mono_font(7))
        self.log_view.setStyleSheet(
            'QTextEdit { background: #faf9f6; color: #2c2c2a; border: 0.5px solid #d3d1c7; '
            'border-radius: 3px; padding: 4px; }'
        )
        layout.addWidget(self.log_view, stretch=1)

        self._btn_lock = QPushButton('Space - 锁定当前节点')
        self._btn_lock.setFont(font_mono)
        self._btn_lock.setStyleSheet(
            'QPushButton { background: #ba7517; color: #ffffff; border: none; border-radius: 3px; padding: 5px 8px; }'
        )
        self._btn_lock.clicked.connect(self.lock_requested.emit)
        layout.addWidget(self._btn_lock)
        layout.addWidget(_separator())

        detect_title = QLabel('识别参数')
        detect_title.setFont(font_title)
        detect_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(detect_title)
        self.yes_conf_input = self._add_input(layout, 'yes 置信度', '0.65', font_mono)
        self.axis_tol_input = self._add_input(layout, '中心偏差', '0.2', font_mono)
        self._btn_apply_config = QPushButton('应用识别参数')
        self._btn_apply_config.setFont(font_mono)
        self._btn_apply_config.setStyleSheet(
            'QPushButton { background: #1d9e75; color: #ffffff; border: none; border-radius: 3px; padding: 4px 8px; }'
        )
        self._btn_apply_config.clicked.connect(self._apply_config)
        layout.addWidget(self._btn_apply_config)

    @staticmethod
    def _add_input(layout, label_text, default, font):
        row = QHBoxLayout()
        label = QLabel(label_text)
        label.setFont(font)
        label.setStyleSheet('color: #888780;')
        row.addWidget(label)
        value = QLineEdit(default)
        value.setFont(font)
        value.setStyleSheet(
            'QLineEdit { border: 0.5px solid #d3d1c7; border-radius: 3px; padding: 2px 4px; '
            'color: #2c2c2a; background: #faf9f6; }'
        )
        row.addWidget(value)
        layout.addLayout(row)
        return value

    def _apply_config(self):
        try:
            self.detector_config_changed.emit(float(self.yes_conf_input.text()), float(self.axis_tol_input.text()))
        except ValueError:
            self.append('识别参数格式无效', 'error')

    def set_detector_defaults(self, yes_conf, axis_tolerance):
        self.yes_conf_input.setText(str(yes_conf))
        self.axis_tol_input.setText(str(axis_tolerance))

    def append(self, msg: str, level: str = ''):
        color = {'error': '#e24b4a', 'warn': '#ba7517', 'info': '#1d9e75', '': '#2c2c2a'}.get(level, '#2c2c2a')
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_view.append(f'<span style="color:#b4b2a9;">[{ts}]</span> <span style="color:{color};">{msg}</span>')
        cursor = self.log_view.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_view.setTextCursor(cursor)


def _separator():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet('color: #d3d1c7;')
    return line
