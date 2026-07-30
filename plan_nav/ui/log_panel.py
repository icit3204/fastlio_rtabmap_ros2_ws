# [DONE] F-8.1 右侧日志面板：实时滚动事件日志
# [DONE] F-8.2 UDP 配置编辑 (IP/端口)
# [DONE] F-10.1 识别参数 UI 接入
# [DONE] F-11.7 轨迹控制参数 UI 区块

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QTextEdit, QLabel, QLineEdit,
    QPushButton, QFrame, QHBoxLayout,
)
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFont, QTextCursor
from datetime import datetime


from core.ui_font import mono_font  # [ADAPT-UBU-02] 跨平台等宽字体
class LogPanel(QWidget):
    """右侧面板 (200px)：实时日志 + UDP 配置"""

    lock_requested = pyqtSignal()
    config_changed = pyqtSignal(str, int, float, float)  # ip, port, yes_conf, axis_tolerance
    traj_params_changed = pyqtSignal(float, float, float, float, float)
    # 顺序：r_min, r_max, delta_thresh(°), v_straight, v_turn

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(200)
        self.setStyleSheet(
            'background: #ffffff; border-left: 0.5px solid #d3d1c7;'
        )
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 12)
        layout.setSpacing(8)

        font_title = mono_font(10)
        font_title.setBold(True)
        font_mono = mono_font(8)

        # ─── 日志标题行（标题 + 状态指示灯）───
        title_row = QHBoxLayout()
        title = QLabel('日志')
        title.setFont(font_title)
        title.setStyleSheet('color: #2c2c2a;')
        title_row.addWidget(title)
        title_row.addStretch()

        self._status_dot = QLabel()
        self._status_dot.setFixedSize(10, 10)
        self._status_dot.setStyleSheet(
            'border-radius: 5px; background: #e24b4a;'
        )
        self._status_dot.setVisible(False)  # 调试模式下隐藏
        title_row.addWidget(self._status_dot)
        layout.addLayout(title_row)

        # ─── 日志输出区 ───
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(mono_font(7))
        self.log_view.setStyleSheet(
            'QTextEdit { background: #faf9f6; color: #2c2c2a; '
            'border: 0.5px solid #d3d1c7; border-radius: 3px; '
            'padding: 4px; }'
        )
        layout.addWidget(self.log_view, stretch=1)

        # ─── 锁定按钮 ───
        self._btn_lock = QPushButton('Space - 锁定当前节点')
        self._btn_lock.setFont(mono_font(8))
        self._btn_lock.setStyleSheet(
            'QPushButton { background: #ba7517; color: #ffffff; '
            'border: none; border-radius: 3px; padding: 5px 8px; }'
            'QPushButton:hover { opacity: 0.85; }'
        )
        self._btn_lock.clicked.connect(self.lock_requested.emit)
        layout.addWidget(self._btn_lock)

        # ─── 分割线 ───
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet('color: #d3d1c7;')
        layout.addWidget(line)

        # ─── 识别参数 ───
        detect_title = QLabel('识别参数')
        detect_title.setFont(font_title)
        detect_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(detect_title)

        yes_layout = QHBoxLayout()
        yes_label = QLabel('yes 置信度')
        yes_label.setFont(font_mono)
        yes_label.setStyleSheet('color: #888780;')
        yes_layout.addWidget(yes_label)
        self.yes_conf_input = QLineEdit('0.65')
        self.yes_conf_input.setFont(font_mono)
        self.yes_conf_input.setStyleSheet(
            'QLineEdit { border: 0.5px solid #d3d1c7; border-radius: 3px; '
            'padding: 2px 4px; color: #2c2c2a; background: #faf9f6; }'
        )
        yes_layout.addWidget(self.yes_conf_input)
        layout.addLayout(yes_layout)

        axis_layout = QHBoxLayout()
        axis_label = QLabel('中心偏差')
        axis_label.setFont(font_mono)
        axis_label.setStyleSheet('color: #888780;')
        axis_layout.addWidget(axis_label)
        self.axis_tol_input = QLineEdit('0.2')
        self.axis_tol_input.setFont(font_mono)
        self.axis_tol_input.setStyleSheet(
            'QLineEdit { border: 0.5px solid #d3d1c7; border-radius: 3px; '
            'padding: 2px 4px; color: #2c2c2a; background: #faf9f6; }'
        )
        axis_layout.addWidget(self.axis_tol_input)
        layout.addLayout(axis_layout)

        # ─── 分割线 ───
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet('color: #d3d1c7;')
        layout.addWidget(line2)

        # ─── UDP 配置 ───
        udp_title = QLabel('UDP 配置')
        udp_title.setFont(font_title)
        udp_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(udp_title)

        ip_layout = QHBoxLayout()
        ip_label = QLabel('IP')
        ip_label.setFont(font_mono)
        ip_label.setStyleSheet('color: #888780;')
        ip_layout.addWidget(ip_label)
        self.ip_input = QLineEdit('127.0.0.1')
        self.ip_input.setFont(font_mono)
        self.ip_input.setStyleSheet(
            'QLineEdit { border: 0.5px solid #d3d1c7; border-radius: 3px; '
            'padding: 2px 4px; color: #2c2c2a; background: #faf9f6; }'
        )
        ip_layout.addWidget(self.ip_input)
        layout.addLayout(ip_layout)

        port_layout = QHBoxLayout()
        port_label = QLabel('端口')
        port_label.setFont(font_mono)
        port_label.setStyleSheet('color: #888780;')
        port_layout.addWidget(port_label)
        self.port_input = QLineEdit('14550')
        self.port_input.setFont(font_mono)
        self.port_input.setStyleSheet(
            'QLineEdit { border: 0.5px solid #d3d1c7; border-radius: 3px; '
            'padding: 2px 4px; color: #2c2c2a; background: #faf9f6; }'
        )
        port_layout.addWidget(self.port_input)
        layout.addLayout(port_layout)

        self._btn_apply_config = QPushButton('应用配置')
        self._btn_apply_config.setFont(font_mono)
        self._btn_apply_config.setStyleSheet(
            'QPushButton { background: #1d9e75; color: #ffffff; '
            'border: none; border-radius: 3px; padding: 4px 8px; }'
            'QPushButton:hover { opacity: 0.85; }'
        )
        self._btn_apply_config.clicked.connect(self._apply_config)
        layout.addWidget(self._btn_apply_config)

        # ─── 分割线 ───
        line_traj = QFrame()
        line_traj.setFrameShape(QFrame.HLine)
        line_traj.setStyleSheet('color: #d3d1c7;')
        layout.addWidget(line_traj)

        # ─── 轨迹控制 ───
        traj_title = QLabel('轨迹控制')
        traj_title.setFont(font_title)
        traj_title.setStyleSheet('color: #185fa5;')
        layout.addWidget(traj_title)

        params = [
            ('r_min (m)',  'pursuit_r_min_input',  '0.50'),
            ('r_max (m)',  'pursuit_r_max_input',  '1.50'),
            ('δ阈值 (°)',  'pursuit_dt_input',     '30'),
            ('v直行',      'pursuit_vs_input',     '5000'),
            ('v转弯',      'pursuit_vt_input',     '3000'),
        ]
        for label_text, attr, default in params:
            row = QHBoxLayout()
            lbl = QLabel(label_text)
            lbl.setFont(font_mono)
            lbl.setStyleSheet('color: #888780;')
            row.addWidget(lbl)
            inp = QLineEdit(default)
            inp.setFont(font_mono)
            inp.setStyleSheet(
                'QLineEdit { border: 0.5px solid #d3d1c7; border-radius: 3px; '
                'padding: 2px 4px; color: #2c2c2a; background: #faf9f6; }'
            )
            row.addWidget(inp)
            setattr(self, attr, inp)
            layout.addLayout(row)

        # 应用按钮
        self._btn_apply_traj = QPushButton('应用轨迹参数')
        self._btn_apply_traj.setFont(font_mono)
        self._btn_apply_traj.setStyleSheet(
            'QPushButton { background: #185fa5; color: #fff; border-radius: 4px; padding: 4px 8px; }'
            'QPushButton:hover { background: #1470c0; }'
        )
        self._btn_apply_traj.clicked.connect(self._apply_traj_params)
        layout.addWidget(self._btn_apply_traj)

    def _apply_config(self):
        """读取 UDP 配置和识别参数，发射 config_changed 信号"""
        try:
            ip = self.ip_input.text().strip()
            port = int(self.port_input.text())
            yes_conf = float(self.yes_conf_input.text())
            axis_tol = float(self.axis_tol_input.text())
            self.config_changed.emit(ip, port, yes_conf, axis_tol)
        except ValueError:
            pass

    def _apply_traj_params(self):
        try:
            r_min = float(self.pursuit_r_min_input.text())
            r_max = float(self.pursuit_r_max_input.text())
            dt = float(self.pursuit_dt_input.text())
            vs = float(self.pursuit_vs_input.text())
            vt = float(self.pursuit_vt_input.text())
            self.traj_params_changed.emit(r_min, r_max, dt, vs, vt)
        except ValueError:
            pass

    def set_detector_defaults(self, yes_conf, axis_tolerance):
        self.yes_conf_input.setText(str(yes_conf))
        self.axis_tol_input.setText(str(axis_tolerance))

    def set_traj_defaults(self, r_min, r_max, dt, vs, vt):
        self.pursuit_r_min_input.setText(str(r_min))
        self.pursuit_r_max_input.setText(str(r_max))
        self.pursuit_dt_input.setText(str(dt))
        self.pursuit_vs_input.setText(str(vs))
        self.pursuit_vt_input.setText(str(vt))

    def append(self, msg: str, level: str = ''):
        """向日志区追加一条带时间戳和颜色的记录"""
        color_map = {
            'error': '#e24b4a',
            'warn':  '#ba7517',
            'info':  '#1d9e75',
            '':      '#2c2c2a',
        }
        color = color_map.get(level, '#2c2c2a')
        ts = datetime.now().strftime('%H:%M:%S')
        html = f'<span style="color:#b4b2a9;">[{ts}]</span> <span style="color:{color};">{msg}</span>'
        self.log_view.append(html)
        # 自动滚动到底
        cursor = self.log_view.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_view.setTextCursor(cursor)

    def set_operation_mode(self, op: bool):
        """切换右侧面板的调试/操作模式：锁定参数、禁用按钮、隐藏锁定按钮、放大字体"""
        inputs = [
            self.yes_conf_input, self.axis_tol_input,
            self.ip_input, self.port_input,
            self.pursuit_r_min_input, self.pursuit_r_max_input,
            self.pursuit_dt_input, self.pursuit_vs_input, self.pursuit_vt_input,
        ]
        for inp in inputs:
            inp.setReadOnly(op)
            inp.setStyleSheet(
                'QLineEdit { border: 0.5px solid #d3d1c7; border-radius: 3px; '
                'padding: 2px 4px; color: #2c2c2a; '
                f'background: {"#e8e6e1" if op else "#faf9f6"}; }}'
            )

        self._btn_apply_config.setEnabled(not op)
        self._btn_apply_traj.setEnabled(not op)
        self._btn_lock.setVisible(not op)

        font_size = 11 if op else 7
        self.log_view.setFont(mono_font(font_size))

        self._status_dot.setVisible(op)

    def set_pose_status(self, receiving: bool):
        """主线程调用：更新状态指示灯颜色，True=绿色，False=红色"""
        color = '#2ecc71' if receiving else '#e24b4a'
        self._status_dot.setStyleSheet(
            f'border-radius: 5px; background: {color};'
        )
