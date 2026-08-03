# [DONE] F-8.2 左侧边栏：数据集信息、统计卡片、工具选择、播放控制
# [DONE] F-9.1 处理数据按钮 / F-9.11 auto_node 占位
# [DONE] OP-1 调试/操作模式切换按钮 + 操作模式 UI 锁定

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QProgressBar,
    QButtonGroup, QFrame, QHBoxLayout, QTextEdit, QComboBox,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont, QPainter, QColor


# ─── 模式切换按钮控件 ────────────────────────────────────────

from core.ui_font import mono_font  # [ADAPT-UBU-02] 跨平台等宽字体
class ModeSwitchButton(QWidget):
    """
    调试模式 | 操作模式 两段式切换按钮。
    点击左半侧 → 发射 mode_changed('debug')
    点击右半侧 → 发射 mode_changed('op')
    """
    mode_changed = pyqtSignal(str)  # 'debug' | 'op'

    _BG_ACTIVE_DEBUG = '#555555'
    _BG_ACTIVE_OP    = '#1d9e75'
    _BG_INACTIVE     = '#f4f2ed'
    _FG_ACTIVE       = '#ffffff'
    _FG_INACTIVE     = '#888780'
    _BORDER          = '#d3d1c7'

    def __init__(self, parent=None):
        super().__init__(parent)
        self._mode = 'debug'  # 当前模式
        self.setFixedHeight(28)
        font = mono_font(8)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._btn_debug = QPushButton('调试模式')
        self._btn_debug.setFont(font)
        self._btn_debug.setFixedHeight(28)
        self._btn_debug.setCursor(Qt.PointingHandCursor)
        self._btn_debug.clicked.connect(lambda: self._switch('debug'))

        self._btn_op = QPushButton('操作模式')
        self._btn_op.setFont(font)
        self._btn_op.setFixedHeight(28)
        self._btn_op.setCursor(Qt.PointingHandCursor)
        self._btn_op.clicked.connect(lambda: self._switch('op'))

        layout.addWidget(self._btn_debug)
        layout.addWidget(self._btn_op)
        self._refresh_style()

    def _switch(self, mode: str):
        if self._mode == mode:
            return
        self._mode = mode
        self._refresh_style()
        self.mode_changed.emit(mode)

    def _refresh_style(self):
        # 左侧（调试模式）
        if self._mode == 'debug':
            db_bg, db_fg = self._BG_ACTIVE_DEBUG, self._FG_ACTIVE
            op_bg, op_fg = self._BG_INACTIVE,     self._FG_INACTIVE
        else:
            db_bg, db_fg = self._BG_INACTIVE,     self._FG_INACTIVE
            op_bg, op_fg = self._BG_ACTIVE_OP,    self._FG_ACTIVE

        self._btn_debug.setStyleSheet(
            f'QPushButton {{ background: {db_bg}; color: {db_fg}; '
            f'border: 0.5px solid {self._BORDER}; '
            f'border-right: none; '
            f'border-top-left-radius: 3px; border-bottom-left-radius: 3px; '
            f'padding: 4px 6px; }}'
        )
        self._btn_op.setStyleSheet(
            f'QPushButton {{ background: {op_bg}; color: {op_fg}; '
            f'border: 0.5px solid {self._BORDER}; '
            f'border-top-right-radius: 3px; border-bottom-right-radius: 3px; '
            f'padding: 4px 6px; }}'
        )

    @property
    def current_mode(self) -> str:
        return self._mode


# ─── 主边栏 ──────────────────────────────────────────────────

class Sidebar(QWidget):
    """左侧边栏 (220px)"""

    import_requested = pyqtSignal()
    play_toggled = pyqtSignal()
    reset_requested = pyqtSignal()
    tool_changed = pyqtSignal(str)  # 'pan' | 'node' | 'edge' | 'plan' | 'delete_edge'
    process_data_requested = pyqtSignal()  # F-9.1 处理数据按钮
    auto_node_requested = pyqtSignal()  # F-9.11 auto_node 占位按钮
    mode_changed = pyqtSignal(str)      # OP-1 'debug' | 'op'
    authority_mode_changed = pyqtSignal(str)
    mission_publish_requested = pyqtSignal()
    mission_start_requested = pyqtSignal()
    mission_cancel_requested = pyqtSignal()
    mission_pause_requested = pyqtSignal()
    mission_resume_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(220)
        self.setStyleSheet(
            'background: #ffffff; border-right: 0.5px solid #d3d1c7;'
        )
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        font_main = mono_font(9)
        font_small = mono_font(8)
        font_title = mono_font(10)
        font_title.setBold(True)

        # ─── OP-1 模式切换按钮（替换原"数据集"标签） ───
        self._mode_btn = ModeSwitchButton()
        self._mode_btn.mode_changed.connect(self.mode_changed.emit)
        layout.addWidget(self._mode_btn)

        # ─── 导入按钮 ───
        self._btn_import = QPushButton('导入 .db 文件')
        self._btn_import.setFont(font_main)
        self._btn_import.setStyleSheet(_btn_style('#1d9e75', '#ffffff'))
        self._btn_import.clicked.connect(self.import_requested.emit)
        layout.addWidget(self._btn_import)

        # ─── 进度条 ───
        self.progress = QProgressBar()
        # 记录导入按钮引用（用于操作模式禁用）
        self.progress.setFont(font_small)
        self.progress.setStyleSheet(
            'QProgressBar { border: 0.5px solid #d3d1c7; border-radius: 3px; '
            'background: #f4f2ed; height: 14px; }'
            'QProgressBar::chunk { background: #1d9e75; border-radius: 2px; }'
        )
        self.progress.setVisible(False)
        layout.addWidget(self.progress)

        # ─── 分割线 ───
        layout.addWidget(_separator())

        # ─── 统计卡片 ───
        stats_title = QLabel('统计')
        stats_title.setFont(font_title)
        stats_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(stats_title)

        self.lbl_nodes = QLabel('节点: --')
        self.lbl_nodes.setFont(font_small)
        self.lbl_nodes.setStyleSheet('color: #888780;')
        layout.addWidget(self.lbl_nodes)

        self.lbl_edges = QLabel('边: --')
        self.lbl_edges.setFont(font_small)
        self.lbl_edges.setStyleSheet('color: #888780;')
        layout.addWidget(self.lbl_edges)

        self.lbl_frame = QLabel('帧: -- / --')
        self.lbl_frame.setFont(font_small)
        self.lbl_frame.setStyleSheet('color: #888780;')
        layout.addWidget(self.lbl_frame)

        # ─── 分割线 ───
        layout.addWidget(_separator())

        # ─── 工具选择 ───
        tools_title = QLabel('工具')
        tools_title.setFont(font_title)
        tools_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(tools_title)

        self._tool_group = QButtonGroup(self)
        self._tool_group.setExclusive(True)
        self._tool_btns: dict[str, QPushButton] = {}  # tool_id → 按钮引用

        tools = [
            ('pan', '平移 (默认)'),
            ('node', '节点模式'),
            ('edge', '连边模式'),
            ('plan', '规划模式'),
            ('delete_edge', '删边模式'),
            ('delete_node', '删节点模式'),
        ]
        for tool_id, tool_label in tools:
            btn = QPushButton(tool_label)
            btn.setFont(font_small)
            btn.setCheckable(True)
            btn.setStyleSheet(_tool_btn_style())
            btn.clicked.connect(
                lambda checked, t=tool_id: self.tool_changed.emit(t)
            )
            self._tool_group.addButton(btn, tools.index((tool_id, tool_label)))
            self._tool_btns[tool_id] = btn
            layout.addWidget(btn)

        # 默认选中平移
        self._tool_group.buttons()[0].setChecked(True)

        # ─── 分割线 ───
        layout.addWidget(_separator())

        # ─── 播放控制 ───
        play_title = QLabel('轨迹播放')
        play_title.setFont(font_title)
        play_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(play_title)

        self.btn_play = QPushButton('Z - 播放')
        self.btn_play.setFont(font_main)
        self.btn_play.setStyleSheet(_btn_style('#1d9e75', '#ffffff'))
        self.btn_play.clicked.connect(self.play_toggled.emit)
        layout.addWidget(self.btn_play)

        self._btn_lock = QPushButton('Space - 锁定节点')
        self._btn_lock.setFont(font_main)
        self._btn_lock.setStyleSheet(_btn_style('#ba7517', '#ffffff'))
        layout.addWidget(self._btn_lock)
        # 锁定信号由 main_window 直接绑定快捷键，此处仅作视觉提示

        self._btn_reset = QPushButton('复原 - 清空全部')
        self._btn_reset.setFont(font_main)
        self._btn_reset.setStyleSheet(_btn_style('#e24b4a', '#ffffff'))
        self._btn_reset.clicked.connect(self.reset_requested.emit)
        layout.addWidget(self._btn_reset)

        # ─── F-9.1 处理数据按钮 ───
        self._btn_process = QPushButton('处理数据')
        self._btn_process.setFont(font_main)
        self._btn_process.setStyleSheet(_btn_style('#185fa5', '#ffffff'))
        self._btn_process.clicked.connect(self.process_data_requested.emit)
        layout.addWidget(self._btn_process)

        # ─── F-9.11 auto_node 占位按钮（灰色表未启用） ───
        self._btn_auto_node = QPushButton('auto_node')
        self._btn_auto_node.setFont(font_main)
        self._btn_auto_node.setStyleSheet(_btn_style('#888780', '#ffffff'))
        self._btn_auto_node.clicked.connect(self.auto_node_requested.emit)
        layout.addWidget(self._btn_auto_node)

        # ─── 分割线 ───
        layout.addWidget(_separator())

        mission_title = QLabel('Mission')
        mission_title.setFont(font_title)
        mission_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(mission_title)

        self._authority_mode = QComboBox()
        self._authority_mode.setFont(font_small)
        self._authority_mode.addItem('legacy', 'legacy')
        self._authority_mode.addItem('mission_nav2', 'mission_nav2')
        self._authority_mode.currentIndexChanged.connect(
            lambda _idx: self.authority_mode_changed.emit(self.current_authority_mode())
        )
        layout.addWidget(self._authority_mode)

        self._mission_status = QLabel('mission: --')
        self._mission_status.setFont(mono_font(7))
        self._mission_status.setWordWrap(True)
        self._mission_status.setStyleSheet('color: #2c2c2a; background: #faf9f6; padding: 4px;')
        layout.addWidget(self._mission_status)

        self._btn_mission_publish = QPushButton('Publish Mission')
        self._btn_mission_start = QPushButton('Start Mission')
        self._btn_mission_cancel = QPushButton('Cancel')
        self._btn_mission_pause = QPushButton('Pause')
        self._btn_mission_resume = QPushButton('Resume')
        for btn in (
            self._btn_mission_publish,
            self._btn_mission_start,
            self._btn_mission_cancel,
            self._btn_mission_pause,
            self._btn_mission_resume,
        ):
            btn.setFont(font_small)
            btn.setStyleSheet(_btn_style('#185fa5', '#ffffff'))
            layout.addWidget(btn)
        self._btn_mission_publish.clicked.connect(self.mission_publish_requested.emit)
        self._btn_mission_start.clicked.connect(self.mission_start_requested.emit)
        self._btn_mission_cancel.clicked.connect(self.mission_cancel_requested.emit)
        self._btn_mission_pause.clicked.connect(self.mission_pause_requested.emit)
        self._btn_mission_resume.clicked.connect(self.mission_resume_requested.emit)
        self.set_mission_controls(False, False)

        layout.addWidget(_separator())

        # ─── UDP 发送预览 ───
        udp_title = QLabel('UDP 发送预览')
        udp_title.setFont(font_title)
        udp_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(udp_title)

        self.udp_preview = QTextEdit()
        self.udp_preview.setReadOnly(True)
        self.udp_preview.setFont(mono_font(7))
        self.udp_preview.setStyleSheet(
            'QTextEdit { background: #faf9f6; color: #2c2c2a; '
            'border: 0.5px solid #d3d1c7; border-radius: 3px; '
            'padding: 4px; }'
        )
        self.udp_preview.setMinimumHeight(80)
        layout.addWidget(self.udp_preview, stretch=1)

    def update_stats(self, node_count: int = None, edge_count: int = None):
        if node_count is not None:
            self.lbl_nodes.setText(f'节点: {node_count}')
        if edge_count is not None:
            self.lbl_edges.setText(f'边: {edge_count}')

    def update_frame(self, current: int, total: int):
        self.lbl_frame.setText(f'帧: {current} / {total}')

    def set_operation_mode(self, op: bool):
        """切换边栏的调试/操作模式：操作模式下锁定除规划外的所有交互控件"""
        if op:
            # ── 操作模式：禁用导入与播放相关按钮 ──
            self._btn_import.setEnabled(False)
            self._btn_lock.setEnabled(False)
            self._btn_reset.setEnabled(False)
            self._btn_process.setEnabled(False)
            self._btn_auto_node.setEnabled(False)
            self.btn_play.setEnabled(False)

            # ── 操作模式：禁用除"规划模式"外的所有工具按钮 ──
            for tool_id, btn in self._tool_btns.items():
                if tool_id != 'plan':
                    btn.setEnabled(False)

            # ── 操作模式：强制选中"规划模式"按钮并发射信号 ──
            plan_btn = self._tool_btns.get('plan')
            if plan_btn is not None:
                plan_btn.setChecked(True)
            self.tool_changed.emit('plan')

            # ── 操作模式：放大 UDP 预览字体 ──
            self.udp_preview.setFont(mono_font(11))
        else:
            # ── 调试模式：恢复所有按钮 ──
            self._btn_import.setEnabled(True)
            self._btn_lock.setEnabled(True)
            self._btn_reset.setEnabled(True)
            self._btn_process.setEnabled(True)
            self._btn_auto_node.setEnabled(True)
            self.btn_play.setEnabled(True)

            for btn in self._tool_btns.values():
                btn.setEnabled(True)

            # ── 调试模式：恢复 UDP 预览字体 ──
            self.udp_preview.setFont(mono_font(7))

    def set_progress(self, value: int):
        self.progress.setVisible(True)
        self.progress.setValue(value)
        if value >= 100:
            self.progress.setVisible(False)

    def current_authority_mode(self) -> str:
        return str(self._authority_mode.currentData() or 'legacy')

    def set_mission_controls(self, route_ready: bool, start_ready: bool):
        mission_mode = self.current_authority_mode() == 'mission_nav2'
        self._btn_mission_publish.setEnabled(mission_mode and route_ready)
        self._btn_mission_start.setEnabled(mission_mode and start_ready)
        self._btn_mission_cancel.setEnabled(mission_mode)
        self._btn_mission_pause.setEnabled(mission_mode)
        self._btn_mission_resume.setEnabled(mission_mode)

    def set_mission_status(self, text: str):
        self._mission_status.setText(text)

    def set_play_button_text(self, playing: bool):
        self.btn_play.setText('Z - 暂停' if playing else 'Z - 播放')

    def clear_udp_preview(self):
        """清空 UDP 预览面板"""
        self.udp_preview.clear()

    def append_udp_info(self, text: str, color: str = '#2c2c2a'):
        """追加一行 UDP 预览信息"""
        html = f'<span style="color:{color};">{text}</span>'
        self.udp_preview.append(html)
        # 自动滚动到底部
        from PyQt5.QtGui import QTextCursor
        cursor = self.udp_preview.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.udp_preview.setTextCursor(cursor)


# ─── 样式辅助 ───────────────────────────────────────

def _btn_style(bg: str, fg: str) -> str:
    return (
        f'QPushButton {{ background: {bg}; color: {fg}; '
        f'border: none; border-radius: 3px; padding: 6px 10px; }}'
        f'QPushButton:hover {{ opacity: 0.85; }}'
    )


def _tool_btn_style() -> str:
    return (
        'QPushButton { background: #f4f2ed; color: #2c2c2a; '
        'border: 0.5px solid #d3d1c7; border-radius: 3px; '
        'padding: 4px 8px; text-align: left; }'
        'QPushButton:checked { background: #1d9e75; color: #ffffff; '
        'border-color: #1d9e75; }'
    )


def _separator() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet('color: #d3d1c7;')
    return line
