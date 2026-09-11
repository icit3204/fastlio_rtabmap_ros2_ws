"""PlanNav topology and route-authoring sidebar."""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QButtonGroup, QFrame, QLabel, QProgressBar, QPushButton, QVBoxLayout, QWidget

from core.ui_font import mono_font


class Sidebar(QWidget):
    """Authoring-only sidebar; runtime mission controls live in Operator GUI."""

    import_requested = pyqtSignal()
    optimized_import_requested = pyqtSignal()
    play_toggled = pyqtSignal()
    reset_requested = pyqtSignal()
    tool_changed = pyqtSignal(str)
    process_data_requested = pyqtSignal()
    auto_node_requested = pyqtSignal()
    mission_publish_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(220)
        self.setStyleSheet('background: #ffffff; border-right: 0.5px solid #d3d1c7;')
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        font_main = mono_font(9)
        font_small = mono_font(8)
        font_title = mono_font(10)
        font_title.setBold(True)

        self._btn_import = QPushButton('导入 .db 文件')
        self._btn_import.setFont(font_main)
        self._btn_import.setStyleSheet(_btn_style('#1d9e75', '#ffffff'))
        self._btn_import.clicked.connect(self.import_requested.emit)
        layout.addWidget(self._btn_import)

        self._btn_import_optimized = QPushButton('导入优化 map DB')
        self._btn_import_optimized.setFont(font_main)
        self._btn_import_optimized.setStyleSheet(_btn_style('#185fa5', '#ffffff'))
        self._btn_import_optimized.clicked.connect(self.optimized_import_requested.emit)
        layout.addWidget(self._btn_import_optimized)

        self.progress = QProgressBar()
        self.progress.setFont(font_small)
        self.progress.setStyleSheet(
            'QProgressBar { border: 0.5px solid #d3d1c7; border-radius: 3px; background: #f4f2ed; height: 14px; }'
            'QProgressBar::chunk { background: #1d9e75; border-radius: 2px; }'
        )
        self.progress.setVisible(False)
        layout.addWidget(self.progress)
        layout.addWidget(_separator())

        stats_title = QLabel('统计')
        stats_title.setFont(font_title)
        stats_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(stats_title)
        self.lbl_nodes = QLabel('节点: --')
        self.lbl_edges = QLabel('边: --')
        self.lbl_frame = QLabel('帧: -- / --')
        for label in (self.lbl_nodes, self.lbl_edges, self.lbl_frame):
            label.setFont(font_small)
            label.setStyleSheet('color: #888780;')
            layout.addWidget(label)
        layout.addWidget(_separator())

        tools_title = QLabel('工具')
        tools_title.setFont(font_title)
        tools_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(tools_title)
        self._tool_group = QButtonGroup(self)
        self._tool_group.setExclusive(True)
        self._tool_btns = {}
        tools = [
            ('pan', '平移 (默认)'), ('node', '节点模式'), ('edge', '连边模式'),
            ('plan', '规划模式'), ('delete_edge', '删边模式'), ('delete_node', '删节点模式'),
        ]
        for index, (tool_id, tool_label) in enumerate(tools):
            button = QPushButton(tool_label)
            button.setFont(font_small)
            button.setCheckable(True)
            button.setStyleSheet(_tool_btn_style())
            button.clicked.connect(lambda _checked, item=tool_id: self.tool_changed.emit(item))
            self._tool_group.addButton(button, index)
            self._tool_btns[tool_id] = button
            layout.addWidget(button)
        self._tool_group.buttons()[0].setChecked(True)
        layout.addWidget(_separator())

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
        self._btn_reset = QPushButton('复原 - 清空全部')
        self._btn_reset.setFont(font_main)
        self._btn_reset.setStyleSheet(_btn_style('#e24b4a', '#ffffff'))
        self._btn_reset.clicked.connect(self.reset_requested.emit)
        layout.addWidget(self._btn_reset)
        self._btn_process = QPushButton('处理数据')
        self._btn_process.setFont(font_main)
        self._btn_process.setStyleSheet(_btn_style('#185fa5', '#ffffff'))
        self._btn_process.clicked.connect(self.process_data_requested.emit)
        layout.addWidget(self._btn_process)
        self._btn_auto_node = QPushButton('auto_node')
        self._btn_auto_node.setFont(font_main)
        self._btn_auto_node.setStyleSheet(_btn_style('#888780', '#ffffff'))
        self._btn_auto_node.clicked.connect(self.auto_node_requested.emit)
        layout.addWidget(self._btn_auto_node)
        layout.addWidget(_separator())

        route_title = QLabel('RouteMission Intent')
        route_title.setFont(font_title)
        route_title.setStyleSheet('color: #2c2c2a;')
        layout.addWidget(route_title)
        self._mission_status = QLabel('route: select Start and Goal')
        self._mission_status.setFont(mono_font(7))
        self._mission_status.setWordWrap(True)
        self._mission_status.setStyleSheet('color: #2c2c2a; background: #faf9f6; padding: 4px;')
        layout.addWidget(self._mission_status)
        self._btn_mission_publish = QPushButton('Publish RouteMission')
        self._btn_mission_publish.setFont(font_small)
        self._btn_mission_publish.setStyleSheet(_btn_style('#185fa5', '#ffffff'))
        self._btn_mission_publish.clicked.connect(self.mission_publish_requested.emit)
        self._btn_mission_publish.setEnabled(False)
        layout.addWidget(self._btn_mission_publish)
        layout.addStretch(1)

    def update_stats(self, node_count=None, edge_count=None):
        if node_count is not None:
            self.lbl_nodes.setText(f'节点: {node_count}')
        if edge_count is not None:
            self.lbl_edges.setText(f'边: {edge_count}')

    def update_frame(self, current: int, total: int):
        self.lbl_frame.setText(f'帧: {current} / {total}')

    def set_progress(self, value: int):
        self.progress.setVisible(True)
        self.progress.setValue(value)
        if value >= 100:
            self.progress.setVisible(False)

    def set_route_publish_enabled(self, route_ready: bool):
        self._btn_mission_publish.setEnabled(bool(route_ready))

    def set_mission_status(self, text: str):
        self._mission_status.setText(text)

    def set_play_button_text(self, playing: bool):
        self.btn_play.setText('Z - 暂停' if playing else 'Z - 播放')


def _btn_style(bg: str, fg: str) -> str:
    return (
        f'QPushButton {{ background: {bg}; color: {fg}; border: none; border-radius: 3px; padding: 6px 10px; }}'
        'QPushButton:hover { opacity: 0.85; }'
    )


def _tool_btn_style() -> str:
    return (
        'QPushButton { background: #f4f2ed; color: #2c2c2a; border: 0.5px solid #d3d1c7; '
        'border-radius: 3px; padding: 4px 8px; text-align: left; }'
        'QPushButton:checked { background: #1d9e75; color: #ffffff; border-color: #1d9e75; }'
    )


def _separator() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet('color: #d3d1c7;')
    return line
