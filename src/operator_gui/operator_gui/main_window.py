"""Qt widgets for the standalone Operator GUI."""

from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets

from .state_model import GuiView, button_enabled


class MainWindow(QtWidgets.QMainWindow):
    command_requested = QtCore.pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('Parking Robot Operator')
        self.resize(900, 650)
        self._fields = {}
        self._buttons = {}
        self._build_ui()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(18, 18, 18, 18)
        title = QtWidgets.QLabel('Parking Robot — Operator Supervisory Interface')
        title.setObjectName('title')
        root.addWidget(title)
        sections = QtWidgets.QHBoxLayout()
        sections.addWidget(self._section('MISSION', [
            ('Mission ID', 'mission_id'), ('Start Node', 'start_node'), ('Goal Node', 'goal_node'),
            ('Current Edge', 'current_edge'), ('Current Goal', 'current_goal'),
            ('Progress', 'progress'), ('Mission Manager State', 'mission_state'),
        ]))
        sections.addWidget(self._section('NAVIGATION', [
            ('Nav2 State', 'nav2_state'), ('Controller Valid', 'controller_valid'),
            ('Safe Commanded Speed', 'safe_speed'),
        ]))
        sections.addWidget(self._section('SAFETY', [
            ('Perception Valid', 'perception_valid'), ('Localization Valid', 'localization_valid'),
            ('Controller Valid', 'safety_controller_valid'), ('Collision Monitor Validity', 'collision_validity'),
            ('Generic Gate State', 'gate_state'), ('Gate Reason', 'gate_reason'),
            ('Fault Reason', 'fault_reason'),
        ]))
        root.addLayout(sections)

        controls = QtWidgets.QGroupBox('CONTROLS')
        control_layout = QtWidgets.QHBoxLayout(controls)
        for name, text in (('start', 'START'), ('pause', 'PAUSE'), ('resume', 'RESUME'), ('cancel', 'CANCEL')):
            button = QtWidgets.QPushButton(text)
            button.setMinimumHeight(44)
            button.clicked.connect(lambda checked=False, n=name: self._request(n))
            self._buttons[name] = button
            control_layout.addWidget(button)
        root.addWidget(controls)

        event_box = QtWidgets.QGroupBox('LATEST EVENT')
        event_layout = QtWidgets.QVBoxLayout(event_box)
        self._event = QtWidgets.QLabel('Waiting for ROS status...')
        self._event.setWordWrap(True)
        self._event.setMinimumHeight(42)
        event_layout.addWidget(self._event)
        root.addWidget(event_box)
        self.setCentralWidget(central)
        self.setStyleSheet(
            '#title { font-size: 20px; font-weight: bold; padding-bottom: 8px; }'
            'QGroupBox { font-weight: bold; margin-top: 8px; }'
            'QLabel { font-size: 13px; }'
            'QPushButton { font-weight: bold; }'
        )

    def _section(self, title: str, rows: list[tuple[str, str]]) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QFormLayout(box)
        layout.setLabelAlignment(QtCore.Qt.AlignLeft)
        for label, key in rows:
            value = QtWidgets.QLabel('UNKNOWN')
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            value.setWordWrap(True)
            self._fields[key] = value
            layout.addRow(QtWidgets.QLabel(label), value)
        return box

    def _request(self, command: str) -> None:
        self._event.setText(f'{command.upper()} requested... authoritative state will confirm the transition.')
        self.command_requested.emit(command)

    @QtCore.pyqtSlot(object)
    def update_view(self, view: GuiView) -> None:
        for key in ('mission_id', 'start_node', 'goal_node', 'current_edge', 'current_goal',
                    'progress', 'mission_state', 'nav2_state', 'controller_valid', 'safe_speed',
                    'perception_valid', 'localization_valid', 'collision_validity', 'gate_state',
                    'gate_reason', 'fault_reason'):
            self._fields[key].setText(str(getattr(view, key)))
        self._fields['safety_controller_valid'].setText(view.controller_valid)
        for name, button in self._buttons.items():
            button.setEnabled(button_enabled(view, name))

    @QtCore.pyqtSlot(str)
    def show_feedback(self, text: str) -> None:
        self._event.setText(text)
