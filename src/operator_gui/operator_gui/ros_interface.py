"""ROS 2 interface running in a Qt worker thread."""

from __future__ import annotations

import time

from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TwistStamped
from parking_robot_interfaces.msg import MissionState, RouteMission
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from PyQt5 import QtCore

from .state_model import GuiStateModel


class _GuiNode(Node):
    def __init__(self, owner: 'RosInterface') -> None:
        super().__init__('operator_gui')
        self.owner = owner
        self.declare_parameter('gate_state_topic', owner.gate_state_topic)
        owner.gate_state_topic = str(self.get_parameter('gate_state_topic').value)
        q = QoSProfile(depth=10)
        q.reliability = ReliabilityPolicy.RELIABLE
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(MissionState, '/mission/state', owner._mission_cb, q)
        self.create_subscription(RouteMission, '/mission/route', owner._route_cb, q)
        self.create_subscription(Bool, '/system/controller_valid', owner._controller_cb, 10)
        self.create_subscription(Bool, '/system/collision_monitor_valid', owner._perception_cb, 10)
        self.create_subscription(Bool, '/system/localization_valid', owner._localization_cb, 10)
        self.create_subscription(TwistStamped, '/vehicle_cmd_safe', owner._safe_speed_cb, 10)
        self.create_subscription(DiagnosticStatus, owner.gate_state_topic, owner._gate_cb, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', owner._nav_status_cb, 10)
        owner._clients = {
            'start': self.create_client(Trigger, '/mission/start'),
            'pause': self.create_client(SetBool, '/mission/pause'),
            'cancel': self.create_client(Trigger, '/mission/cancel'),
        }


class RosInterface(QtCore.QObject):
    """Nonblocking ROS transport; widgets receive only Qt signals."""

    view_changed = QtCore.pyqtSignal(object)
    feedback = QtCore.pyqtSignal(str)
    start_requested = QtCore.pyqtSignal()
    pause_requested = QtCore.pyqtSignal()
    resume_requested = QtCore.pyqtSignal()
    cancel_requested = QtCore.pyqtSignal()
    stop_requested = QtCore.pyqtSignal()

    def __init__(self, gate_state_topic: str = '/vehicle_cmd_safety/state') -> None:
        super().__init__()
        self.gate_state_topic = gate_state_topic
        self.model = GuiStateModel()
        self.node: _GuiNode | None = None
        self._clients = {}
        self._timer: QtCore.QTimer | None = None
        self.start_requested.connect(self.request_start)
        self.pause_requested.connect(self.request_pause)
        self.resume_requested.connect(self.request_resume)
        self.cancel_requested.connect(self.request_cancel)
        self.stop_requested.connect(self.stop)

    @QtCore.pyqtSlot()
    def start(self) -> None:
        if self.node is not None:
            return
        rclpy.init(args=None) if not rclpy.ok() else None
        self.node = _GuiNode(self)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._spin_once)
        self._timer.start(20)
        self._emit_view()

    @QtCore.pyqtSlot()
    def _spin_once(self) -> None:
        if self.node is None:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self._emit_view()
        except (ExternalShutdownException, RuntimeError) as exc:
            self.feedback.emit(f'ROS unavailable: {exc}')

    def _emit_view(self) -> None:
        self.view_changed.emit(self.model.view())

    def _mission_cb(self, msg) -> None:
        self.model.update_mission(msg)

    def _route_cb(self, msg) -> None:
        self.model.update_route(msg)

    def _controller_cb(self, msg) -> None:
        self.model.update_bool('controller_valid', msg.data)

    def _perception_cb(self, msg) -> None:
        self.model.update_bool('perception_valid', msg.data)

    def _localization_cb(self, msg) -> None:
        self.model.update_bool('localization_valid', msg.data)

    def _safe_speed_cb(self, msg) -> None:
        self.model.update_safe_speed(msg.twist.linear.x)

    def _gate_cb(self, msg) -> None:
        self.model.update_gate(msg)

    def _nav_status_cb(self, msg) -> None:
        if msg.status_list:
            self.model.update_nav_status(int(msg.status_list[-1].status))

    def _call(self, kind: str, request, label: str) -> None:
        client = self._clients.get(kind)
        if client is None or not client.service_is_ready():
            self.feedback.emit(f'{label} unavailable: service not ready')
            return
        try:
            future = client.call_async(request)
            future.add_done_callback(lambda done: self._service_done(done, label))
            self.feedback.emit(f'{label} requested...')
        except Exception as exc:
            self.feedback.emit(f'{label} failed: {exc}')

    def _service_done(self, future, label: str) -> None:
        try:
            response = future.result()
            result = 'accepted' if bool(response.success) else 'rejected'
            message = str(getattr(response, 'message', '') or '')
            self.feedback.emit(f'{label} {result}' + (f': {message}' if message else ''))
        except Exception as exc:
            self.feedback.emit(f'{label} failed: {exc}')

    @QtCore.pyqtSlot()
    def request_start(self) -> None:
        self._call('start', Trigger.Request(), 'START')

    @QtCore.pyqtSlot()
    def request_pause(self) -> None:
        self._call('pause', SetBool.Request(data=True), 'PAUSE')

    @QtCore.pyqtSlot()
    def request_resume(self) -> None:
        self._call('pause', SetBool.Request(data=False), 'RESUME')

    @QtCore.pyqtSlot()
    def request_cancel(self) -> None:
        self._call('cancel', Trigger.Request(), 'CANCEL')

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        if self._timer is not None:
            self._timer.stop()
            self._timer.deleteLater()
            self._timer = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
