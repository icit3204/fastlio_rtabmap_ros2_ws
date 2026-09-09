import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtWidgets
from PyQt5.QtTest import QTest
from PyQt5.QtCore import Qt
from std_srvs.srv import SetBool, Trigger

from operator_gui.main import dispatch_command
from operator_gui.main_window import MainWindow
from operator_gui.ros_interface import RosInterface
from operator_gui.state_model import GuiView


class _Future:
    def __init__(self):
        self._response = SimpleNamespace(success=True, message='accepted')

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._response


class _Client:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _Future()


def test_gui_button_event_routes_start_without_local_state_mutation():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow()
    received = []
    window.command_requested.connect(received.append)
    window.update_view(GuiView(route_ready=True, state_code=1, mission_state='RECEIVED'))
    start = window._buttons['start']
    assert start.isEnabled()
    QTest.mouseClick(start, Qt.LeftButton)
    assert received == ['start']
    assert 'requested' in window._event.text().lower()
    window.close()
    app.processEvents()


def test_ros_interface_uses_exact_mission_manager_service_requests():
    ros = RosInterface()
    clients = {name: _Client() for name in ('start', 'pause', 'cancel')}
    ros._clients = clients
    ros.request_start()
    ros.request_pause()
    ros.request_resume()
    ros.request_cancel()
    assert isinstance(clients['start'].requests[0], Trigger.Request)
    assert clients['pause'].requests[0].data is True
    assert clients['pause'].requests[1].data is False
    assert isinstance(clients['cancel'].requests[0], Trigger.Request)


def test_dispatch_command_has_only_the_four_accepted_controls():
    ros = RosInterface()
    observed = []
    ros.start_requested.connect(lambda: observed.append('start'))
    ros.pause_requested.connect(lambda: observed.append('pause'))
    ros.resume_requested.connect(lambda: observed.append('resume'))
    ros.cancel_requested.connect(lambda: observed.append('cancel'))
    for command in ('start', 'pause', 'resume', 'cancel', 'unknown'):
        dispatch_command(command, ros)
    assert observed == ['start', 'pause', 'resume', 'cancel']
