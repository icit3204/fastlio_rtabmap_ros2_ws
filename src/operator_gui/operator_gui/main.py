"""Operator GUI entry point."""

from __future__ import annotations

import signal
import sys

from PyQt5 import QtCore, QtWidgets

from .main_window import MainWindow
from .ros_interface import RosInterface


def dispatch_command(command: str, ros: RosInterface) -> None:
    """Route a Qt control event to the existing Mission Manager client only."""
    handlers = {
        'start': ros.start_requested,
        'pause': ros.pause_requested,
        'resume': ros.resume_requested,
        'cancel': ros.cancel_requested,
    }
    signal = handlers.get(command)
    if signal is not None:
        signal.emit()


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    ros = RosInterface()
    thread = QtCore.QThread()
    ros.moveToThread(thread)
    thread.started.connect(ros.start)
    ros.view_changed.connect(window.update_view)
    ros.feedback.connect(window.show_feedback)
    window.command_requested.connect(lambda command: dispatch_command(command, ros))

    def shutdown() -> None:
        if thread.isRunning():
            QtCore.QMetaObject.invokeMethod(
                ros, 'stop', QtCore.Qt.BlockingQueuedConnection)
            thread.quit()
            thread.wait(2000)

    app.aboutToQuit.connect(shutdown)
    signal.signal(signal.SIGINT, lambda _signum, _frame: app.quit())
    thread.start()
    window.show()
    return app.exec_()
