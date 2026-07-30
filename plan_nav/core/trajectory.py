# [DONE] F-3.1 播放/暂停 (Z键), 按时间戳间隔推进
# [DONE] F-3.2 当前节点标识 (红色凸形图标, 朝向YAW)
# [DONE] F-3.3 轨迹着色 (已走绿色实线 / 未走灰色半透明)

from PyQt5.QtCore import QTimer, pyqtSignal, QObject


class TrajectoryPlayer(QObject):
    """轨迹播放器：按 .db 时间戳间隔逐帧播放节点"""

    frame_changed = pyqtSignal(int)   # 当前帧索引变化
    playback_done = pyqtSignal()      # 播放完成

    def __init__(self, nodes: list):
        super().__init__()
        self.nodes = nodes
        self.current_idx = 0
        self.speed = 1.0
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)

    @property
    def is_playing(self) -> bool:
        return self._timer.isActive()

    def set_speed(self, speed: float):
        """[IMPL] F-13.6 倍速支持：设置播放速度倍率，1.0 为原始速度"""
        self.speed = speed

    def play(self):
        """开始/恢复播放"""
        if self.current_idx >= len(self.nodes) - 1:
            self.current_idx = 0
        self._schedule_next()

    def pause(self):
        """暂停播放"""
        self._timer.stop()

    def toggle(self):
        """播放/暂停切换 (Z键)"""
        if self._timer.isActive():
            self.pause()
        else:
            self.play()

    def seek(self, idx: int):
        """跳转到指定帧"""
        self.current_idx = max(0, min(idx, len(self.nodes) - 1))
        self.frame_changed.emit(self.current_idx)

    def _tick(self):
        """定时器回调：发射当前帧信号，推进到下一帧"""
        self._timer.stop()
        self.frame_changed.emit(self.current_idx)
        self.current_idx += 1
        if self.current_idx < len(self.nodes):
            self._schedule_next()
        else:
            self.playback_done.emit()

    def _schedule_next(self):
        """按下一帧时间戳间隔设置定时器"""
        if self.current_idx < len(self.nodes) - 1:
            dt = (self.nodes[self.current_idx + 1]['timestamp']
                  - self.nodes[self.current_idx]['timestamp'])
            self._timer.start(max(10, int(dt * 1000 / self.speed)))
        else:
            self._tick()
