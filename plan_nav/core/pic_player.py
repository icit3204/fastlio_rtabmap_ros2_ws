# [DONE] F-13.3 图片帧列表获取：优先 datasets/result/，回退 datasets/pic/
# [DONE] F-13.4 图片播放器 PicPlayer：基于 QTimer 按时间戳间隔逐帧播放
# [DONE] F-13.5 图片/轨迹同步：当前帧索引与轨迹节点索引一致
# [DONE] F-13.6 倍速支持：set_speed() 控制播放速度倍率

import os
from PyQt5.QtCore import QTimer, pyqtSignal, QObject


def list_pic_frames(db_workdir: str) -> tuple:
    """获取数据库工作目录对应的图片帧文件列表

    Args:
        db_workdir: 数据库工作目录，如 .../data/underGround_split1/

    Returns:
        (排序后的绝对路径列表, 来源标签)  -- 来源标签为 'result' / 'pic' / 'none'
    """
    # 从 db_workdir 向上 2 级到达项目根目录
    project_root = os.path.normpath(os.path.join(db_workdir, '..', '..'))

    # 优先检查 datasets/result/（已处理的自动节点图片）
    result_dir = os.path.join(project_root, 'datasets', 'result')
    if os.path.isdir(result_dir):
        result_files = sorted([
            os.path.join(result_dir, f)
            for f in os.listdir(result_dir)
            if f.lower().endswith('.jpg')
        ])
        if result_files:
            return (result_files, 'result')

    # 回退到 datasets/pic/（原始提取帧）
    pic_dir = os.path.join(project_root, 'datasets', 'pic')
    if os.path.isdir(pic_dir):
        pic_files = sorted([
            os.path.join(pic_dir, f)
            for f in os.listdir(pic_dir)
            if f.lower().endswith('.jpg')
        ])
        if pic_files:
            return (pic_files, 'pic')

    return ([], 'none')


class PicPlayer(QObject):
    """图片帧播放器：按时间戳间隔逐帧播放 JPG 图片序列

    与 TrajectoryPlayer 保持一致的接口设计，便于同步控制。
    """

    frame_changed = pyqtSignal(int)   # 当前帧索引变化
    playback_done = pyqtSignal()      # 播放完成

    SPEEDS = [0.5, 1.0, 1.5, 2.0, 3.0]

    def __init__(self, frame_paths: list, frame_timestamps: list):
        """
        Args:
            frame_paths: 图片文件的绝对路径列表
            frame_timestamps: 对应的时间戳列表（秒），与 frame_paths 等长
        """
        super().__init__()
        if len(frame_paths) != len(frame_timestamps):
            raise ValueError(
                f"frame_paths 与 frame_timestamps 长度不一致: "
                f"{len(frame_paths)} vs {len(frame_timestamps)}"
            )
        self.frame_paths = frame_paths
        self.frame_timestamps = frame_timestamps
        self.current_idx = 0
        self.speed = 1.0
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)

    @property
    def is_playing(self) -> bool:
        """是否正在播放"""
        return self._timer.isActive()

    def set_speed(self, speed: float):
        """设置播放速度倍率，1.0 为原始速度"""
        self.speed = speed

    def play(self):
        """开始/恢复播放；若已到末尾则从头开始"""
        if self.current_idx >= len(self.frame_paths) - 1:
            self.current_idx = 0
        self._schedule_next()

    def pause(self):
        """暂停播放"""
        self._timer.stop()

    def toggle(self):
        """播放/暂停切换"""
        if self._timer.isActive():
            self.pause()
        else:
            self.play()

    def seek(self, idx: int):
        """跳转到指定帧索引"""
        self.current_idx = max(0, min(idx, len(self.frame_paths) - 1))
        self.frame_changed.emit(self.current_idx)

    def _tick(self):
        """定时器回调：发射当前帧信号，推进到下一帧"""
        self._timer.stop()
        self.frame_changed.emit(self.current_idx)
        self.current_idx += 1
        if self.current_idx < len(self.frame_paths):
            self._schedule_next()
        else:
            self.playback_done.emit()

    def _schedule_next(self):
        """按下一帧时间戳间隔除以倍速来设置定时器"""
        if self.current_idx < len(self.frame_paths) - 1:
            dt = (self.frame_timestamps[self.current_idx + 1]
                  - self.frame_timestamps[self.current_idx])
            # 间隔 = 时间戳差值(秒) * 1000(毫秒) / 倍速，最小 10ms
            interval = max(10, int(dt * 1000 / self.speed))
            self._timer.start(interval)
        else:
            self._tick()
