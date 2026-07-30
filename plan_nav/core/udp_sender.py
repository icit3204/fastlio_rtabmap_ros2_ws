# [DONE] F-11.1 F-11.2 F-11.3 F-11.4 F-11.5
# F-11.1 动态预瞄半径：弯曲段 r→r_min，直线段 r→r_max
# F-11.2 R 计算（mm）并截断至 |R|≤10000
# F-11.3 R 符号：左转<0，右转>0
# F-11.4 速度分级：δ>阈值→v_turn，否则→v_straight
# F-11.5 二进制发送格式：struct.pack('>dd', R, v) 16字节

import math
import socket
import time
import struct
from PyQt5.QtCore import QThread, pyqtSignal


class UdpSender(QThread):
    """按路径点时间戳频率逐帧 UDP 发送 16 字节 binary (R, v)"""

    frame_sent = pyqtSignal(int)
    finished = pyqtSignal()

    def __init__(self, path_nodes: list, ip: str = '127.0.0.1',
                 port: int = 14550,
                 r_min=0.5, r_max=1.5, delta_thresh=30.0,
                 v_straight=5000.0, v_turn=3000.0):
        super().__init__()
        self.path_nodes = path_nodes
        self.ip, self.port = ip, port
        self.r_min = r_min
        self.r_max = r_max
        self.delta_thresh = math.radians(delta_thresh)
        self.v_straight = v_straight
        self.v_turn = v_turn
        self._running = True

    def update_params(self, r_min, r_max, delta_thresh,
                      v_straight, v_turn):
        """热生效：主线程随时调用，线程在下一帧读取新值"""
        self.r_min = r_min
        self.r_max = r_max
        self.delta_thresh = math.radians(delta_thresh)
        self.v_straight = v_straight
        self.v_turn = v_turn

    def run(self):
        """线程主循环：Pure Pursuit 计算 + binary UDP 发送"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        n = len(self.path_nodes)
        if n == 0:
            self.finished.emit()
            return

        # 初始化车辆状态（从第一个路径点出发）
        vx = self.path_nodes[0]['x']
        vy = self.path_nodes[0]['y']
        yaw = self.path_nodes[0].get('yaw', 0.0)

        for i in range(n):
            if not self._running:
                break

            # Pure Pursuit 计算
            r = self._dynamic_r(i)
            look_idx = self._lookahead_idx(i, vx, vy, r)
            lx = self.path_nodes[look_idx]['x']
            ly = self.path_nodes[look_idx]['y']
            delta = self._delta(vx, vy, yaw, lx, ly)
            R_val, v_val = self._calc_Rv(delta, r)

            # UDP 发送 16 字节 binary
            try:
                data = struct.pack('>dd', R_val, v_val)
                sock.sendto(data, (self.ip, self.port))
            except OSError:
                pass

            self.frame_sent.emit(i + 1)

            # 推进车辆位置
            nxt = min(i + 1, n - 1)
            vx = self.path_nodes[nxt]['x']
            vy = self.path_nodes[nxt]['y']
            yaw = self.path_nodes[nxt].get('yaw', yaw)

            dt = 0.1
            if i + 1 < n:
                t_cur = self.path_nodes[i].get('timestamp', 0)
                t_nxt = self.path_nodes[i + 1].get('timestamp', 0)
                if t_nxt > t_cur:
                    dt = min(t_nxt - t_cur, 1.0)
            time.sleep(dt)

        sock.close()
        self.finished.emit()

    def _dynamic_r(self, idx: int) -> float:
        nodes = self.path_nodes
        n = len(nodes)
        if n < 3 or idx >= n - 2:
            return self.r_max
        yaw_cur = nodes[idx].get('yaw', 0.0)
        yaw_nxt = nodes[min(idx + 1, n - 1)].get('yaw', 0.0)
        d_yaw = abs(math.atan2(math.sin(yaw_nxt - yaw_cur),
                               math.cos(yaw_nxt - yaw_cur)))
        ratio = max(0.0, 1.0 - d_yaw / (math.pi / 4))
        return self.r_min + ratio * (self.r_max - self.r_min)

    def _lookahead_idx(self, start: int, vx: float, vy: float, r: float) -> int:
        nodes = self.path_nodes
        n = len(nodes)
        for j in range(start, n):
            dx = nodes[j]['x'] - vx
            dy = nodes[j]['y'] - vy
            if math.hypot(dx, dy) >= r:
                return j
        return n - 1

    def _delta(self, vx, vy, yaw, lx, ly) -> float:
        angle_to_look = math.atan2(ly - vy, lx - vx)
        delta = angle_to_look - yaw
        delta = math.atan2(math.sin(delta), math.cos(delta))
        return delta

    def _calc_Rv(self, delta: float, r: float) -> tuple:
        sin_d = math.sin(delta)
        if abs(sin_d) < 1e-6:
            R = 10000.0
        else:
            R = -(r * 1000.0) / (2.0 * sin_d)
            if abs(R) > 10000.0:
                R = math.copysign(10000.0, R)
        v = self.v_straight if abs(delta) < self.delta_thresh else self.v_turn
        return R, v
