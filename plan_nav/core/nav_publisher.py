# [IMPL] F-16.1 ROS2 /plan 话题发布器
# [IMPL] F-16.2 消息格式: nav_msgs/msg/Path（nav2 标准）
# [IMPL] F-16.4 环境复用: 与 PoseReceiver 相同的 auto-source 逻辑

"""
ROS2 /plan 话题发布模块。

操作模式下以 10Hz 持续发布从当前机器人位置到终点的剩余路径。
使用 nav_msgs/msg/Path 格式，与 nav2 生态兼容。
PlanPublisher 作为独立 QThread 运行，生命周期绑定操作模式。
"""

import json
import math
import os
import subprocess
import sys
import threading

from PyQt5.QtCore import QThread, pyqtSignal


def _source_ros2() -> str | None:
    """
    [IMPL] F-16.4 复用 PoseReceiver 的 auto-source 逻辑。
    扫描 /opt/ros/<distro>/setup.bash，注入环境变量后返回 ROS2 distro 名。
    不可用时返回 None（不发异常，由调用方容错处理）。
    """
    ros_base = '/opt/ros'
    if not os.path.isdir(ros_base):
        return None
    distros = sorted(
        [d for d in os.listdir(ros_base)
         if os.path.isfile(os.path.join(ros_base, d, 'setup.bash'))]
    )
    if not distros:
        return None
    distro = distros[-1]

    setup_path = os.path.join(ros_base, distro, 'setup.bash')
    cmd = f'source "{setup_path}" && python3 -c "import os,json; print(json.dumps(dict(os.environ)))"'
    try:
        proc = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True, text=True, timeout=15
        )
        if proc.returncode != 0:
            return None
        env_dict = json.loads(proc.stdout.strip())
        ros_keys = [
            'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
            'LD_LIBRARY_PATH', 'PATH', 'PYTHONPATH',
            'ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
            'ROS_PYTHON_VERSION', 'ROS_VERSION',
        ]
        for k in ros_keys:
            if k in env_dict:
                os.environ[k] = env_dict[k]
        return distro
    except Exception:
        return None


def _yaw_to_quaternion(yaw: float) -> tuple:
    """偏航角 → 四元数 (x, y, z, w)，仅绕 z 轴旋转"""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


class PlanPublisher(QThread):
    """持续发布 /plan 话题的独立线程。

    操作模式启动时创建，以 10Hz 发布剩余路径（当前位置 → 终点）。
    """

    log_message = pyqtSignal(str, str)  # (msg, level)  日志信号
    connected = pyqtSignal(str)         # 连接成功信号

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._traj: list[dict] = []
        self._current_pose: dict | None = None
        self._lock = threading.Lock()
        self._frame_id = 'map'

    def set_path(self, traj: list[dict], frame_id: str = 'map'):
        """设置完整规划轨迹（线程安全，由主线程调用）"""
        with self._lock:
            self._traj = list(traj)
            self._frame_id = frame_id

    def update_pose(self, x: float, y: float, z: float, yaw: float):
        """更新当前机器人位姿（线程安全，由主线程调用）"""
        with self._lock:
            self._current_pose = {'x': x, 'y': y, 'z': z, 'yaw': yaw}

    def run(self):
        """线程主体：创建 ROS2 节点 + 10Hz 定时发布"""
        self._running = True

        # PoseReceiver 已先启动并验证 ROS2 环境可用，直接导入即可
        try:
            import rclpy
        except ImportError as e:
            self.log_message.emit(f'PlanPublisher: rclpy 导入失败 ({e})', 'error')
            return
        try:
            from nav_msgs.msg import Path
        except ImportError as e:
            self.log_message.emit(
                f'PlanPublisher: nav_msgs 导入失败 ({e})，'
                f'请安装 ros-$ROS_DISTRO-nav-msgs', 'error'
            )
            return
        try:
            from geometry_msgs.msg import PoseStamped, Point, Quaternion
        except ImportError as e:
            self.log_message.emit(f'PlanPublisher: geometry_msgs 导入失败 ({e})', 'error')
            return
        try:
            from builtin_interfaces.msg import Time as RosTime
        except ImportError as e:
            self.log_message.emit(f'PlanPublisher: builtin_interfaces 导入失败 ({e})', 'error')
            return
        try:
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as e:
            self.log_message.emit(f'PlanPublisher: rclpy.executors 导入失败 ({e})', 'error')
            return

        try:
            from core.ros_runtime import ensure_rclpy_initialized
            ensure_rclpy_initialized(args=[])
        except Exception:
            pass

        node = None
        try:
            node = rclpy.create_node('plan_publisher')
            pub = node.create_publisher(Path, '/plan_nav', 10)

            self.connected.emit(
                f'PlanPublisher: /plan_nav 已创建 '
                f'(ROS2 {os.environ.get("ROS_DISTRO", "?")}'
                f' domain={os.environ.get("ROS_DOMAIN_ID", "0")})'
            )

            executor = SingleThreadedExecutor()
            executor.add_node(node)

            import time
            interval = 0.1
            while self._running:
                tick_start = time.time()

                path_msg = self._build_remaining_path(RosTime, Path, PoseStamped, Point, Quaternion)
                if path_msg is not None:
                    pub.publish(path_msg)

                try:
                    executor.spin_once(timeout_sec=0.01)
                except Exception:
                    pass

                elapsed = time.time() - tick_start
                remain = interval - elapsed
                if remain > 0:
                    time.sleep(remain)

        except Exception as e:
            self.log_message.emit(
                f'PlanPublisher: 运行时异常 ({e})，/plan 不可用', 'error'
            )
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass

    def _build_remaining_path(self, RosTime, Path, PoseStamped, Point, Quaternion) -> object | None:
        """构建从当前位置到终点的剩余路径消息（线程内调用）"""
        with self._lock:
            traj = self._traj
            pose = dict(self._current_pose) if self._current_pose else None
            frame_id = self._frame_id

        if not traj or pose is None:
            return None

        # 在轨迹上找离机器人最近的点
        cx, cy = pose['x'], pose['y']
        min_dist = float('inf')
        start_idx = 0
        for i, p in enumerate(traj):
            dx = p.get('x', 0.0) - cx
            dy = p.get('y', 0.0) - cy
            d = dx * dx + dy * dy
            if d < min_dist:
                min_dist = d
                start_idx = i

        # 最近点之后的剩余路径
        remaining = traj[start_idx:]
        if len(remaining) < 2:
            return None

        path_msg = Path()
        path_msg.header.frame_id = frame_id
        import time
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        path_msg.header.stamp = RosTime(sec=sec, nanosec=nsec)

        for p in remaining:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            ts = p.get('timestamp', now)
            s = int(ts)
            ns = int((ts - s) * 1e9)
            pose_stamped.header.stamp = RosTime(sec=s, nanosec=ns)

            pose_stamped.pose.position = Point(
                x=float(p.get('x', 0.0)),
                y=float(p.get('y', 0.0)),
                z=float(p.get('z', 0.0)),
            )
            qx, qy, qz, qw = _yaw_to_quaternion(p.get('yaw', 0.0))
            pose_stamped.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

            path_msg.poses.append(pose_stamped)

        return path_msg

    def stop(self):
        """停止发布线程"""
        self._running = False
        self.wait(3000)
