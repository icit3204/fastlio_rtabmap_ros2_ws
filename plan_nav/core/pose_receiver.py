# core/pose_receiver.py
# 操作模式下从 ROS2 TF 实时获取机器人位姿（map → base_footprint）

import os
import sys
import math
import subprocess

from PyQt5.QtCore import QThread, pyqtSignal


def _find_ros2_setup() -> str | None:
    """在 /opt/ros 下自动检测最新 ROS2 发行版的 setup.bash（相对路径检测）"""
    ros_root = '/opt/ros'
    if not os.path.isdir(ros_root):
        return None
    try:
        distros = sorted([
            d for d in os.listdir(ros_root)
            if os.path.isfile(os.path.join(ros_root, d, 'setup.bash'))
        ])
    except OSError:
        return None
    if not distros:
        return None
    # 优先选 humble > galactic > foxy，取最后一个（字母序最大）
    return os.path.join(ros_root, distros[-1], 'setup.bash')


def source_ros2_env(setup_path: str) -> bool:
    """
    Source ROS2 setup.bash，将环境变量注入当前进程。
    返回 True 表示成功，False 表示失败。
    """
    try:
        cmd = f'source "{setup_path}" && python3 -c "import os,json; print(json.dumps(dict(os.environ)))"'
        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True, text=True, timeout=15
        )
        if result.returncode != 0:
            return False
        import json
        env_dict = json.loads(result.stdout.strip())
        # 只注入 ROS2 相关的关键变量，避免污染整个环境
        ros_keys = [
            'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
            'LD_LIBRARY_PATH', 'PATH', 'PYTHONPATH',
            'ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
            'ROS_PYTHON_VERSION', 'ROS_VERSION',
        ]
        for k in ros_keys:
            if k in env_dict:
                os.environ[k] = env_dict[k]
        return True
    except Exception:
        return False


def check_ros2_available() -> tuple[bool, str]:
    """
    检查 ROS2 是否可用。
    返回 (可用, 诊断信息)
    """
    setup = _find_ros2_setup()
    if setup is None:
        return False, '未在 /opt/ros 下找到任何 ROS2 发行版'

    ok = source_ros2_env(setup)
    if not ok:
        return False, f'source {setup} 失败，请确认 ROS2 安装正确'

    # 尝试导入 rclpy
    try:
        import importlib
        importlib.import_module('rclpy')
        return True, f'ROS2 就绪 ({os.environ.get("ROS_DISTRO", "unknown")})'
    except ImportError:
        return False, (
            f'rclpy 模块不可用（ROS_DISTRO={os.environ.get("ROS_DISTRO", "?")}）。'
            '请确认已正确安装 ROS2 并 source 环境'
        )


class PoseReceiver(QThread):
    """
    实时位姿接收器（操作模式专用）。
    在独立线程中以 ~20Hz 通过 TF2 查询 map→base_footprint，
    将最新位姿通过信号发送给主线程。
    """

    # 位姿更新信号：发射 dict {x,y,z,yaw,qx,qy,qz,qw,stamp}
    pose_updated = pyqtSignal(dict)
    # 错误信号：发射错误描述字符串
    error_occurred = pyqtSignal(str)
    # 状态信号：连接成功后发射
    connected = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False

    def run(self):
        """线程主体：初始化 rclpy，持续查询 TF"""
        self._running = True

        # ── 1. 确保 ROS2 环境已 source ──────────────────────
        setup = _find_ros2_setup()
        if setup:
            source_ros2_env(setup)

        # ── 2. 尝试导入 rclpy ─────────────────────────────
        try:
            import rclpy
            from rclpy.node import Node
            from tf2_ros import Buffer, TransformListener, TransformException
        except ImportError:
            self.error_occurred.emit(
                '系统内没有检测到ROS2版本或无法正确收取到当前位姿。\n'
                f'（rclpy 导入失败，ROS_DISTRO={os.environ.get("ROS_DISTRO", "未知")}）'
            )
            return

        # ── 3. 初始化 ROS2 节点 ───────────────────────────
        try:
            from core.ros_runtime import ensure_rclpy_initialized
            ensure_rclpy_initialized(args=[])
        except Exception as e:
            self.error_occurred.emit(
                f'系统内没有检测到ROS2版本或无法正确收取到当前位姿。\n（rclpy.init 失败: {e}）'
            )
            return

        # ── 4. 创建节点和 TF 监听器 ────────────────────────
        try:
            node = rclpy.create_node('underground_map_pose_receiver')
            tf_buffer = Buffer()
            _tf_listener = TransformListener(tf_buffer, node)
        except Exception as e:
            self.error_occurred.emit(
                f'系统内没有检测到ROS2版本或无法正确收取到当前位姿。\n（节点创建失败: {e}）'
            )
            return

        self.connected.emit(f'ROS2 节点就绪 ({os.environ.get("ROS_DISTRO", "unknown")})')

        # ── 5. 主循环：20Hz 查询 TF ───────────────────────
        import time
        no_data_count = 0
        while self._running:
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
                tf = tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    rclpy.time.Time()
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                yaw = self._to_yaw(r)
                stamp = rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds * 1e-9

                pose = {
                    'x': t.x, 'y': t.y, 'z': t.z,
                    'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w,
                    'yaw': yaw,
                    'stamp': stamp,
                }
                self.pose_updated.emit(pose)
                no_data_count = 0

            except Exception:
                # TF 尚未就绪或短暂断线，忽略单次错误
                no_data_count += 1
                # 连续 100 次（约 5 秒）无数据则报错
                if no_data_count >= 100:
                    self.error_occurred.emit(
                        '系统内没有检测到ROS2版本或无法正确收取到当前位姿。\n'
                        '（持续 5 秒未收到 map→base_footprint TF 数据）'
                    )
                    no_data_count = 0  # 重置，避免重复弹窗

            time.sleep(0.05)  # ~20Hz

        # ── 6. 清理 ──────────────────────────────────────
        try:
            node.destroy_node()
        except Exception:
            pass

    def stop(self):
        """外部调用以停止线程"""
        self._running = False
        self.wait(3000)

    @staticmethod
    def _to_yaw(q) -> float:
        """四元数转偏航角"""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)
