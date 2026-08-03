# [DONE] F-1.4 工作目录创建
# [DONE] F-1.5 自动加载地图
# [DONE] F-8.1 实时日志面板集成
# [DONE] F-8.2 UI 配置编辑集成
# [DONE] F-9.12 相对路径基础设施
# [DONE] F-9.1 处理数据按钮业务接入
# [DONE] F-9.9 轨迹自动加载
# [DONE] F-10.2 settings.json 加载 + 识别参数初值注入
# [DONE] F-10.8 auto_node 业务接入
# [DONE] F-10.9 防重复触发保护
# [DONE] F-10.10 6 级日志规范
# [DONE] F-11.8 轨迹参数热生效
# [DONE] F-11.9 轨迹参数持久化 + 默认值兜底
# [IMPL] F-15.1 节点携带 traj_idx
# [IMPL] F-15.11 旧边自动补建轨迹文件
# [IMPL] F-15.14 规划路径改用轨迹拼接 (concat_trajectory_segments)
# [IMPL] F-16.1 操作模式下 /plan_nav 话题发布
# [IMPL] F-17.2/F-17.4/F-17.5/F-17.6 节点注释编辑

from core.ui_font import mono_font  # [ADAPT-UBU-02] 跨平台等宽字体
import os
import sys
import json
import uuid
from pathlib import Path
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QFileDialog, QStatusBar, QLabel, QShortcut,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QKeySequence

from core.db_loader import load_db
# [COMMENTED] 暂时注释地图解码，仅显示轨迹
# from core.map_decoder import decode_map
from core.trajectory import TrajectoryPlayer
from core.topology import TopologyManager
from core.pathfinder import build_graph, find_path, interpolate_path, concat_trajectory_segments  # [IMPL] F-15.14
from core.udp_sender import UdpSender
from core.mission_bridge import (
    AuthorityMode,
    MissionBridgeThread,
    TERMINAL_MISSION_STATES,
    verify_route_topology_current,
)
from core.topology_identity import build_sparse_route_spec, load_topology, read_manifest

from ui.sidebar import Sidebar
from ui.map_view import MapView
from ui.log_panel import LogPanel

# [IMPL] F-12.6 直线插值辅助函数
import math as _math

def _interpolate_line(x0: float, y0: float,
                      x1: float, y1: float,
                      step: float = 0.05) -> list:
    """在 (x0,y0)→(x1,y1) 之间按 step 间距插值，返回含 x/y/yaw/timestamp 的点列表"""
    dist = _math.hypot(x1 - x0, y1 - y0)
    if dist < 1e-4:
        return []
    yaw = _math.atan2(y1 - y0, x1 - x0)
    n = max(1, int(dist / step))
    pts = []
    for i in range(n + 1):
        t = i / n
        pts.append({
            'x': x0 + t * (x1 - x0),
            'y': y0 + t * (y1 - y0),
            'yaw': yaw,
            'timestamp': 0.0,
        })
    return pts


class MainWindow(QMainWindow):
    """主窗口：侧栏 + 地图视口 + 日志面板 + 状态栏"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Underground Map Editor')
        self.resize(1200, 720)
        self.setStyleSheet('background: #f4f2ed;')

        self.db_data = None
        self.player = None
        self.topology = None
        self.udp_sender = None
        self._current_traj = []  # 当前发送中的插值轨迹
        self._selected_nodes = []  # 规划模式下的节点选择队列
        self._udp_ip = '127.0.0.1'
        self._udp_port = 14550
        self._roboflow_api_key = ''
        self._yes_conf = 0.65
        self._axis_tolerance = 0.2
        self._pursuit_r_min = 0.5
        self._pursuit_r_max = 1.5
        self._pursuit_delta_thresh = 30.0
        self._pursuit_v_straight = 5000.0
        self._pursuit_v_turn = 3000.0
        self._load_settings()

        self._setup_ui()
        self._setup_shortcuts()

        # F-10.7 异步识别线程引用，None 表示未启动
        self.detector_thread = None

        # [IMPL] F-13.1/F-13.11 图片回放联动
        self.pic_player = None
        self.pic_overlay = None  # 延迟创建，首次播放时才 new PicOverlay(self.map_view)
        self._hover_preview = None  # [IMPL] F-14.1/F-14.5 悬停预览浮层

        # [IMPL] F-12.7 PoseReceiver 线程管理
        self._pose_receiver = None       # PoseReceiver | None
        self._plan_publisher = None      # PlanPublisher | None
        self._mission_bridge = None      # MissionBridgeThread | None
        self._authority_mode = AuthorityMode.LEGACY.value
        self._pending_route_spec = None
        self._published_route_spec = None
        self._active_mission_id = ''
        self._active_route_id = ''
        self._latest_mission_state = None
        self._last_pose_time = 0.0       # 最后一次收到 TF 的时间戳
        self._current_robot_pose = None  # 缓存的实时机器人位姿
        self._pose_timeout_timer = QTimer(self)
        self._pose_timeout_timer.setInterval(1000)
        self._pose_timeout_timer.timeout.connect(self._check_pose_timeout)

    # ─── F-10.2 启动配置加载 ────────────────────────────

    def _load_settings(self):
        """F-10.2 启动加载 settings.json"""
        import json
        path = os.path.join(self._project_root(), 'config', 'settings.json')
        if not os.path.exists(path):
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except (json.JSONDecodeError, OSError):
            return
        self._udp_ip = cfg.get('udp_ip', self._udp_ip)
        self._udp_port = int(cfg.get('udp_port', self._udp_port))
        self._roboflow_api_key = cfg.get('roboflow_api_key', '')
        self._yes_conf = float(cfg.get('yes_conf_default', 0.65))
        self._axis_tolerance = float(cfg.get('axis_tolerance_default', 0.2))
        self._pursuit_r_min = float(cfg.get('pursuit_r_min', 0.5))
        self._pursuit_r_max = float(cfg.get('pursuit_r_max', 1.5))
        self._pursuit_delta_thresh = float(cfg.get('pursuit_delta_thresh', 30.0))
        self._pursuit_v_straight = float(cfg.get('pursuit_v_straight', 5000.0))
        self._pursuit_v_turn = float(cfg.get('pursuit_v_turn', 3000.0))
        cfg.setdefault('pic_playback_speed', 1.0)
        cfg.setdefault('pic_overlay_geometry', [1480, 76, 300, 320])
        self._settings = cfg

    # ─── F-9.12 相对路径基础设施 ─────────────────────────

    def _project_root(self):
        """返回项目根目录绝对路径（ui/ 的上一级）"""
        if getattr(sys, 'frozen', False):
            return sys._MEIPASS
        ui_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.dirname(ui_dir)

    def _ensure_trajectory(self):
        """F-9.9 优先复用 db_data；未加载则扫描项目根目录首个 .db 自动加载"""
        if self.db_data:
            return self.db_data['nodes']
        import glob
        root = self._project_root()
        dbs = sorted(glob.glob(os.path.join(root, '*.db')))
        if not dbs:
            return None
        self.log(f'自动加载轨迹: {os.path.basename(dbs[0])}', 'info')
        self.db_data = load_db(dbs[0])
        return self.db_data['nodes']

    def _on_process_data(self):
        """F-9.1 处理数据按钮主流程"""
        from core.video_processor import pick_video, clear_pic_dir, extract_frames
        root = self._project_root()
        video_dir = os.path.join(root, 'datasets', 'video')
        pic_dir = os.path.join(root, 'datasets', 'pic')

        # F-9.9 轨迹来源
        nodes = self._ensure_trajectory()
        if not nodes:
            self.log('未找到轨迹数据（请先导入 .db 或确认根目录存在 .db 文件）', 'error')
            return

        # F-9.2 视频源选择
        video = pick_video(video_dir)
        if not video:
            self.log(f'目录 {os.path.relpath(video_dir, root)} 内无可用视频', 'error')
            return
        self.log(f'选定视频: {os.path.basename(video)}', 'info')

        # F-9.8 清空 → F-9.4/5/6/7/10 抽帧
        n_cleared = clear_pic_dir(pic_dir)
        if n_cleared > 0:
            self.log(f'已清空 {n_cleared} 张旧图片', 'info')

        self.sidebar.set_progress(0)
        result = extract_frames(
            video, nodes, pic_dir,
            progress_cb=lambda p: self.sidebar.set_progress(p)
        )

        if result['reason']:
            self.log(f'抽帧失败: {result["reason"]}', 'error')
            return
        self.log(
            f'抽帧完成: 总 {result["total"]} 节点 / 成功 {result["saved"]} 张 '
            f'/ 跳过 {result["skipped"]}',
            'warn'
        )

    def _on_auto_node(self):
        """F-10.8 目标识别业务主流程"""
        from core.object_detector import ObjectDetectorThread

        # F-10.9 防重复触发
        if self.detector_thread is not None and self.detector_thread.isRunning():
            self.log('识别进行中，请等待', 'info')
            return

        root = self._project_root()
        pic_dir = os.path.join(root, 'datasets', 'pic')
        result_dir = os.path.join(root, 'datasets', 'result')

        # 前置检查 1：pic 目录非空
        if not os.path.isdir(pic_dir) or not any(
            f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.webp'))
            for f in os.listdir(pic_dir)
        ):
            self.log('datasets/pic/ 为空，请先点击「处理数据」', 'error')
            return

        # 前置检查 2：轨迹已加载
        nodes = self._ensure_trajectory()
        if not nodes:
            self.log('未找到轨迹数据，无法注入 z/yaw', 'error')
            return

        # 前置检查 3：API key 已配置
        if not self._roboflow_api_key:
            self.log('未配置 roboflow_api_key（请编辑 config/settings.json）', 'error')
            return

        # 统计待处理图片数量
        img_count = sum(
            1 for f in os.listdir(pic_dir)
            if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.webp'))
        )

        # F-10.10 启动日志
        self.log(
            f'auto_node 启动: {img_count} 张图待处理 '
            f'(yes_conf={self._yes_conf}, axis_tol={self._axis_tolerance})',
            'warn'
        )

        # 创建异步线程
        self.detector_thread = ObjectDetectorThread(
            pic_dir, result_dir, nodes,
            self._roboflow_api_key,
            self._yes_conf, self._axis_tolerance,
        )
        # 信号连接
        self.detector_thread.yes_detected.connect(self._on_yes_node)
        self.detector_thread.log_message.connect(self.log)
        self.detector_thread.progress.connect(self._on_detect_progress)
        self.detector_thread.finished_with_stats.connect(self._on_detect_done)
        self.detector_thread.start()

    def _on_yes_node(self, x: float, y: float, z: float,
                     yaw: float, t: float, fname: str):
        """F-10.7 主线程接收 yes 信号 → add_waypoint + 实时刷新"""
        if not self.topology:
            return
        # [IMPL] F-15.1 按 timestamp 在 db_nodes 中查找轨迹索引
        traj_idx = -1
        if self.db_data and self.db_data['nodes']:
            import math as _m
            best_dist = float('inf')
            for i, n in enumerate(self.db_data['nodes']):
                d = abs(n['timestamp'] - t)
                if d < best_dist:
                    best_dist = d
                    traj_idx = i
        prev_count = len(self.topology.waypoints)
        wp = self.topology.add_waypoint(x, y, z, yaw, t, traj_idx=traj_idx)
        new_count = len(self.topology.waypoints)
        # F-10.10 区分新增 vs 合并
        if new_count > prev_count:
            self.log(
                f'yes ✓ 添加 {wp["label"]} 在 ({x:.2f}, {y:.2f})',
                'warn'
            )
        else:
            # 计算距合并节点的距离
            import math
            dist = math.hypot(wp['x'] - x, wp['y'] - y)
            self.log(
                f'yes ✓ 合并到 {wp["label"]} (距 {dist:.2f}m)',
                'info'
            )
        # 实时刷新地图
        self.map_view.draw_waypoints(self.topology.waypoints)
        self.map_view.draw_edges(self.topology.edges, self.topology.waypoints, topology=self.topology)
        self._status_node.setText(
            f'节点: {len(self.topology.waypoints)} | '
            f'边: {len(self.topology.edges)}'
        )

    def _on_detect_progress(self, current: int, total: int):
        """F-10.7 进度反馈到 sidebar 进度条"""
        if total > 0:
            self.sidebar.set_progress(int(current / total * 100))

    def _on_detect_done(self, stats: dict):
        """F-10.10 完成汇总日志"""
        if stats.get('reason'):
            self.log(f'auto_node 失败: {stats["reason"]}', 'error')
            return
        self.log(
            f'auto_node 完成: 识别 {stats["total"]} / '
            f'yes {stats["yes"]} / '
            f'新增节点 {stats["added"]} / '
            f'失败 {stats["failed"]}',
            'warn'
        )

    # ─── UI 布局 ────────────────────────────────────────

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 左侧边栏 (220px)
        self.sidebar = Sidebar(self)
        layout.addWidget(self.sidebar)

        # 主视口
        self.map_view = MapView(self)
        layout.addWidget(self.map_view, stretch=1)

        # 右侧日志面板 (200px)
        self.log_panel = LogPanel(self)
        layout.addWidget(self.log_panel)

        # ─── 信号连接 ───
        self.sidebar.import_requested.connect(self.import_db)
        self.sidebar.play_toggled.connect(self._on_play_toggle)
        self.sidebar.tool_changed.connect(self._on_tool_change)
        self.sidebar.reset_requested.connect(self._on_reset)
        self.sidebar.process_data_requested.connect(self._on_process_data)
        self.sidebar.auto_node_requested.connect(self._on_auto_node)
        # [IMPL] F-12.1 模式切换信号
        self.sidebar.mode_changed.connect(self._on_mode_changed)
        self.sidebar.authority_mode_changed.connect(self._on_authority_mode_changed)
        self.sidebar.mission_publish_requested.connect(self._on_publish_mission_requested)
        self.sidebar.mission_start_requested.connect(self._on_start_mission_requested)
        self.sidebar.mission_cancel_requested.connect(self._on_cancel_mission_requested)
        self.sidebar.mission_pause_requested.connect(self._on_pause_mission_requested)
        self.sidebar.mission_resume_requested.connect(self._on_resume_mission_requested)
        self.log_panel.lock_requested.connect(self.lock_current_node)
        self.log_panel.config_changed.connect(self._on_config_change)
        self.log_panel.traj_params_changed.connect(self._on_traj_params_change)
        self.map_view.world_clicked.connect(self._on_map_click)
        # [IMPL] F-17.1 平移模式点击节点 → 注释编辑
        self.map_view.annotation_requested.connect(self._on_annotation_request)

        # ─── 状态栏 ───
        self.status_bar = QStatusBar()
        self.status_bar.setFont(mono_font(8))  # [ADAPT-UBU-02] 跨平台等宽字体
        self.status_bar.setStyleSheet(
            'QStatusBar { background: #ffffff; border-top: 0.5px solid #d3d1c7; '
            'color: #888780; padding: 2px 8px; }'
        )
        self.setStatusBar(self.status_bar)
        self._status_node = QLabel('节点: 0')
        self._status_edge = QLabel('边: 0')
        self._status_path = QLabel('路径: --')
        self._status_hint = QLabel('拖拽平移 · 滚轮缩放')
        self.status_bar.addWidget(self._status_node)
        self.status_bar.addWidget(self._status_edge)
        self.status_bar.addWidget(self._status_path)
        self.status_bar.addPermanentWidget(self._status_hint)

        # F-10.2 注入识别参数初值到右栏输入框
        self.log_panel.set_detector_defaults(self._yes_conf, self._axis_tolerance)
        # F-11.9 注入轨迹控制参数初值
        self.log_panel.set_traj_defaults(
            self._pursuit_r_min, self._pursuit_r_max,
            self._pursuit_delta_thresh,
            self._pursuit_v_straight, self._pursuit_v_turn,
        )

        # [IMPL] F-14.1/F-14.5 节点悬停信号连接
        self.map_view.node_hover_enter.connect(self._on_node_hover_enter)
        self.map_view.node_hover_leave.connect(self._on_node_hover_leave)

    # ─── 快捷键 ─────────────────────────────────────────

    def _setup_shortcuts(self):
        QShortcut(QKeySequence('Z'), self, self._on_play_toggle)
        QShortcut(QKeySequence('Space'), self, self.lock_current_node)
        # F-U1: Ctrl+Z 撤销
        QShortcut(QKeySequence('Ctrl+Z'), self, self._undo)
        # 复原：清空所有节点/边/路径
        QShortcut(QKeySequence('Ctrl+R'), self, self._on_reset)
        # 工具快捷键
        QShortcut(QKeySequence('1'), self, lambda: self._set_tool('pan'))
        QShortcut(QKeySequence('2'), self, lambda: self._set_tool('node'))
        QShortcut(QKeySequence('3'), self, lambda: self._set_tool('edge'))
        QShortcut(QKeySequence('4'), self, lambda: self._set_tool('plan'))
        QShortcut(QKeySequence('5'), self, lambda: self._set_tool('delete_edge'))
        QShortcut(QKeySequence('6'), self, lambda: self._set_tool('delete_node'))

    # ─── 日志 ─────────────────────────────────────────

    def log(self, msg: str, level: str = ''):
        """向右侧日志面板追加一条日志"""
        self.log_panel.append(msg, level)

    # ─── 数据库导入 (F-1.1, F-1.4, F-1.5) ──────────────

    def import_db(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '导入 RTAB-Map 数据库', '',
            'RTAB-Map DB (*.db);;All Files (*)'
        )
        if not path:
            return

        self.log('导入 ' + os.path.basename(path))

        # 重置图片回放，避免复用旧数据集的图片
        self.pic_player = None
        self.pic_overlay = None
        if self._hover_preview:
            self._hover_preview.hide()
            self._hover_preview = None

        # F-1.1: 解析数据库（带进度回调）
        data = load_db(path, progress_callback=self.sidebar.set_progress)
        self.db_data = data
        self.log(f'解析完成: {len(data["nodes"])} 个节点, {len(data["links"])} 条约束')

        # F-1.4: 创建工作目录
        stem = os.path.splitext(os.path.basename(path))[0]
        work_dir = os.path.join(os.path.dirname(path) or '.', stem)
        os.makedirs(work_dir, exist_ok=True)
        self.log(f'工作目录: {work_dir}', 'info')

        # [COMMENTED] 暂时注释地图解码和底图加载，仅显示轨迹
        # if data['map_blob']:
        #     map_meta = decode_map(data['map_blob'], data['map_meta'])
        #     self.map_view.load_map(map_meta)
        #     self.log(
        #         f'地图加载: {map_meta["width"]}×{map_meta["height"]} '
        #         f'(分辨率 {map_meta["resolution"]}m/cell)', 'info'
        #     )
        self.map_view.setup_trajectory_view(data['map_meta'], data['nodes'])

        # 初始化播放器
        self.player = TrajectoryPlayer(data['nodes'])
        self.player.frame_changed.connect(self._on_frame)
        self.player.playback_done.connect(self._on_playback_done)

        # 初始化拓扑管理器，加载已有节点和边
        self.topology = TopologyManager(work_dir)
        loaded_n, loaded_e = self.topology.load_all()
        if loaded_n > 0 or loaded_e > 0:
            self.log(
                f'恢复已保存数据: {loaded_n} 个节点, {loaded_e} 条边', 'info'
            )

        # 更新统计
        self.sidebar.update_stats(
            node_count=len(data['nodes']),
            edge_count=len(data['links']),
        )
        self._status_node.setText(f'节点: {len(data["nodes"])}')

        # [IMPL] F-15.11 旧边自动补建轨迹文件（必须在 draw_edges 之前执行）
        self._rebuild_edge_trajectories()

        # F-1.5: 自动显示底图 + 初始轨迹 + 已保存节点/边
        self.map_view.draw_trajectory(data['nodes'], 0)
        self.map_view.draw_robot(
            data['nodes'][0]['x'], data['nodes'][0]['y'],
            data['nodes'][0]['yaw']
        )
        self.map_view.draw_waypoints(self.topology.waypoints)
        self.map_view.draw_edges(self.topology.edges, self.topology.waypoints,
                                 topology=self.topology)  # [IMPL] F-15.12
        self.sidebar.update_frame(0, len(data['nodes']))
        self._status_node.setText(
            f'语义节点: {len(self.topology.waypoints)} | '
            f'拓扑边: {len(self.topology.edges)}'
        )
        self.log('就绪，按 Z 开始播放', 'info')

    def _rebuild_edge_trajectories(self):
        """
        [IMPL] F-15.11 扫描所有 traj_file 为空的边，自动从 DB 轨迹提取并生成轨迹文件。
        先修复所有缺失 traj_idx 的节点（按 timestamp 匹配），再补建边的轨迹文件。
        导入旧格式 edges.txt 后调用，完成后更新 edges.txt 为新格式。
        """
        if not self.db_data or not self.topology:
            return

        db_nodes = self.db_data['nodes']

        # [FIX] 2026-07-04 先修复所有缺失 traj_idx 的节点（不仅是边端点）
        # 否则未连边的节点在后续手动连边时无法提取轨迹段，只能画直线
        fixed_indices = 0
        for w in self.topology.waypoints:
            if w.get('traj_idx', -1) < 0:
                idx = self._find_traj_idx_by_ts(db_nodes, w['timestamp'])
                if idx >= 0:
                    w['traj_idx'] = idx
                    fixed_indices += 1
        if fixed_indices > 0:
            self.topology.save_nodes()
            self.log(f'已修复 {fixed_indices} 个节点的轨迹索引', 'info')

        rebuilt = 0
        for e in self.topology.edges:
            if e.get('traj_file'):
                continue  # 已有轨迹文件，跳过

            from_wp = next((w for w in self.topology.waypoints
                           if w['id'] == e['from_id']), None)
            to_wp = next((w for w in self.topology.waypoints
                         if w['id'] == e['to_id']), None)
            if not from_wp or not to_wp:
                continue

            if from_wp['traj_idx'] < 0 or to_wp['traj_idx'] < 0:
                continue  # 节点无 DB 关联

            # 提取轨迹段
            points = self.topology._extract_trajectory_segment(
                db_nodes, from_wp, to_wp, e['direction'])
            if points:
                edge_idx = self.topology.edges.index(e) + 1
                traj_file = f'edge_{edge_idx}_traj.txt'
                self.topology._save_edge_trajectory(traj_file, points)
                e['traj_file'] = traj_file
                rebuilt += 1

        if rebuilt > 0:
            self.topology.save_edges()
            self.log(f'已补建 {rebuilt} 条边的轨迹文件', 'info')

    def _find_traj_idx_by_ts(self, db_nodes: list, timestamp: float) -> int:
        """[IMPL] F-15.11 按时间戳在 db_nodes 中二分查找最近匹配索引"""
        import math as _m
        best_idx, best_dist = -1, float('inf')
        for i, n in enumerate(db_nodes):
            d = abs(n['timestamp'] - timestamp)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    # ─── 播放控制 (F-3.1) ──────────────────────────────

    def _on_play_toggle(self):
        if not self.player:
            return
        self.player.toggle()
        state = '播放中' if self.player.is_playing else '已暂停'
        self.log(state, 'info')
        self.sidebar.set_play_button_text(self.player.is_playing)

        # [IMPL] F-13.1 / F-13.11 播放联动：首次播放时创建/唤醒悬浮窗
        self._ensure_pic_player()
        if self.pic_player:
            self.pic_player.toggle()
            if self.pic_overlay:
                self.pic_overlay.show()
                self.pic_overlay.set_play_text(self.pic_player.is_playing)

    def _on_frame(self, idx):
        """每帧更新"""
        if not self.db_data:
            return
        node = self.db_data['nodes'][idx]
        self.map_view.draw_trajectory(self.db_data['nodes'], idx)
        self.map_view.draw_robot(node['x'], node['y'], node['yaw'])
        self.map_view.draw_waypoints(self.topology.waypoints)
        self.map_view.draw_edges(self.topology.edges, self.topology.waypoints, topology=self.topology)
        self.sidebar.update_frame(idx, len(self.db_data['nodes']))
        # 状态栏显示当前坐标
        self._status_node.setText(
            f'坐标: ({node["x"]:.2f}, {node["y"]:.2f}) '
            f'YAW: {node["yaw"]:.2f}°'
        )

    def _on_playback_done(self):
        self.log('播放完成', 'info')
        self.sidebar.set_play_button_text(False)

    # ─── 图片回放联动 (F-13.x / F-14.x) ──────────────────

    def _ensure_pic_player(self):
        """[IMPL] F-13.2 图片来源回退：result/ 非空优先，否则 pic/，仍为空则不创建悬浮窗"""
        if self.pic_player is not None:
            return
        import os
        from core.frame_matcher import parse_frame_timestamp
        from core.pic_player import PicPlayer

        root = self._project_root()

        # F-13.2 优先级：datasets/result/ → datasets/pic/
        # 逐个尝试目录，校验时间戳匹配当前轨迹后才使用
        paths = []
        source = 'none'
        for d, tag in [(os.path.join(root, 'datasets', 'result'), 'result'),
                       (os.path.join(root, 'datasets', 'pic'), 'pic')]:
            if not os.path.isdir(d):
                continue
            files = sorted(f for f in os.listdir(d) if f.lower().endswith('.jpg'))
            if not files:
                continue
            cand_paths = [os.path.join(d, f) for f in files]
            timestamps = [parse_frame_timestamp(os.path.basename(p)) for p in cand_paths]

            # 校验时间戳是否与当前轨迹匹配（避免旧数据集残留）
            if self.db_data and self.db_data['nodes']:
                traj_ts_min = self.db_data['nodes'][0]['timestamp']
                traj_ts_max = self.db_data['nodes'][-1]['timestamp']
                valid_ts = [t for t in timestamps if t is not None
                            and traj_ts_min <= t <= traj_ts_max]
                if not valid_ts:
                    self.log(
                        f'{tag}/ 图片时间戳与当前轨迹不匹配，跳过', 'info'
                    )
                    continue

            paths = cand_paths
            source = tag
            break

        if not paths:
            self.log('未找到与当前轨迹匹配的图片（请点击「处理数据」）', 'warn')
            return

        self.pic_player = PicPlayer(paths, timestamps)
        self.pic_player.frame_changed.connect(self._on_pic_frame)

        from ui.pic_overlay import PicOverlay
        self.pic_overlay = PicOverlay(self.map_view)  # F-13.10: map_view 为 parent
        self.pic_overlay.move(self.map_view.width() - 336, 40)  # 默认右上角
        self.pic_overlay.play_toggled.connect(self._on_play_toggle)       # F-13.11
        self.pic_overlay.seek_requested.connect(self.pic_player.seek)     # F-13.4
        self.pic_overlay.speed_changed.connect(self._on_pic_speed_change) # F-13.3/F-13.6

        # F-13.3 从 settings 恢复上次倍速
        saved_speed = self._settings.get('pic_playback_speed', 1.0)
        self.pic_player.set_speed(saved_speed)
        if self.player and hasattr(self.player, 'set_speed'):
            self.player.set_speed(saved_speed)

        self.log(f'图片回放就绪：来源={source}，共 {len(paths)} 帧', 'info')

    def _on_pic_frame(self, idx: int):
        """[IMPL] F-13.5 图片帧 → 同步轨迹位置指示器"""
        from PyQt5.QtGui import QPixmap
        pixmap = QPixmap(self.pic_player.frame_paths[idx])
        if self.pic_overlay:
            self.pic_overlay.set_frame(pixmap, idx, len(self.pic_player.frame_paths))
        # 按时间戳就近匹配轨迹节点，刷新地图当前位置指示
        if self.player and self.db_data:
            from core.frame_matcher import find_nearest_by_timestamp
            ts = self.pic_player.frame_timestamps[idx]
            node_idx = find_nearest_by_timestamp(
                ts, self.db_data['nodes'], lambda n: n['timestamp']
            )
            if node_idx >= 0:
                self.player.seek(node_idx)

    def _on_pic_speed_change(self, speed: float):
        """[IMPL] F-13.6 倍速联动：图片与轨迹播放器共享同一倍速值"""
        if self.pic_player:
            self.pic_player.set_speed(speed)
        if self.player and hasattr(self.player, 'set_speed'):
            self.player.set_speed(speed)
        # 持久化到 settings
        self._settings['pic_playback_speed'] = speed
        self._save_settings()

    def _on_node_hover_enter(self, node_id: int):
        """[IMPL] F-14.1/F-14.3/F-14.4 悬停弹出节点对应图片预览"""
        if not self.topology:
            return
        node = next((n for n in self.topology.waypoints if n['id'] == node_id), None)
        if not node:
            return
        # 确保图片数据已加载（不启动播放）
        self._ensure_pic_player()
        if self.pic_player:
            self.pic_player.pause()
        if not self.pic_player:
            return
        from core.frame_matcher import find_nearest_by_timestamp
        idx = find_nearest_by_timestamp(
            node['timestamp'],
            list(zip(self.pic_player.frame_paths, self.pic_player.frame_timestamps)),
            lambda pair: pair[1]
        )
        if idx < 0:
            return
        from PyQt5.QtGui import QPixmap
        from ui.pic_hover_preview import HoverPreview
        if self._hover_preview is None:
            # HoverPreview 为 Qt.ToolTip 顶层窗口，parent 用于生命周期归属
            self._hover_preview = HoverPreview(self)
        pixmap = QPixmap(self.pic_player.frame_paths[idx])
        self._hover_preview.set_frame(pixmap, idx, len(self.pic_player.frame_paths))
        # 定位到主窗口右下角，避免遮挡地图中心区域
        br_local = self.rect().bottomRight()
        br_global = self.mapToGlobal(br_local)
        pw, ph = self._hover_preview.width(), self._hover_preview.height()
        self._hover_preview.move(
            br_global.x() - pw - 20,
            br_global.y() - ph - 40,
        )
        self._hover_preview.show()

    def _on_node_hover_leave(self):
        """[IMPL] F-14.5 移开即消失"""
        if self._hover_preview:
            self._hover_preview.hide()

    # ─── 节点锁定 (F-4.1) ───────────────────────────────

    def lock_current_node(self):
        if not self.player or not self.db_data:
            return
        node = self.db_data['nodes'][self.player.current_idx]
        # [IMPL] F-15.1 传入轨迹索引 traj_idx
        wp = self.topology.add_waypoint(
            node['x'], node['y'], node['z'],
            node['yaw'], node['timestamp'],
            traj_idx=self.player.current_idx,
        )
        self.log(
            f'{wp["label"]} 锁定 ({node["x"]:.2f}, {node["y"]:.2f})',
            'warn'
        )
        # 刷新显示
        self.map_view.draw_waypoints(self.topology.waypoints)
        self._status_node.setText(
            f'节点: {len(self.topology.waypoints)} | '
            f'边: {len(self.topology.edges)}'
        )

    # ─── 工具模式切换 ──────────────────────────────────

    def _on_tool_change(self, tool: str):
        self.map_view._tool_mode = tool
        self.map_view._edge_start_id = None
        self._selected_nodes = []
        self.log(f'工具: {tool}', 'info')
        hints = {
            'pan': '拖拽平移 · 滚轮缩放',
            'node': '节点模式 · 点击选中节点',
            'edge': '连边模式 · 点击起点 → 点击终点',
            'plan': '规划模式 · 点击起点 → 点击终点',
            'delete_edge': '删边模式 · 点击边',
            'delete_node': '删节点模式 · 点击节点',
        }
        self._status_hint.setText(hints.get(tool, ''))

    def _set_tool(self, tool: str):
        """程序化设置工具模式（快捷键用）"""
        self.sidebar.tool_changed.emit(tool)

    # ─── 地图点击处理 ──────────────────────────────────

    def _on_map_click(self, wx: float, wy: float):
        tool = self.map_view._tool_mode

        if tool == 'node':
            # F-U2: 优先吸附轨迹 0.5m 内的最近点
            snapped = self._snap_to_trajectory(wx, wy, threshold=0.5)
            if snapped:
                # [IMPL] F-15.1 传入轨迹索引
                wp = self.topology.add_waypoint(
                    snapped['x'], snapped['y'], snapped['z'],
                    snapped['yaw'], snapped['timestamp'],
                    traj_idx=snapped.get('traj_idx', -1),
                )
                self.log(
                    f'{wp["label"]} 吸附轨迹点 ({snapped["x"]:.2f}, '
                    f'{snapped["y"]:.2f})', 'warn'
                )
                self.map_view.draw_waypoints(self.topology.waypoints)
            else:
                # 未吸附到轨迹则查找已有节点
                nearest = self._find_nearest_waypoint(wx, wy, threshold=1.0)
                if nearest:
                    self.log(f'选中 {nearest["label"]}', 'info')

        elif tool == 'edge':
            nearest = self._find_nearest_waypoint(wx, wy, threshold=1.0)
            if nearest:
                if self.map_view._edge_start_id is None:
                    self.map_view._edge_start_id = nearest['id']
                    self.log(f'连边起点: {nearest["label"]}', 'info')
                else:
                    from_id = self.map_view._edge_start_id
                    to_id = nearest['id']
                    if from_id != to_id:
                        # [IMPL] F-15.3 传入 db_nodes 以提取轨迹段
                        edge = self.topology.add_edge(
                            from_id, to_id,
                            db_nodes=self.db_data['nodes'] if self.db_data else None,
                        )
                        self.log(
                            f'添加边: WP-{from_id:02d} → WP-{to_id:02d} '
                            f'({edge["length"]:.2f}m)', 'warn'
                        )
                        self.map_view.draw_edges(
                            self.topology.edges, self.topology.waypoints,
                            topology=self.topology,  # [IMPL] F-15.12
                        )
                    self.map_view._edge_start_id = None

        elif tool == 'plan':
            nearest = self._find_nearest_waypoint(wx, wy, threshold=1.0)
            if nearest:
                self._selected_nodes.append(nearest)
                if len(self._selected_nodes) == 1:
                    self.log(f'规划起点: {nearest["label"]}', 'info')
                elif len(self._selected_nodes) == 2:
                    start = self._selected_nodes[0]
                    end = self._selected_nodes[1]
                    self.log(
                        f'规划: {start["label"]} → {end["label"]}', 'info'
                    )
                    G = build_graph(
                        self.topology.waypoints, self.topology.edges
                    )
                    path_ids, total_len = find_path(G, start['id'], end['id'])
                    if path_ids:
                        wp_map = {w['id']: w for w in
                                  self.topology.waypoints}
                        path_nodes = [wp_map[pid] for pid in path_ids]
                        self._status_path.setText(
                            f'路径: {total_len:.2f}m ({len(path_ids)} 节点)'
                        )
                        self.log(
                            f'路径找到: {total_len:.2f}m, '
                            f'{len(path_ids)} 个节点', 'info'
                        )
                        # [IMPL] F-15.14 沿轨迹边拼接生成密集路径，替换原直线插值
                        traj = concat_trajectory_segments(
                            path_nodes, self.topology.edges, self.topology
                        )
                        # [IMPL] F-15.15 用密集轨迹点绘制规划路径折线；无轨迹时回退稀疏节点
                        self.map_view.draw_planned_path(
                            traj if traj else path_nodes
                        )
                        # [IMPL] F-12.6 操作模式路径拼接：robot → wp_a 直线段
                        if self._is_operation_mode() and self._current_robot_pose is not None:
                            rx = self._current_robot_pose['x']
                            ry = self._current_robot_pose['y']
                            straight_nodes = _interpolate_line(
                                rx, ry, start['x'], start['y'], step=0.05
                            )
                            if straight_nodes:
                                # 直线段不含 timestamp，使用 traj[0] 的 timestamp 填充
                                t0 = traj[0]['timestamp'] if traj else 0.0
                                for pt in straight_nodes:
                                    pt['timestamp'] = t0
                                traj = straight_nodes + traj
                        # 侧栏 UDP 预览：新规划清除旧内容，写入摘要
                        self.sidebar.clear_udp_preview()
                        self.sidebar.append_udp_info(
                            f'{start["label"]} → {end["label"]}  '
                            f'{total_len:.2f}m  {len(traj)}点  '
                            f'步长约{self._calc_traj_step(traj):.2f}m',
                            '#1d9e75'
                        )
                        self.sidebar.append_udp_info(
                            f'目标 {self._udp_ip}:{self._udp_port}  '
                            f'间隔约{self._calc_traj_dt(traj):.3f}s',
                            '#888780'
                        )
                        self._prepare_mission_route(path_ids)
                        if self._authority_mode == AuthorityMode.LEGACY.value:
                            self._start_udp_send(traj)
                        else:
                            self._stop_udp_send()
                            self.log('mission_nav2: UDP disabled; dense /plan_nav is display-only', 'info')
                        # [IMPL] F-16.1 操作模式下更新 /plan_nav 话题路径
                        if self._is_operation_mode() and self._plan_publisher:
                            self._plan_publisher.set_path(traj)
                            self.log(f'/plan_nav 路径已更新: {len(traj)} 个轨迹点', 'info')
                        self.log(
                            f'UDP 轨迹: {len(traj)} 个插值点 '
                            f'(步长约 {self._calc_traj_step(traj):.2f}m)', 'info'
                        )
                    else:
                        self.log('无可行路径', 'error')
                    self._selected_nodes = []

        elif tool == 'delete_edge':
            nearest_edge = self._find_nearest_edge(wx, wy, threshold=1.5)
            if nearest_edge:
                e = nearest_edge
                count = self.topology.remove_edge_bidirectional(
                    e['from_id'], e['to_id']
                )
                if count == 2:
                    self.log(
                        f'删除双向边: WP-{e["from_id"]:02d} ↔ '
                        f'WP-{e["to_id"]:02d}', 'warn'
                    )
                else:
                    self.log(
                        f'删除边: WP-{e["from_id"]:02d} → '
                        f'WP-{e["to_id"]:02d}', 'warn'
                    )
                self.map_view.draw_edges(
                    self.topology.edges, self.topology.waypoints, topology=self.topology  # [IMPL] F-15.12
                )

        elif tool == 'delete_node':
            nearest = self._find_nearest_waypoint(wx, wy, threshold=1.0)
            if nearest:
                label = self.topology.remove_waypoint(nearest['id'])
                if label:
                    self.log(f'删除节点: {label}', 'warn')
                    self.map_view.clear_waypoints()
                    self.map_view.clear_edges()
                    self.map_view.clear_planned_path()
                    self.map_view.draw_waypoints(self.topology.waypoints)
                    self.map_view.draw_edges(
                        self.topology.edges, self.topology.waypoints, topology=self.topology  # [IMPL] F-15.12
                    )
                    self._status_path.setText('路径: --')

        self._status_node.setText(
            f'节点: {len(self.topology.waypoints)} | '
            f'边: {len(self.topology.edges)}'
        )

    # ─── 节点注释编辑 (F-17.2 F-17.4 F-17.5 F-17.6) ─────

    def _on_annotation_request(self, node_id: int):
        """
        [IMPL] F-17.2 弹出 QInputDialog 编辑节点注释。
        OK → 设置注释并刷新地图
        Cancel → 保持原值 (F-17.5)
        空字符串 → 清空注释 (F-17.4)
        预填当前值 (F-17.6)
        """
        if not self.topology:
            return

        wp = next((w for w in self.topology.waypoints
                   if w['id'] == node_id), None)
        if not wp:
            return

        from PyQt5.QtWidgets import QInputDialog
        current = wp.get('annotation', '')
        title = f'编辑节点注释 — {wp["label"]}'
        label_text = f'请输入 {wp["label"]} 的注释文字（留空清除）：'

        # 用 QInputDialog 实例替代静态方法，手动定位到左上角避免与图片悬浮窗重叠
        # [FIX] 2026-07-04 先隐藏 PicOverlay，避免 Linux 窗口管理器下对话框被遮挡
        pic_was_visible = self.pic_overlay and self.pic_overlay.isVisible()
        if pic_was_visible:
            self.pic_overlay.hide()

        dlg = QInputDialog(self)
        dlg.setWindowTitle(title)
        dlg.setLabelText(label_text)
        dlg.setTextValue(current)
        dlg.setInputMode(QInputDialog.TextInput)
        # 定位到主窗口左上角（避开右上角的图片悬浮窗）
        top_left = self.mapToGlobal(self.rect().topLeft())
        dlg.move(top_left.x() + 40, top_left.y() + 100)
        ok = dlg.exec_()

        if pic_was_visible and self.pic_overlay:
            self.pic_overlay.show()
        text = dlg.textValue() if ok else ''
        # [IMPL] F-17.5 取消时保持原值不变
        if ok:
            old_val = wp.get('annotation', '')
            new_val = text.strip()
            if new_val != old_val:
                self.topology.set_annotation(node_id, new_val)
                self.map_view.draw_waypoints(self.topology.waypoints)
                if new_val:
                    self.log(f'{wp["label"]} 注释: "{new_val}"', 'info')
                else:
                    # [IMPL] F-17.4 空注释清除
                    self.log(f'{wp["label"]} 注释已清除', 'info')

    def _find_nearest_waypoint(self, wx: float, wy: float,
                                threshold: float = 1.0) -> dict | None:
        """查找距离点击位置最近的节点"""
        if not self.topology:
            return None
        best, best_dist = None, threshold
        for wp in self.topology.waypoints:
            d = ((wp['x'] - wx) ** 2 + (wp['y'] - wy) ** 2) ** 0.5
            if d < best_dist:
                best_dist = d
                best = wp
        return best

    def _find_nearest_edge(self, wx: float, wy: float,
                            threshold: float = 1.5) -> dict | None:
        """查找距离点击位置最近的边"""
        if not self.topology:
            return None
        wp_map = {w['id']: w for w in self.topology.waypoints}
        best, best_dist = None, threshold
        for e in self.topology.edges:
            a = wp_map.get(e['from_id'])
            b = wp_map.get(e['to_id'])
            if not a or not b:
                continue
            mx, my = (a['x'] + b['x']) / 2, (a['y'] + b['y']) / 2
            d = ((mx - wx) ** 2 + (my - wy) ** 2) ** 0.5
            if d < best_dist:
                best_dist = d
                best = e
        return best

    # ─── 撤销操作 (F-U1) ──────────────────────────────

    def _undo(self):
        """Ctrl+Z 撤销最近一次节点/边操作"""
        if not self.topology:
            return
        desc = self.topology.undo()
        if desc:
            self.log(desc, 'warn')
            # 先清空所有叠加层，再完整重绘
            self.map_view.clear_overlay()
            if self.db_data and self.player:
                idx = self.player.current_idx
                self.map_view.draw_trajectory(self.db_data['nodes'], idx)
                self.map_view.draw_robot(
                    self.db_data['nodes'][idx]['x'],
                    self.db_data['nodes'][idx]['y'],
                    self.db_data['nodes'][idx]['yaw'],
                )
            self.map_view.draw_waypoints(self.topology.waypoints)
            self.map_view.draw_edges(
                self.topology.edges, self.topology.waypoints, topology=self.topology  # [IMPL] F-15.12
            )
            self._status_node.setText(
                f'节点: {len(self.topology.waypoints)} | '
                f'边: {len(self.topology.edges)}'
            )
        else:
            self.log('无可撤销操作', 'info')

    # ─── 复原操作 ─────────────────────────────────────

    def _on_reset(self):
        """清空所有语义节点、拓扑边、规划路径及 txt 文件"""
        if not self.topology:
            return
        self.topology.reset_all()
        self.map_view.clear_waypoints()
        self.map_view.clear_edges()
        self.map_view.clear_planned_path()
        self.sidebar.clear_udp_preview()
        self._status_path.setText('路径: --')
        self._status_node.setText(
            f'节点: {len(self.topology.waypoints)} | '
            f'边: {len(self.topology.edges)}'
        )
        self.log('已清空所有节点、边和规划路径', 'warn')

    # ─── 轨迹吸附 (F-U2) ──────────────────────────────

    def _snap_to_trajectory(self, wx: float, wy: float,
                            threshold: float = 0.5) -> dict | None:
        """
        [IMPL] F-15.1 查找轨迹上距点击位置最近的节点，threshold 米内吸附。
        返回值附加 traj_idx 字段（在 db_data['nodes'] 中的索引）。
        """
        if not self.db_data:
            return None
        best, best_idx, best_dist = None, -1, threshold
        for i, node in enumerate(self.db_data['nodes']):
            d = ((node['x'] - wx) ** 2 + (node['y'] - wy) ** 2) ** 0.5
            if d < best_dist:
                best_dist = d
                best = node
                best_idx = i
        if best:
            result = dict(best)
            result['traj_idx'] = best_idx  # [IMPL] F-15.1 附加轨迹索引
            return result
        return None

    # ─── 轨迹统计辅助 ─────────────────────────────────────

    def _calc_traj_step(self, traj: list) -> float:
        """计算轨迹相邻点平均间距（米）"""
        if len(traj) < 2:
            return 0.0
        import math
        total = sum(
            math.hypot(traj[i]['x'] - traj[i-1]['x'],
                       traj[i]['y'] - traj[i-1]['y'])
            for i in range(1, len(traj))
        )
        return total / (len(traj) - 1)

    def _calc_traj_dt(self, traj: list) -> float:
        """计算轨迹相邻点平均时间间隔（秒），无法计算时返回 0"""
        if len(traj) < 2:
            return 0.0
        dts = [
            traj[i]['timestamp'] - traj[i-1]['timestamp']
            for i in range(1, len(traj))
            if traj[i]['timestamp'] > 0 and traj[i-1]['timestamp'] > 0
        ]
        return sum(dts) / len(dts) if dts else 0.0

    # ─── 操作模式切换 (F-12.6 / F-12.7) ─────────────────────

    def _on_mode_changed(self, mode: str):
        """处理模式切换：通知子组件 + 启停 PoseReceiver"""
        op = (mode == 'op')
        # 1. 通知各子组件切换 UI 状态
        self.sidebar.set_operation_mode(op)
        self.log_panel.set_operation_mode(op)
        self.map_view.set_operation_mode(op)

        if op:
            # 2. 操作模式：初始化 🚗 图元（需 map 已加载）
            if self.map_view.map_meta:
                self.map_view.init_car_indicator()
            # 3. 启动 ROS2 位姿接收线程
            self._start_pose_receiver()
            # 4. 启动 /plan 发布线程
            self._start_plan_publisher()
            if self._authority_mode == AuthorityMode.MISSION_NAV2.value:
                self._start_mission_bridge()
        else:
            self._stop_mission_bridge()
            # 4. 停止 /plan 发布线程
            self._stop_plan_publisher()
            # 5. 调试模式：停止位姿接收线程
            self._stop_pose_receiver()

        self.log(f'已切换到{"操作" if op else "调试"}模式', 'info')

    def _start_pose_receiver(self):
        """启动 ROS2 位姿接收线程"""
        from core.pose_receiver import PoseReceiver
        self._stop_pose_receiver()  # 防止重复启动
        self._pose_receiver = PoseReceiver(self)
        self._pose_receiver.pose_updated.connect(self._on_pose_updated)
        self._pose_receiver.error_occurred.connect(self._on_pose_error)
        self._pose_receiver.connected.connect(
            lambda msg: self.log(msg, 'info')
        )
        self._pose_receiver.start()
        import time
        self._last_pose_time = time.time()
        self._pose_timeout_timer.start()

    def _stop_pose_receiver(self):
        """停止 ROS2 位姿接收线程"""
        self._pose_timeout_timer.stop()
        if self._pose_receiver and self._pose_receiver.isRunning():
            self._pose_receiver.stop()
            self._pose_receiver = None

    def _start_plan_publisher(self):
        """启动 /plan_nav 话题发布线程"""
        from core.nav_publisher import PlanPublisher
        self._stop_plan_publisher()
        self._plan_publisher = PlanPublisher(self)
        self._plan_publisher.log_message.connect(self.log)
        self._plan_publisher.connected.connect(lambda msg: self.log(msg, 'info'))
        self._plan_publisher.start()

    def _stop_plan_publisher(self):
        """停止 /plan_nav 话题发布线程"""
        if self._plan_publisher and self._plan_publisher.isRunning():
            self._plan_publisher.stop()
            self._plan_publisher = None

    def _on_pose_updated(self, pose: dict):
        """主线程：收到新位姿，更新 🚗 和状态灯"""
        import time
        self._last_pose_time = time.time()
        self._current_robot_pose = pose  # 缓存实时位姿，路径规划时使用
        self.map_view.update_car_pose(pose['x'], pose['y'], pose['yaw'])
        self.log_panel.set_pose_status(True)
        # 同步更新 PlanPublisher 的当前位置
        if self._plan_publisher and self._plan_publisher.isRunning():
            self._plan_publisher.update_pose(
                pose['x'], pose['y'], pose['z'], pose['yaw']
            )

    def _on_pose_error(self, msg: str):
        """主线程：ROS2 错误弹窗"""
        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.critical(self, 'ROS2 连接错误',
                             '系统内没有检测到ROS2版本或无法正确收取到当前位姿。')
        self.log(msg, 'error')

    def _check_pose_timeout(self):
        """每秒检查一次：超过 1 秒无 TF 数据则亮红灯"""
        import time
        elapsed = time.time() - self._last_pose_time
        self.log_panel.set_pose_status(elapsed < 1.0)

    def _is_operation_mode(self) -> bool:
        """返回当前是否处于操作模式"""
        return self.sidebar._mode_btn.current_mode == 'op'

    def _on_authority_mode_changed(self, mode: str):
        self._authority_mode = mode if mode in {m.value for m in AuthorityMode} else AuthorityMode.LEGACY.value
        if self._authority_mode == AuthorityMode.MISSION_NAV2.value:
            self._stop_udp_send()
            if self._is_operation_mode():
                self._start_mission_bridge()
        else:
            self._stop_mission_bridge()
        self._refresh_mission_controls()
        self.sidebar.set_mission_status(f'mode: {self._authority_mode}')
        self.log(f'authority mode: {self._authority_mode}', 'info')

    def _prepare_mission_route(self, path_ids: list[int]):
        self._pending_route_spec = None
        self._published_route_spec = None
        self._active_mission_id = ''
        self._active_route_id = ''
        if not self.topology:
            self._set_mission_rejection('TOPOLOGY_NOT_LOADED', 'topology is not loaded')
            return
        topology_work_dir = Path(os.path.abspath(self.topology.work_dir))
        nodes, edges, _legacy = load_topology(topology_work_dir)
        manifest = read_manifest(topology_work_dir)
        if manifest is None:
            self._set_mission_rejection('TOPOLOGY_MANIFEST_MISSING', 'topology_manifest.json missing')
            return
        result = build_sparse_route_spec(
            mission_id=f'mission-{uuid.uuid4().hex}',
            ordered_node_ids=[int(node_id) for node_id in path_ids],
            nodes=nodes,
            edges=edges,
            topology_manifest=manifest,
        )
        if not result.valid:
            self._set_mission_rejection(result.reason_code, result.detail)
            return
        self._pending_route_spec = result.route
        self.sidebar.set_mission_status(
            f'mode: {self._authority_mode}\n'
            f'mission ready\n'
            f'route: {result.route.route_id}\n'
            f'topology: {result.route.topology_version}'
        )
        self._refresh_mission_controls()

    def _set_mission_rejection(self, reason: str, detail: str):
        self.sidebar.set_mission_status(f'mode: {self._authority_mode}\n{reason}\n{detail}')
        self.log(f'mission route rejected: {reason} ({detail})', 'error')
        self._refresh_mission_controls()

    def _start_mission_bridge(self):
        if self._mission_bridge and self._mission_bridge.isRunning():
            return
        self._mission_bridge = MissionBridgeThread(self)
        self._mission_bridge.mission_state_received.connect(self._on_mission_state_received)
        self._mission_bridge.service_result_received.connect(self._on_mission_service_result)
        self._mission_bridge.connected.connect(lambda msg: self.log(msg, 'info'))
        self._mission_bridge.error_occurred.connect(lambda msg: self.log(msg, 'error'))
        self._mission_bridge.start()

    def _stop_mission_bridge(self):
        if self._mission_bridge and self._mission_bridge.isRunning():
            self._mission_bridge.stop()
        self._mission_bridge = None

    def _on_publish_mission_requested(self):
        if self._authority_mode != AuthorityMode.MISSION_NAV2.value:
            self._set_mission_rejection('MISSION_MODE_DISABLED', 'select mission_nav2 mode first')
            return
        if self._mission_active():
            self._set_mission_rejection('MISSION_ALREADY_ACTIVE', 'wait for terminal mission state before publishing another mission')
            return
        if self._pending_route_spec is None:
            self._set_mission_rejection('NO_TOPOLOGICAL_ROUTE', 'select a valid route first')
            return
        ok, reason, detail = verify_route_topology_current(self.topology.work_dir, self._pending_route_spec)
        if not ok:
            self._set_mission_rejection(reason, detail)
            return
        self._start_mission_bridge()
        if not self._mission_bridge or not self._mission_bridge.publish_route(self._pending_route_spec):
            self._set_mission_rejection('MISSION_BRIDGE_UNAVAILABLE', 'mission bridge is not ready')
            return
        self._published_route_spec = self._pending_route_spec
        self._active_mission_id = self._pending_route_spec.mission_id
        self._active_route_id = self._pending_route_spec.route_id
        self.sidebar.set_mission_status(
            f'mode: {self._authority_mode}\n'
            f'published: {self._active_mission_id}\n'
            f'route: {self._active_route_id}\n'
            'waiting for RECEIVED'
        )
        self._refresh_mission_controls()

    def _on_start_mission_requested(self):
        if self._start_enabled() and self._mission_bridge:
            self._mission_bridge.request_start()

    def _on_cancel_mission_requested(self):
        if self._mission_bridge:
            self._mission_bridge.request_cancel()

    def _on_pause_mission_requested(self):
        if self._mission_bridge:
            self._mission_bridge.request_pause()

    def _on_resume_mission_requested(self):
        if self._mission_bridge:
            self._mission_bridge.request_resume()

    def _on_mission_service_result(self, name: str, success: bool, reason: str, message: str):
        self.log(f'/mission/{name}: {reason} success={success} {message}', 'info' if success else 'warn')

    def _on_mission_state_received(self, msg):
        if self._active_mission_id and msg.mission_id and msg.mission_id != self._active_mission_id:
            return
        if self._active_route_id and msg.route_id and msg.route_id != self._active_route_id:
            return
        self._latest_mission_state = msg
        state_name = self._mission_state_name(int(msg.state))
        self.sidebar.set_mission_status(
            f'mode: {self._authority_mode}\n'
            f'mission: {msg.mission_id}\n'
            f'route: {msg.route_id}\n'
            f'state: {state_name}\n'
            f'wp: {msg.current_waypoint_index}  done: {msg.completed_waypoint_count}/{msg.total_waypoint_count}\n'
            f'progress: {float(msg.progress):.2f}\n'
            f'{msg.reason_code} {msg.detail}'
        )
        self._refresh_mission_controls()

    def _mission_state_name(self, state: int) -> str:
        names = {
            0: 'IDLE',
            1: 'RECEIVED',
            2: 'VALIDATING',
            3: 'PLANNING',
            4: 'NAVIGATING',
            5: 'PAUSED',
            6: 'CANCELLING',
            7: 'CANCELLED',
            8: 'SUCCEEDED',
            9: 'TEMPORARILY_BLOCKED',
            10: 'BLOCKED',
            11: 'FAILED',
            12: 'HELP_REQUIRED',
        }
        return names.get(state, f'STATE_{state}')

    def _mission_active(self) -> bool:
        if self._latest_mission_state is None:
            return False
        if not self._active_mission_id:
            return False
        return int(self._latest_mission_state.state) not in TERMINAL_MISSION_STATES

    def _start_enabled(self) -> bool:
        msg = self._latest_mission_state
        return (
            self._authority_mode == AuthorityMode.MISSION_NAV2.value
            and msg is not None
            and msg.mission_id == self._active_mission_id
            and msg.route_id == self._active_route_id
            and int(msg.state) == 1
        )

    def _refresh_mission_controls(self):
        route_ready = self._pending_route_spec is not None and not self._mission_active()
        self.sidebar.set_mission_controls(route_ready, self._start_enabled())

    # ─── 配置变更 (F-8.2) ──────────────────────────────

    def _on_config_change(self, ip: str, port: int,
                          yes_conf: float, axis_tolerance: float):
        self._udp_ip = ip
        self._udp_port = port
        self._yes_conf = yes_conf
        self._axis_tolerance = axis_tolerance

    # ─── 轨迹参数变更 (F-11.8 F-11.9) ──────────────────

    def _on_traj_params_change(self, r_min, r_max, dt, vs, vt):
        """轨迹参数变更：内存更新 + 持久化 + 热生效"""
        self._pursuit_r_min = r_min
        self._pursuit_r_max = r_max
        self._pursuit_delta_thresh = dt
        self._pursuit_v_straight = vs
        self._pursuit_v_turn = vt
        # 持久化
        self._save_pursuit_settings()
        # 热生效：若 UdpSender 正在运行，立即更新参数
        if self.udp_sender and self.udp_sender.isRunning():
            self.udp_sender.update_params(r_min, r_max, dt, vs, vt)
            self.log('轨迹参数热生效', 'info')

    def _save_pursuit_settings(self):
        """读取 settings.json → 合并 pursuit_* → 写回"""
        path = os.path.join(self._project_root(), 'config', 'settings.json')
        try:
            with open(path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except Exception:
            cfg = {}
        cfg.update({
            'pursuit_r_min': self._pursuit_r_min,
            'pursuit_r_max': self._pursuit_r_max,
            'pursuit_delta_thresh': self._pursuit_delta_thresh,
            'pursuit_v_straight': self._pursuit_v_straight,
            'pursuit_v_turn': self._pursuit_v_turn,
        })
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)

    def _save_settings(self):
        """[IMPL] F-13.x 合并写回 settings.json，不覆盖已有字段"""
        import json
        path = os.path.join(self._project_root(), 'config', 'settings.json')
        try:
            with open(path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except Exception:
            cfg = {}
        cfg.update(self._settings)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)

    # ─── UDP 发送 (F-7.1 ~ F-7.3) ─────────────────────

    def _start_udp_send(self, path_nodes: list):
        """启动 UDP 逐帧发送路径点（Pure Pursuit F-11.1~F-11.5）"""
        if self._authority_mode == AuthorityMode.MISSION_NAV2.value:
            self.log('mission_nav2: UDP send blocked by authority mode', 'warn')
            return
        self._stop_udp_send()  # 先停掉旧线程
        self.udp_sender = UdpSender(
            path_nodes,
            ip=self._udp_ip,
            port=self._udp_port,
            r_min=self._pursuit_r_min,
            r_max=self._pursuit_r_max,
            delta_thresh=self._pursuit_delta_thresh,
            v_straight=self._pursuit_v_straight,
            v_turn=self._pursuit_v_turn,
        )
        self.udp_sender.frame_sent.connect(
            lambda idx: self.statusBar().showMessage(
                f'UDP 发送中  帧 {idx}/{len(path_nodes)}  '
                f'目标 {self._udp_ip}:{self._udp_port}'
            )
        )
        self.udp_sender.finished.connect(self._on_udp_finished)
        self.udp_sender.start()

    def _stop_udp_send(self):
        """停止并清理 UDP 发送线程"""
        if self.udp_sender and self.udp_sender.isRunning():
            self.udp_sender._running = False
            self.udp_sender.wait(2000)
        self.udp_sender = None

    def _on_udp_finished(self):
        """UDP 发送完成回调"""
        self.statusBar().showMessage('UDP 路径发送完毕', 3000)
        self.udp_sender = None

    def closeEvent(self, event):
        self._stop_mission_bridge()
        self._stop_plan_publisher()
        self._stop_pose_receiver()
        self._stop_udp_send()
        try:
            from core.ros_runtime import shutdown_rclpy_once
            shutdown_rclpy_once()
        except Exception:
            pass
        super().closeEvent(event)
