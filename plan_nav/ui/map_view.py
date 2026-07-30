# [DONE] F-2.1 2D底图显示 (QGraphicsView + QGraphicsPixmapItem)
# [DONE] F-2.2 坐标系转换 (世界坐标 ↔ 像素坐标)
# [DONE] F-2.3 视角控制 (拖拽平移 + 滚轮缩放 + 自动fitInView)
# [IMPL] F-15.12 边折线绘制 + F-15.13 距离标注 + F-15.15 规划路径折线
# [IMPL] F-17.1/F-17.3/F-17.9 节点注释触发 + 注释渲染

from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from core.ui_font import mono_font  # [ADAPT-UBU-02] 跨平台等宽字体
from PyQt5.QtCore import Qt, QPointF, pyqtSignal
from PyQt5.QtGui import (
    QPixmap, QImage, QPainter, QPen, QBrush, QColor,
    QPolygonF, QFont, QTransform, QPainterPath,  # [IMPL] F-15.12 QPainterPath 用于折线绘制
)
from core.map_decoder import world_to_pixel, pixel_to_world
from core.ui_font import mono_family  # [ADAPT-UBU-02] 字体族探测
import numpy as np
import math


class MapView(QGraphicsView):
    """2D 地图视口：渲染底图、轨迹、节点、边、规划路径"""

    world_clicked = pyqtSignal(float, float)  # 鼠标点击的世界坐标
    # [IMPL] F-14.1/F-14.2/F-14.5 节点悬停信号
    node_hover_enter = pyqtSignal(int)  # node_id
    node_hover_leave = pyqtSignal()
    # [IMPL] F-17.1 平移模式下点击节点 → 注释编辑信号
    annotation_requested = pyqtSignal(int)  # node_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(
            QGraphicsView.AnchorUnderMouse
        )
        self.setBackgroundBrush(QBrush(QColor('#f0eeea')))

        self.map_meta = None
        self._map_item = None
        self._overlay_items = []
        self._waypoint_items = []      # 语义节点独立跟踪
        self._edge_items = []          # 拓扑边独立跟踪
        self._planned_path_items = []  # F-U3 规划路径独立跟踪
        self._nav_indicator = None      # UDP 发送当前位置红球
        self._car_item = None       # [IMPL] F-12.5 操作模式下的 CarIndicatorItem
        self._op_mode  = False      # [IMPL] F-12.5 当前是否在操作模式

        # 交互模式
        self._tool_mode = 'pan'  # 'pan' | 'node' | 'edge' | 'plan' | 'delete_edge'
        self._edge_start_id = None  # 连边起点节点ID
        # [IMPL] F-14.1/F-14.2/F-14.5 节点悬停检测
        self.setMouseTracking(True)
        self._hover_node_id = None
        self._waypoint_geoms = {}   # node_id → (px, py)，draw_waypoints 时同步写入

    # ─── 底图加载 (F-2.1) ───────────────────────────────

    def load_map(self, map_dict: dict):
        """加载/替换底图"""
        self.map_meta = map_dict
        rgb = map_dict['image_rgb']
        h, w = rgb.shape[:2]
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        if self._map_item:
            self._scene.removeItem(self._map_item)
        self._map_item = QGraphicsPixmapItem(pixmap)
        self._scene.addItem(self._map_item)
        self._scene.setSceneRect(0, 0, w, h)
        self.fitInView(
            self._scene.sceneRect(), Qt.KeepAspectRatio
        )

    def setup_trajectory_view(self, map_meta: dict, nodes: list):
        """仅设置坐标系统（无底图），基于轨迹点包围盒计算场景范围"""
        self.map_meta = map_meta
        xs, ys = [], []
        for n in nodes:
            px, py = self._w2p(n['x'], n['y'])
            xs.append(px)
            ys.append(py)
        if xs:
            margin = 120
            self._scene.setSceneRect(
                min(xs) - margin, min(ys) - margin,
                max(xs) - min(xs) + 2 * margin,
                max(ys) - min(ys) + 2 * margin,
            )
            self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)

    # ─── 视角控制 (F-2.3) ───────────────────────────────

    def wheelEvent(self, event):
        """滚轮缩放，锚点跟随鼠标"""
        factor = 1.15 if event.angleDelta().y() > 0 else 0.87
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        """鼠标点击 → 根据工具模式处理"""
        if event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            if self.map_meta:
                # [IMPL] F-17.1 平移模式下点击节点 → 发射注释信号
                if self._tool_mode == 'pan':
                    hit_id = self._hit_test_waypoint(scene_pos)
                    if hit_id is not None:
                        self.annotation_requested.emit(hit_id)
                        return  # 不触发 world_clicked
                wx, wy = pixel_to_world(
                    int(scene_pos.x()), int(scene_pos.y()), self.map_meta
                )
                self.world_clicked.emit(wx, wy)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """[IMPL] F-14.1/F-14.2/F-14.5 节点悬停检测：仅非连边/非规划模式生效"""
        if self._tool_mode in ('edge', 'plan'):
            if self._hover_node_id is not None:
                self._hover_node_id = None
                self.node_hover_leave.emit()
            super().mouseMoveEvent(event)
            return

        scene_pos = self.mapToScene(event.pos())
        hit_id = None
        HIT_RADIUS = 8  # 像素，与节点绘制半径一致
        for node_id, (px, py) in self._waypoint_geoms.items():
            if (scene_pos.x() - px) ** 2 + (scene_pos.y() - py) ** 2 <= HIT_RADIUS ** 2:
                hit_id = node_id
                break

        if hit_id != self._hover_node_id:
            if hit_id is None:
                self.node_hover_leave.emit()
            else:
                self.node_hover_enter.emit(hit_id)
            self._hover_node_id = hit_id

        super().mouseMoveEvent(event)

    # ─── 坐标转换 (F-2.2) ───────────────────────────────

    def _w2p(self, wx, wy):
        """世界坐标 → 像素坐标（快捷方法）"""
        return world_to_pixel(wx, wy, self.map_meta)

    # ─── 轨迹绘制 ───────────────────────────────────────

    def clear_overlay(self):
        """清除所有叠加图元"""
        for item in self._overlay_items:
            self._scene.removeItem(item)
        self._overlay_items.clear()
        self.clear_waypoints()
        self.clear_edges()
        self.clear_planned_path()

    def clear_waypoints(self):
        """清除语义节点图层"""
        for item in self._waypoint_items:
            self._scene.removeItem(item)
        self._waypoint_items.clear()

    def clear_edges(self):
        """清除拓扑边图层"""
        for item in self._edge_items:
            self._scene.removeItem(item)
        self._edge_items.clear()

    def clear_planned_path(self):
        """清除规划路径叠加层 (F-U3)"""
        for item in self._planned_path_items:
            self._scene.removeItem(item)
        self._planned_path_items.clear()
        self.clear_nav_indicator()

    def draw_nav_indicator(self, wx: float, wy: float):
        """在规划路径上绘制红色实心圆，标记 UDP 当前发送位置"""
        if self._op_mode:
            return   # [IMPL] F-12.5 操作模式下不显示红球
        self.clear_nav_indicator()
        # 仅当有规划路径时才显示
        if not self._planned_path_items:
            return
        px, py = self._w2p(wx, wy)
        r = 5
        self._nav_indicator = self._scene.addEllipse(
            px - r, py - r, r * 2, r * 2,
            QPen(QColor('#c0392b'), 2.0),
            QBrush(QColor('#e24b4a')),
        )

    def clear_nav_indicator(self):
        """清除导航红球"""
        if self._nav_indicator:
            self._scene.removeItem(self._nav_indicator)
            self._nav_indicator = None

    # [IMPL] F-12.5 操作模式管理
    def set_operation_mode(self, op: bool):
        """切换操作模式。op=True 时标记；op=False 时移除 🚗 图元"""
        self._op_mode = op
        if not op:
            if self._car_item:
                self._scene.removeItem(self._car_item)
                self._car_item = None

    def init_car_indicator(self):
        """操作模式启动时调用，创建 CarIndicatorItem 并加入场景（需 map_meta 已加载）"""
        from core.car_indicator import create_car_item
        if self._car_item:
            self._scene.removeItem(self._car_item)
        self._car_item = create_car_item()
        self._scene.addItem(self._car_item)

    def update_car_pose(self, wx: float, wy: float, yaw_rad: float):
        """操作模式下，每次收到 TF 数据时调用，更新 🚗 位置和朝向"""
        if not self._car_item or not self.map_meta:
            return
        from core.car_indicator import yaw_to_rotation
        px, py = self._w2p(wx, wy)
        self._car_item.set_pose(px, py, yaw_to_rotation(yaw_rad))

    def draw_trajectory(self, nodes: list, current_idx: int):
        """绘制已走（绿 #1d9e75）和未走（灰半透明）轨迹 (F-3.3)"""
        self.clear_overlay()
        for i in range(1, len(nodes)):
            x0, y0 = self._w2p(nodes[i-1]['x'], nodes[i-1]['y'])
            x1, y1 = self._w2p(nodes[i]['x'], nodes[i]['y'])
            if i <= current_idx:
                pen = QPen(QColor('#1d9e75'), 2.0)
            else:
                pen = QPen(QColor(170, 168, 160, 140), 1.2)
            item = self._scene.addLine(x0, y0, x1, y1, pen)
            self._overlay_items.append(item)

    # ─── 机器人绘制 (F-3.2) ──────────────────────────────

    def draw_robot(self, wx: float, wy: float, yaw: float):
        """红色方块图标，随YAW旋转"""
        px, py = self._w2p(wx, wy)
        s = 7
        item = self._scene.addRect(
            -s, -s, s * 2, s * 2,
            QPen(QColor('#c0392b'), 1.5),
            QBrush(QColor('#e24b4a')),
        )
        item.setPos(px, py)
        item.setRotation(-math.degrees(yaw))
        self._overlay_items.append(item)

    # ─── 语义节点绘制 (F-4.1) ────────────────────────────

    def draw_waypoints(self, waypoints: list):
        """橙色圆圈 + WP-XX 标签 + 注释文字（若存在）(F-17.3 F-17.9)"""
        self.clear_waypoints()
        self._waypoint_geoms.clear()
        for wp in waypoints:
            px, py = self._w2p(wp['x'], wp['y'])
            self._waypoint_geoms[wp['id']] = (px, py)

            # [IMPL] F-17.3 注释文字渲染：红色加粗，比标签大 2pt，显示在标签上方
            ann = wp.get('annotation', '')
            if ann:
                ann_item = self._scene.addText(ann)
                ann_item.setDefaultTextColor(QColor('#e24b4a'))
                ann_font = QFont(mono_family(), 10, QFont.Bold)
                ann_item.setFont(ann_font)
                ann_item.setPos(px + 8, py - 22)  # 标签 py-8，注释再往上 14px
                self._waypoint_items.append(ann_item)

            ellipse = self._scene.addEllipse(
                px - 6, py - 6, 12, 12,
                QPen(QColor('#ba7517'), 1.5),
                QBrush(QColor(250, 199, 117, 80)),
            )
            self._waypoint_items.append(ellipse)
            text = self._scene.addText(wp['label'])
            text.setDefaultTextColor(QColor('#854f0b'))
            font = mono_font(8)
            text.setFont(font)
            text.setPos(px + 8, py - 8)
            self._waypoint_items.append(text)

    # ─── 边绘制 (F-5.1, F-5.2, F-15.12, F-15.13) ──────────

    def draw_edges(self, edges: list, waypoints: list,
                   topology=None):
        """
        [IMPL] F-15.12 F-15.13 绘制拓扑边为轨迹折线 + 距离标注。
        topology 传入时尝试加载边轨迹文件绘制折线；无轨迹时 fallback 直线。
        线宽统一 3.0px；单向橙 #ba7517，双向绿 #1d9e75。
        """
        self.clear_edges()
        wp_map = {w['id']: w for w in waypoints}
        # 检测双向边对
        edge_set = {(e['from_id'], e['to_id']) for e in edges}
        drawn_bi = set()  # 已绘制的双向对，避免重复画线
        for e in edges:
            pair_key = frozenset([e['from_id'], e['to_id']])
            is_bi = (e['to_id'], e['from_id']) in edge_set
            if is_bi:
                if pair_key in drawn_bi:
                    continue  # 双向边只画一次
                drawn_bi.add(pair_key)

            edge_color = '#1d9e75' if is_bi else '#ba7517'
            # [IMPL] F-15.12 线宽统一 6.0px
            pen = QPen(QColor(edge_color), 6.0)

            # [IMPL] F-15.12 尝试加载轨迹文件绘制折线
            points = None
            if topology and e.get('traj_file'):
                points = topology._load_edge_trajectory(e['traj_file'])

            if points and len(points) >= 2:
                # 折线绘制：沿轨迹点序列
                path = QPainterPath()
                first_px, first_py = self._w2p(points[0]['x'], points[0]['y'])
                path.moveTo(first_px, first_py)
                for pt in points[1:]:
                    px, py = self._w2p(pt['x'], pt['y'])
                    path.lineTo(px, py)
                item = self._scene.addPath(path, pen)
                self._edge_items.append(item)

                # [IMPL] F-15.13 距离标注取轨迹中点
                mid_idx = len(points) // 2
                mx, my = self._w2p(points[mid_idx]['x'], points[mid_idx]['y'])
            else:
                # fallback：无轨迹时画直线
                a = wp_map.get(e['from_id'])
                b = wp_map.get(e['to_id'])
                if not a or not b:
                    continue
                x0, y0 = self._w2p(a['x'], a['y'])
                x1, y1 = self._w2p(b['x'], b['y'])
                item = self._scene.addLine(x0, y0, x1, y1, pen)
                self._edge_items.append(item)
                # 距离标注取两端中点
                mx, my = (x0 + x1) / 2, (y0 + y1) / 2

            label = self._scene.addText(f"{e['length']:.2f}m")
            label.setDefaultTextColor(QColor('#1d9e75' if is_bi else '#888780'))
            label_font = mono_font(7)
            label.setFont(label_font)
            label.setPos(mx + 4, my - 4)
            self._edge_items.append(label)

    # ─── 规划路径绘制 (F-6.3) ────────────────────────────

    def draw_planned_path(self, path_nodes: list):
        """蓝色虚线 6px 高亮路径，新路径自动清除旧路径 (F-U3)"""
        self.clear_planned_path()
        pen = QPen(QColor('#185fa5'), 6.0, Qt.DashLine)
        for i in range(1, len(path_nodes)):
            x0, y0 = self._w2p(path_nodes[i-1]['x'], path_nodes[i-1]['y'])
            x1, y1 = self._w2p(path_nodes[i]['x'], path_nodes[i]['y'])
            item = self._scene.addLine(x0, y0, x1, y1, pen)
            self._planned_path_items.append(item)

    # ─── 节点命中检测 (F-17.1) ──────────────────────────

    def _hit_test_waypoint(self, scene_pos) -> int | None:
        """
        [IMPL] F-17.1 检测场景坐标是否落在某节点圆圈内（8px 半径）。
        返回 node_id 或 None。
        """
        for node_id, (px, py) in self._waypoint_geoms.items():
            dx = scene_pos.x() - px
            dy = scene_pos.y() - py
            if dx * dx + dy * dy <= 64:  # 8^2 = 64
                return node_id
        return None
