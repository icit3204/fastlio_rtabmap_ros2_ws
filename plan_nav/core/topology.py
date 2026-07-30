# [DONE] F-4.1 节点创建 (空格键锁定位置 → 橙色圆圈)
# [DONE] F-4.2 节点聚合 (直径 0.5m 内合并取均值)
# [DONE] F-4.3 节点持久化 (nodes.txt)
# [DONE] F-5.1 默认边生成 (相邻时间戳节点间单向边)
# [DONE] F-5.2 手动添加边 (从节点A拖拽至B, A→B)
# [DONE] F-5.3 边状态切换 (单向↔双向)
# [DONE] F-5.4 边持久化 (edges.txt)

import math
import os
from urllib.parse import quote, unquote

MERGE_RADIUS = 0.25           # 空间合并半径 (m)
MERGE_MAX_TIME_DELTA = 10.0   # [IMPL] F-15.2 合并时间戳差上限 (s)，超过此值不合并
TRAJ_BREAK_THRESHOLD = 2.0    # [IMPL] F-15.5 轨迹断点判定阈值 (m)


class TopologyManager:
    """语义节点与拓扑边的管理、持久化"""

    def __init__(self, work_dir: str):
        self.work_dir = work_dir
        self.waypoints: list[dict] = []
        self.edges: list[dict] = []
        self._next_wp_id = 1
        self._action_history: list[dict] = []  # F-U1 撤销栈

    # ─── 持久化加载 ──────────────────────────────────

    def load_all(self) -> tuple:
        """
        从工作目录加载已有 nodes.txt 和 edges.txt。
        返回 (loaded_nodes_count, loaded_edges_count)
        """
        n = self._load_nodes()
        e = self._load_edges()
        return n, e

    def reset_all(self):
        """清空所有节点、边、撤销历史，并清除 txt 文件。"""
        self.waypoints.clear()
        self.edges.clear()
        self._action_history.clear()
        self._next_wp_id = 1
        self.save_nodes()
        self.save_edges()

    def _load_nodes(self) -> int:
        """
        从 nodes.txt 恢复语义节点，兼容新旧格式 (F-17.10 F-15.1)。

        v1.1 (9列): node_id, label, annotation, x, y, z, yaw_deg, timestamp, traj_idx
        v1.0 (8列): node_id, label, x, y, z, yaw_deg, timestamp, traj_idx
        旧 7 列: node_id, label, x, y, z, yaw_deg, timestamp
        """
        path = os.path.join(self.work_dir, 'nodes.txt')
        if not os.path.exists(path):
            return 0
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = [p.strip() for p in line.split(',')]
                if len(parts) < 7:
                    continue

                node_id = int(parts[0])
                if len(parts) >= 9:
                    # [IMPL] F-17.7 v1.1 新格式：含 annotation 列
                    node = {
                        'id': node_id,
                        'label': parts[1],
                        'annotation': self._decode_annotation(parts[2]),
                        'x': float(parts[3]),
                        'y': float(parts[4]),
                        'z': float(parts[5]),
                        'yaw': math.radians(float(parts[6])),
                        'timestamp': float(parts[7]),
                        'traj_idx': int(parts[8]),
                    }
                elif len(parts) == 8:
                    # v1.0 旧格式：无 annotation，annotation 默认空
                    node = {
                        'id': node_id,
                        'label': parts[1],
                        'annotation': '',  # [IMPL] F-17.10 旧格式默认空注释
                        'x': float(parts[2]),
                        'y': float(parts[3]),
                        'z': float(parts[4]),
                        'yaw': math.radians(float(parts[5])),
                        'timestamp': float(parts[6]),
                        'traj_idx': int(parts[7]),
                    }
                else:
                    # 旧 7 列：无 annotation 且无 traj_idx
                    node = {
                        'id': node_id,
                        'label': parts[1],
                        'annotation': '',  # [IMPL] F-17.10 默认空
                        'x': float(parts[2]),
                        'y': float(parts[3]),
                        'z': float(parts[4]),
                        'yaw': math.radians(float(parts[5])),
                        'timestamp': float(parts[6]),
                        'traj_idx': -1,  # [IMPL] F-15.1 旧格式缺省 -1
                    }
                self.waypoints.append(node)
                if node_id >= self._next_wp_id:
                    self._next_wp_id = node_id + 1
        return len(self.waypoints)

    def _load_edges(self) -> int:
        """从 edges.txt 恢复拓扑边，兼容旧 4 列格式 (F-15.9 traj_file)"""
        path = os.path.join(self.work_dir, 'edges.txt')
        if not os.path.exists(path):
            return 0
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = [p.strip() for p in line.split(',')]
                if len(parts) < 4:
                    continue
                # 格式: from_id, to_id, length_m, direction[, traj_file]
                edge = {
                    'from_id': int(parts[0]),
                    'to_id': int(parts[1]),
                    'length': float(parts[2]),
                    'direction': parts[3],
                    'traj_file': parts[4] if len(parts) >= 5 else '',
                    # [IMPL] F-15.11 旧格式（4 列）traj_file 为空串，后续由补建逻辑处理
                }
                self.edges.append(edge)
        return len(self.edges)

    def add_waypoint(self, x: float, y: float, z: float,
                     yaw: float, timestamp: float,
                     traj_idx: int = -1) -> dict:
        """
        [IMPL] F-15.1 F-15.2 添加语义节点。
        合并规则：0.25m 内且时间戳差 ≤ 10s → 合并取均值；
        0.25m 内但时间戳差 > 10s → 不合并，作为新节点。
        新节点存储 traj_idx 字段。
        返回最终节点（新增或合并后）。
        """
        for wp in self.waypoints:
            dist = math.hypot(wp['x'] - x, wp['y'] - y)
            if dist < MERGE_RADIUS:
                # [IMPL] F-15.2 时间戳检查：差距过大不合并（如回环重叠区域）
                time_delta = abs(wp['timestamp'] - timestamp)
                if time_delta <= MERGE_MAX_TIME_DELTA:
                    # 合并：取均值
                    wp['x'] = (wp['x'] + x) / 2
                    wp['y'] = (wp['y'] + y) / 2
                    wp['z'] = (wp['z'] + z) / 2
                    wp['yaw'] = (wp['yaw'] + yaw) / 2
                    # 合并时更新 traj_idx（若新值有效）
                    if traj_idx >= 0:
                        wp['traj_idx'] = traj_idx
                    self.save_nodes()
                    return wp
                # else: 时间差 > 10s，不合并，继续检查下一个或新增

        node = {
            'id': self._next_wp_id,
            'label': f'WP-{self._next_wp_id:02d}',
            'annotation': '',  # [IMPL] F-17.10 默认空注释
            'x': x, 'y': y, 'z': z,
            'yaw': yaw, 'timestamp': timestamp,
            'traj_idx': traj_idx,  # [IMPL] F-15.1 轨迹索引，-1 表示无关联
        }
        self.waypoints.append(node)
        self._next_wp_id += 1
        self._action_history.append({'type': 'add_node', 'node_id': node['id']})
        self.save_nodes()
        return node

    def save_nodes(self):
        """[IMPL] F-15.1 F-17.7 持久化节点到 nodes.txt，含 annotation 列（URL-encode）"""
        path = os.path.join(self.work_dir, 'nodes.txt')
        with open(path, 'w', encoding='utf-8') as f:
            f.write('# node_id, label, annotation, x, y, z, '
                    'yaw_deg, timestamp_unix, traj_idx\n')
            for w in self.waypoints:
                ann = self._encode_annotation(w.get('annotation', ''))
                f.write(
                    f"{w['id']}, {w['label']}, {ann}, "
                    f"{w['x']:.4f}, {w['y']:.4f}, {w['z']:.4f}, "
                    f"{math.degrees(w['yaw']):.2f}, "
                    f"{w['timestamp']:.3f}, {w.get('traj_idx', -1)}\n"
                )

    # ─── 节点注释 (F-17.2 F-17.6 F-17.8) ──────────────────

    def set_annotation(self, node_id: int, text: str) -> bool:
        """
        [IMPL] F-17.2/F-17.6 设置节点注释。
        text 为空字符串时清空注释。
        值未变化时不记录撤销历史。
        返回 True 表示成功修改，False 表示节点不存在。
        """
        for w in self.waypoints:
            if w['id'] == node_id:
                old_val = w.get('annotation', '')
                new_val = text.strip()
                if old_val == new_val:
                    return True  # 无变化，不记录历史
                # [IMPL] F-17.8 记录撤销历史
                self._action_history.append({
                    'type': 'set_annotation',
                    'node_id': node_id,
                    'old_value': old_val,
                    'new_value': new_val,
                })
                w['annotation'] = new_val
                self.save_nodes()
                return True
        return False

    # ─── 撤销操作 (F-U1) ──────────────────────────────

    def undo(self) -> str | None:
        """撤销最近一次操作（节点或边），返回撤销描述"""
        if not self._action_history:
            return None
        action = self._action_history.pop()
        if action['type'] == 'add_node':
            node_id = action['node_id']
            self.waypoints = [w for w in self.waypoints
                              if w['id'] != node_id]
            # 删除关联的边
            self.edges = [e for e in self.edges
                          if e['from_id'] != node_id
                          and e['to_id'] != node_id]
            # 回退 ID 计数器
            if node_id == self._next_wp_id - 1:
                self._next_wp_id = node_id
            self.save_nodes()
            self.save_edges()
            return f'撤销节点 WP-{node_id:02d}'
        elif action['type'] == 'add_edge':
            self.edges = [e for e in self.edges
                          if not (e['from_id'] == action['from_id']
                                  and e['to_id'] == action['to_id'])]
            self.save_edges()
            return (f'撤销边 WP-{action["from_id"]:02d} → '
                    f'WP-{action["to_id"]:02d}')
        # [IMPL] F-17.8 撤销注释修改
        elif action['type'] == 'set_annotation':
            node_id = action['node_id']
            for w in self.waypoints:
                if w['id'] == node_id:
                    w['annotation'] = action['old_value']
                    self.save_nodes()
                    if action['old_value']:
                        return (f'撤销注释 WP-{node_id:02d} '
                                f'恢复为 "{action["old_value"]}"')
                    else:
                        return f'撤销注释 WP-{node_id:02d} (已清空)'
            return None
        return None

    # ─── 边管理 (F-5.1 ~ F-5.4) ────────────────────────

    def generate_default_edges(self, db_nodes: list | None = None):
        """
        [IMPL] F-15.3 相邻时间戳节点间自动生成默认单向边 (F-5.1)。
        db_nodes 传入时提取轨迹段，否则仅存欧氏距离。
        """
        sorted_wps = sorted(self.waypoints, key=lambda w: w['timestamp'])
        for i in range(len(sorted_wps) - 1):
            a, b = sorted_wps[i], sorted_wps[i + 1]
            # 检查是否已存在
            if not any(e['from_id'] == a['id'] and e['to_id'] == b['id']
                       for e in self.edges):
                self.add_edge(a['id'], b['id'], 'uni', db_nodes=db_nodes)

    def add_edge(self, from_id: int, to_id: int,
                 direction: str = 'uni',
                 db_nodes: list | None = None) -> dict:
        """
        [IMPL] F-15.3 添加边 A→B，可选从 DB 轨迹提取边路径。

        db_nodes: db_data['nodes'] 列表，用于提取轨迹段。
                  为 None 时仅存欧氏距离（兼容旧逻辑）。
        """
        length = self._calc_length(from_id, to_id)
        edge = {
            'from_id': from_id,
            'to_id': to_id,
            'length': round(length, 3),
            'direction': direction,
            'traj_file': None,  # [IMPL] F-15.8 默认无轨迹文件
        }

        # [IMPL] F-15.3 提取轨迹段
        if db_nodes:
            a = next((w for w in self.waypoints if w['id'] == from_id), None)
            b = next((w for w in self.waypoints if w['id'] == to_id), None)
            if a and b:
                points = self._extract_trajectory_segment(db_nodes, a, b, direction)
                if points:
                    edge_idx = len(self.edges) + 1
                    edge['traj_file'] = f'edge_{edge_idx}_traj.txt'
                    self._save_edge_trajectory(edge['traj_file'], points)

        self.edges.append(edge)
        self._action_history.append(
            {'type': 'add_edge', 'from_id': from_id, 'to_id': to_id}
        )
        self.save_edges()
        return edge

    def remove_waypoint(self, node_id: int) -> str | None:
        """
        删除语义节点及关联边，重新索引剩余节点使 ID 连续。
        返回被删除节点的 label，若节点不存在则返回 None。
        """
        target = next((w for w in self.waypoints if w['id'] == node_id), None)
        if target is None:
            return None
        label = target['label']
        self.waypoints = [w for w in self.waypoints if w['id'] != node_id]
        # 删除关联边
        self.edges = [e for e in self.edges
                      if e['from_id'] != node_id and e['to_id'] != node_id]
        # 重新索引：按 id 排序后重编号
        self.waypoints.sort(key=lambda w: w['id'])
        old_to_new = {}
        for i, w in enumerate(self.waypoints, start=1):
            old_id = w['id']
            w['id'] = i
            w['label'] = f'WP-{i:02d}'
            old_to_new[old_id] = i
        # 更新边中的节点引用
        for e in self.edges:
            e['from_id'] = old_to_new[e['from_id']]
            e['to_id'] = old_to_new[e['to_id']]
        self._next_wp_id = len(self.waypoints) + 1
        # 重索引后撤销栈中旧 ID 全部失效，直接清空
        self._action_history.clear()
        self.save_nodes()
        self.save_edges()
        return label

    def remove_edge(self, from_id: int, to_id: int):
        """删除边"""
        self.edges = [
            e for e in self.edges
            if not (e['from_id'] == from_id and e['to_id'] == to_id)
        ]
        self.save_edges()

    def remove_edge_bidirectional(self, from_id: int, to_id: int) -> int:
        """
        删除边；若反向边也存在则一并删除（双向边一次删两个方向）。
        返回实际删除的边数（1 或 2）。
        """
        count = 0
        # 删除正向
        old_len = len(self.edges)
        self.edges = [
            e for e in self.edges
            if not (e['from_id'] == from_id and e['to_id'] == to_id)
        ]
        if len(self.edges) < old_len:
            count += 1
        # 删除反向
        old_len = len(self.edges)
        self.edges = [
            e for e in self.edges
            if not (e['from_id'] == to_id and e['to_id'] == from_id)
        ]
        if len(self.edges) < old_len:
            count += 1
        if count > 0:
            self.save_edges()
        return count

    def toggle_direction(self, from_id: int, to_id: int):
        """切换边方向 uni↔bi (F-5.3)"""
        for e in self.edges:
            if e['from_id'] == from_id and e['to_id'] == to_id:
                e['direction'] = 'bi' if e['direction'] == 'uni' else 'uni'
                self.save_edges()
                return

    def _calc_length(self, from_id: int, to_id: int) -> float:
        """计算两节点间欧氏距离"""
        a = next(w for w in self.waypoints if w['id'] == from_id)
        b = next(w for w in self.waypoints if w['id'] == to_id)
        return math.hypot(a['x'] - b['x'], a['y'] - b['y'])

    def save_edges(self):
        """[IMPL] F-15.9 持久化边到 edges.txt，含第 5 列 traj_file"""
        path = os.path.join(self.work_dir, 'edges.txt')
        with open(path, 'w', encoding='utf-8') as f:
            f.write('# from_id, to_id, length_m, direction, traj_file\n')
            for e in self.edges:
                traj = e.get('traj_file', '') or ''
                f.write(
                    f"{e['from_id']}, {e['to_id']}, "
                    f"{e['length']}, {e['direction']}, {traj}\n"
                )

    # ═══════════════════════════════════════════════════════
    # [IMPL] F-15.3 ~ F-15.10 轨迹边提取与持久化
    # ═══════════════════════════════════════════════════════

    def _extract_trajectory_segment(self, db_nodes: list,
                                     from_wp: dict, to_wp: dict,
                                     direction: str) -> list[dict]:
        """
        [IMPL] F-15.3 F-15.4 F-15.5 F-15.6 从 DB 轨迹中提取两个节点间的轨迹点序列。

        - 正向边 A→B (A.traj_idx <= B.traj_idx)：取 db_nodes[from_idx:to_idx+1]
        - 反向边 (B.traj_idx < A.traj_idx)：取 DB 段后反转
        - traj_idx < 0 时返回空列表
        - 遇到间距 > TRAJ_BREAK_THRESHOLD 的断点时，分段并在断点处插值补点

        返回：
          [{'x','y','z','yaw','timestamp'}, ...]  按规划方向排列
        """
        from core.pathfinder import _split_by_breaks, _interleave_segments

        a_idx = from_wp.get('traj_idx', -1)
        b_idx = to_wp.get('traj_idx', -1)

        if a_idx < 0 or b_idx < 0:
            # 无 DB 轨迹关联
            return []

        # 按索引方向切片
        if a_idx <= b_idx:
            raw = list(db_nodes[a_idx:b_idx + 1])  # 含 to_idx
            forward = True
        else:
            raw = list(db_nodes[b_idx:a_idx + 1])  # 从较小索引到较大索引
            forward = False

        # 检测断点，分段
        segments = _split_by_breaks(raw, threshold=TRAJ_BREAK_THRESHOLD)

        # 断点之间插值补点
        result = _interleave_segments(segments, db_nodes)

        # 若非正向则整体反转（F-15.4 反向边倒序）
        if not forward:
            result.reverse()

        return result

    def _save_edge_trajectory(self, filename: str, points: list[dict]):
        """
        [IMPL] F-15.8 F-15.10 将轨迹点序列写入工作目录下的 edge_N_traj.txt。
        格式: x, y, z, yaw_rad, timestamp_unix
        """
        path = os.path.join(self.work_dir, filename)
        with open(path, 'w', encoding='utf-8') as f:
            f.write('# x, y, z, yaw_rad, timestamp_unix\n')
            for p in points:
                f.write(f"{p['x']:.4f}, {p['y']:.4f}, {p['z']:.4f}, "
                        f"{p['yaw']:.4f}, {p['timestamp']:.3f}\n")

    def _load_edge_trajectory(self, filename: str) -> list[dict]:
        """
        [IMPL] F-15.8 F-15.12 从 edge_N_traj.txt 加载轨迹点序列。
        返回 list[dict]，文件不存在时返回空列表。
        """
        path = os.path.join(self.work_dir, filename)
        if not os.path.exists(path):
            return []
        points = []
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = [p.strip() for p in line.split(',')]
                if len(parts) < 5:
                    continue
                points.append({
                    'x': float(parts[0]),
                    'y': float(parts[1]),
                    'z': float(parts[2]),
                    'yaw': float(parts[3]),
                    'timestamp': float(parts[4]),
                })
        return points

    # ─── 注释编码辅助 (F-17.7) ───────────────────────────

    @staticmethod
    def _encode_annotation(text: str) -> str:
        """[IMPL] F-17.7 URL-encode 注释文本，防止逗号污染 CSV"""
        if not text:
            return ''
        return quote(text, safe='')

    @staticmethod
    def _decode_annotation(encoded: str) -> str:
        """[IMPL] F-17.7 URL-decode 注释文本"""
        if not encoded:
            return ''
        return unquote(encoded)
