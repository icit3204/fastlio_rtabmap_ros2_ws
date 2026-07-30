# [DONE] F-6.1 节点选择 (规划模式下依次点击起点与终点)
# [DONE] F-6.2 Dijkstra 最短路径 (欧氏距离为权重)
# [DONE] F-6.3 路径可视化 (蓝色虚线 2px 高亮, 状态栏显示总长度)
# [DONE] F-11.6 插值点携带 speed/radius 静态估算

import math
import networkx as nx


def build_graph(waypoints: list, edges: list) -> nx.DiGraph:
    """从节点和边列表构建有向图"""
    G = nx.DiGraph()
    for wp in waypoints:
        G.add_node(wp['id'], x=wp['x'], y=wp['y'])
    for e in edges:
        G.add_edge(e['from_id'], e['to_id'], weight=e['length'])
        if e['direction'] == 'bi':
            G.add_edge(e['to_id'], e['from_id'], weight=e['length'])
    return G


def find_path(G: nx.DiGraph, start_id: int, end_id: int) -> tuple:
    """
    Dijkstra 最短路径搜索。

    返回:
      (path_ids, total_length): 节点ID列表和总长度
      无路径时返回 ([], -1)
    """
    try:
        path_ids = nx.shortest_path(
            G, start_id, end_id, weight='weight'
        )
        length = nx.shortest_path_length(
            G, start_id, end_id, weight='weight'
        )
        return path_ids, round(length, 3)
    except nx.NetworkXNoPath:
        return [], -1


# [DEPRECATED] F-6.5 — 替换为 concat_trajectory_segments（F-15.14）
def interpolate_path(path_waypoints: list,
                     db_nodes: list | None = None) -> list[dict]:
    """
    沿规划路径的每条边直线插值，生成密集轨迹点。

    空间步长取原 DB 轨迹的平均步长（缺省 0.1m）；
    时间戳取原 DB 轨迹的平均帧间隔均匀递增；
    偏航角 yaw 取当前点到下一目标点的行进方向。

    返回:
      [{'x': float, 'y': float, 'yaw': float, 'timestamp': float}, ...]
    """
    if len(path_waypoints) < 2:
        # 单点路径直接返回
        if path_waypoints:
            wp = path_waypoints[0]
            return [{'x': wp['x'], 'y': wp['y'],
                     'yaw': wp.get('yaw', 0.0),
                     'timestamp': wp.get('timestamp', 0.0),
                     'speed': 5000.0, 'radius': 10000.0}]
        return []

    # 从原轨迹推算空间步长和时间间隔
    step_m, time_dt = _derive_params(db_nodes)

    t0 = path_waypoints[0].get('timestamp', 0.0)
    result = []
    t = t0

    for i in range(len(path_waypoints) - 1):
        a = path_waypoints[i]
        b = path_waypoints[i + 1]
        dx = b['x'] - a['x']
        dy = b['y'] - a['y']
        seg_len = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)

        if seg_len < 1e-6:
            # 零长边，只记录起点
            result.append({'x': a['x'], 'y': a['y'],
                           'yaw': yaw, 'timestamp': t,
                           'speed': 5000.0, 'radius': 10000.0})
            t += time_dt
            continue

        steps = max(1, int(seg_len / step_m))
        for s in range(steps):
            ratio = s / steps
            result.append({
                'x': a['x'] + dx * ratio,
                'y': a['y'] + dy * ratio,
                'yaw': yaw,
                'timestamp': t,
                'speed': 5000.0,
                'radius': 10000.0,
            })
            t += time_dt

    # 终点
    last = path_waypoints[-1]
    yaw_end = result[-1]['yaw'] if result else 0.0
    result.append({
        'x': last['x'], 'y': last['y'],
        'yaw': yaw_end,
        'timestamp': t,
        'speed': 5000.0,
        'radius': 10000.0,
    })

    return result


def _derive_params(db_nodes: list | None) -> tuple[float, float]:
    """
    从原 DB 轨迹推算 (平均空间步长_米, 平均时间间隔_秒)。
    数据不足时返回默认值 (0.1, 0.1)。
    """
    if not db_nodes or len(db_nodes) < 2:
        return 0.1, 0.1

    total_dist = 0.0
    for i in range(1, len(db_nodes)):
        dx = db_nodes[i]['x'] - db_nodes[i - 1]['x']
        dy = db_nodes[i]['y'] - db_nodes[i - 1]['y']
        total_dist += math.hypot(dx, dy)

    t0 = db_nodes[0].get('timestamp', 0)
    t1 = db_nodes[-1].get('timestamp', 0)
    total_time = t1 - t0 if t1 > t0 else len(db_nodes) * 0.1

    avg_dist = total_dist / (len(db_nodes) - 1) if total_dist > 0 else 0.1
    avg_dt = total_time / (len(db_nodes) - 1) if total_time > 0 else 0.1
    return avg_dist, avg_dt


# ═══════════════════════════════════════════════════════════
# [IMPL] F-15.6 F-15.7 F-15.14 F-15.16 轨迹路径规划工具函数
# ═══════════════════════════════════════════════════════════

def _derive_traj_params(db_nodes: list | None) -> tuple[float, float]:
    """
    [IMPL] F-15.6 从 DB 轨迹推算 (平均空间步长_米, 平均时间间隔_秒)。
    同 _derive_params 逻辑，导出供 topology.py 使用。
    数据不足时返回默认值 (0.1, 0.1)。
    """
    if not db_nodes or len(db_nodes) < 2:
        return 0.1, 0.1

    total_dist = 0.0
    for i in range(1, len(db_nodes)):
        dx = db_nodes[i]['x'] - db_nodes[i - 1]['x']
        dy = db_nodes[i]['y'] - db_nodes[i - 1]['y']
        total_dist += math.hypot(dx, dy)

    t0 = db_nodes[0].get('timestamp', 0)
    t1 = db_nodes[-1].get('timestamp', 0)
    total_time = t1 - t0 if t1 > t0 else len(db_nodes) * 0.1

    avg_dist = total_dist / (len(db_nodes) - 1) if total_dist > 0 else 0.1
    avg_dt = total_time / (len(db_nodes) - 1) if total_time > 0 else 0.1
    return avg_dist, avg_dt


def _split_by_breaks(points: list, threshold: float = 2.0) -> list[list]:
    """
    [IMPL] F-15.5 按间距阈值将轨迹点序列切割为多段。
    相邻点间距 > threshold 时切分（如回环闭合跳变）。
    返回 list of segments，每个 segment 是连续点的列表。
    """
    if len(points) < 2:
        return [points] if points else []

    segments = []
    seg = [points[0]]
    for i in range(1, len(points)):
        d = math.hypot(points[i]['x'] - points[i - 1]['x'],
                       points[i]['y'] - points[i - 1]['y'])
        if d > threshold:
            segments.append(seg)
            seg = [points[i]]
        else:
            seg.append(points[i])
    if seg:
        segments.append(seg)
    return segments


def _interleave_segments(segments: list, db_nodes: list) -> list[dict]:
    """
    [IMPL] F-15.6 在多段轨迹段之间插入直线插值补点。
    插值步长和时间间隔从 db_nodes 推算（调用 _derive_traj_params）。
    插值点包含完整的 x/y/z/yaw/timestamp。
    返回完整轨迹点列表。
    """
    if len(segments) == 1:
        return list(segments[0])  # 返回副本

    # 从 db_nodes 推算平均步长和时间间隔
    step_m, time_dt = _derive_traj_params(db_nodes)

    result = list(segments[0])
    for i in range(1, len(segments)):
        # 前段终点 → 后段起点 之间直线插值
        prev_end = segments[i - 1][-1]
        next_start = segments[i][0]
        dx = next_start['x'] - prev_end['x']
        dy = next_start['y'] - prev_end['y']
        seg_len = math.hypot(dx, dy)

        if seg_len > 1e-6:
            t0 = prev_end.get('timestamp', 0)
            steps = max(1, int(seg_len / step_m))
            yaw = math.atan2(dy, dx)
            dz = next_start['z'] - prev_end['z']
            for s in range(1, steps):  # 不含端点（端点已由段提供）
                ratio = s / steps
                result.append({
                    'x': prev_end['x'] + dx * ratio,
                    'y': prev_end['y'] + dy * ratio,
                    'z': prev_end['z'] + dz * ratio,
                    'yaw': yaw,
                    'timestamp': t0 + time_dt * s,
                })

        result.extend(segments[i])

    return result


def _find_edge(edges: list, from_id: int, to_id: int) -> dict | None:
    """
    [IMPL] F-15.14 在边列表中查找 from→to 的边（精确匹配）。
    返回 dict 或 None。
    """
    for e in edges:
        if e['from_id'] == from_id and e['to_id'] == to_id:
            return e
    return None


def concat_trajectory_segments(path_waypoints: list,
                                edges: list,
                                topology) -> list[dict]:
    """
    [IMPL] F-15.14 将规划路径经过的节点间的轨迹段拼接为完整轨迹。
    替换原 interpolate_path。

    参数:
      path_waypoints: Dijkstra 输出的节点序列 [WP-A, WP-X, WP-B]
      edges: 拓扑边列表
      topology: TopologyManager 实例，用于加载边轨迹文件

    返回:
      拼接后的轨迹点列表 [{'x','y','z','yaw','timestamp'}, ...]
      无轨迹文件时 fallback 为两点直线。
    """
    if len(path_waypoints) < 2:
        if path_waypoints:
            wp = path_waypoints[0]
            return [{'x': wp['x'], 'y': wp['y'],
                     'z': wp.get('z', 0.0), 'yaw': wp.get('yaw', 0.0),
                     'timestamp': wp.get('timestamp', 0.0)}]
        return []

    result = []
    for i in range(len(path_waypoints) - 1):
        a = path_waypoints[i]
        b = path_waypoints[i + 1]

        # 查找对应的边
        edge = _find_edge(edges, a['id'], b['id'])
        traj = None

        if edge and edge.get('traj_file'):
            traj = topology._load_edge_trajectory(edge['traj_file'])

        # [IMPL] F-15.4 若正向边不存在，尝试反向边（双向边场景 B→A）
        if not traj and not edge:
            reverse_edge = _find_edge(edges, b['id'], a['id'])
            if reverse_edge and reverse_edge.get('traj_file'):
                raw = topology._load_edge_trajectory(reverse_edge['traj_file'])
                if raw:
                    traj = list(reversed(raw))  # 倒序为 B→A 方向

        if traj:
            # 跳过重复端点（前一段的终点 = 后一段的起点）
            if result:
                last = result[-1]
                first = traj[0]
                if (abs(last['x'] - first['x']) < 1e-4 and
                        abs(last['y'] - first['y']) < 1e-4):
                    result.extend(traj[1:])  # 跳过重复首点
                else:
                    result.extend(traj)
            else:
                result.extend(traj)
        else:
            # fallback：无轨迹文件时的降级直线
            result.append({'x': a['x'], 'y': a['y'],
                           'z': a.get('z', 0.0), 'yaw': a.get('yaw', 0.0),
                           'timestamp': a.get('timestamp', 0.0)})
            result.append({'x': b['x'], 'y': b['y'],
                           'z': b.get('z', 0.0), 'yaw': b.get('yaw', 0.0),
                           'timestamp': b.get('timestamp', 0.0)})

    return result
