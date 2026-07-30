#!/usr/bin/env python3
# [DONE] F-11.5 接收端适配：binary struct 解析
"""UDP 路径接收端 — 接收来自 Underground Map Editor 规划模式的 Pure Pursuit 数据

用法:
    python tools/udp_receiver.py                          # 默认 0.0.0.0:14550
    python tools/udp_receiver.py --port 15550             # 指定端口
    python tools/udp_receiver.py -o ./received            # 指定输出目录
    python tools/udp_receiver.py -t 10                    # 10 秒超时

发送端数据帧格式 (binary):
    struct.pack('>dd', R, v)  — 16 字节大端
    R: 转弯半径 (mm), v: 速度 (mm/s)
"""

import socket
import struct
import json
import argparse
import signal
import sys
import time
from pathlib import Path
from datetime import datetime


class UdpReceiver:
    """UDP 路径接收器：监听 → 实时显示 → 排序 → 保存 → 统计"""

    def __init__(self, ip: str = '0.0.0.0', port: int = 14550,
                 output_dir: str = '.', timeout: float = 5.0):
        self.ip = ip
        self.port = port
        self.output_dir = Path(output_dir)
        self.timeout = timeout
        self.packets: dict[int, dict] = {}  # seq → 数据包
        self._running = True
        self._last_recv = time.time()

    # ─── 启动监听 ─────────────────────────────────────

    def start(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.ip, self.port))
        sock.settimeout(0.5)  # 0.5s 超时以便检查 _running 标志

        print(f'[UDP 接收端] 监听 {self.ip}:{self.port}')
        print(f'[UDP 接收端] 超时 {self.timeout}s — 无新数据自动结束')
        print('─' * 50)

        seq = 0
        while self._running:
            try:
                data, addr = sock.recvfrom(4096)
                self._last_recv = time.time()

                # F-11.5 binary 格式：16 字节大端 double × 2
                if len(data) != 16:
                    continue

                R_val, v_val = struct.unpack('>dd', data)
                packet = {'seq': seq, 'R': R_val, 'v': v_val}
                self.packets[seq] = packet
                self._print_status(packet)
                seq += 1
            except socket.timeout:
                # 超时自动结束
                if self.packets and time.time() - self._last_recv > self.timeout:
                    break

        sock.close()
        print('\n')
        self._process_result()

    def _print_status(self, packet: dict):
        """实时单行刷新"""
        seq = packet.get('seq', -1)
        R = packet.get('R', 0.0)
        v = packet.get('v', 0.0)
        count = len(self.packets)
        print(f'\r  seq={seq:04d}  R={R:+.1f} mm  v={v:.1f} mm/s  '
              f'|  已收 {count} 点  ', end='', flush=True)

    # ─── 结果处理 ─────────────────────────────────────

    def _process_result(self):
        sorted_seqs = sorted(self.packets.keys())
        if not sorted_seqs:
            print('[WARN] 未收到任何数据包')
            return

        path = [self.packets[s] for s in sorted_seqs]

        # 丢包检测
        expected_max = max(sorted_seqs)
        missing = [s for s in range(expected_max + 1) if s not in self.packets]
        if missing:
            print(f'[WARN] 检测到丢包: seq={missing}')

        # 控制量统计
        R_vals = [p['R'] for p in path]
        v_vals = [p['v'] for p in path]
        straight_count = sum(1 for R in R_vals if abs(abs(R) - 10000.0) < 0.01)
        turn_count = len(path) - straight_count

        print('─' * 50)
        print(f'  接收完成: {len(path)} 个路径点')
        print(f'  R 范围: [{min(R_vals):.1f}, {max(R_vals):.1f}] mm')
        print(f'  v 范围: [{min(v_vals):.1f}, {max(v_vals):.1f}] mm/s')
        print(f'  直行帧: {straight_count} / 转弯帧: {turn_count}')

        # 保存到文件
        self.output_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # JSON（结构化，含元信息）
        json_path = self.output_dir / f'path_{ts}.json'
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump({
                'metadata': {
                    'source_ip': self.ip,
                    'source_port': self.port,
                    'received_at': datetime.now().isoformat(),
                    'point_count': len(path),
                    'straight_frames': straight_count,
                    'turn_frames': turn_count,
                },
                'path': path,
            }, f, ensure_ascii=False, indent=2)
        print(f'  已保存: {json_path}')

        # CSV（表格，便于导入 Excel/QGIS）
        csv_path = self.output_dir / f'path_{ts}.csv'
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.write('seq,R_mm,v_mms\n')
            for p in path:
                f.write(f"{p['seq']},{p['R']:.4f},{p['v']:.4f}\n")
        print(f'  已保存: {csv_path}')

    def stop(self):
        """外部停止（信号处理）"""
        self._running = False


# ─── 入口 ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='UDP 路径接收端 — 接收 Underground Map Editor Pure Pursuit 数据'
    )
    parser.add_argument('--ip', default='0.0.0.0',
                        help='监听 IP，默认 0.0.0.0（所有网卡）')
    parser.add_argument('--port', '-p', type=int, default=14550,
                        help='监听端口，默认 14550')
    parser.add_argument('--output', '-o', default='./received_paths',
                        help='输出目录，默认 ./received_paths')
    parser.add_argument('--timeout', '-t', type=float, default=5.0,
                        help='接收超时秒数，默认 5.0')
    args = parser.parse_args()

    receiver = UdpReceiver(args.ip, args.port, args.output, args.timeout)

    # 优雅退出
    def _on_interrupt(sig, frame):
        print('\n[INFO] 收到中断信号，正在结束接收...')
        receiver.stop()
    signal.signal(signal.SIGINT, _on_interrupt)

    receiver.start()


if __name__ == '__main__':
    main()
