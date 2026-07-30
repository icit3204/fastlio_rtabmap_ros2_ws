import socket
import time
import struct
import random

def main():
    host_ip = "127.0.0.1"  # 替换为服务器的实际 IP 地址
    port = 9999  # 端口号
    server_address = (host_ip, port)

    # 创建 UDP 套接字
    sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            data1 = random.randint(1, 100)  # 生成一个随机 double 数据 单位mm 半径 10000直行 负数是左转 正数是右转
            data2 = random.randint(1, 100)  # 生成另一个随机 double 数据  单位mm/s
            # 将两个 double 数据打包成 bytes
            # '>ddd' 表示网络字节序 (big-endian), 两个 double (8 字节每个)
            # 'd' 表示 8 字节的双精度浮点数
            data_bytes = struct.pack('>dd', data1, data2)

            # 发送数据
            try:
                send_bytes = sockfd.sendto(data_bytes, server_address)
                print(f"发送了两个 double 数据: {data1} 和 {data2}")
                if send_bytes == len(data_bytes):
                    print("发送成功")
                else:
                    print(f"部分数据发送: {send_bytes}/{len(data_bytes)}")
            except socket.error as e:
                print(f"发送失败: {e}")

            time.sleep(1)  # 添加间隔避免过度占用 CPU

    except KeyboardInterrupt:
        print("\nCtrl+C 捕获，正常退出")
    finally:
        sockfd.close()  # 确保套接字关闭

if __name__ == "__main__":
    main()
