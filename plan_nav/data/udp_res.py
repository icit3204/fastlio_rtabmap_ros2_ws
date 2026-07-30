import socket
import struct  # 用于解析结构体数据

def main():
    # 配置服务器地址（与客户端目标地址一致）
    server_addr = ('127.0.0.1', 9999)

    # 创建UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 设置地址重用（可选）
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # 设置为非阻塞模式
    # udp_socket.setblocking(False)

    # 绑定端口
    udp_socket.bind(server_addr)
    print(f"UDP服务端已启动，监听 {server_addr[0]}:{server_addr[1]}")

    while True:
        try:
            # 接收数据（缓冲区大小为16字节，以接收两个 double 数据）
            recv_data, client_addr = udp_socket.recvfrom(16)
            
            # 验证数据长度
            if len(recv_data) != 16:
                print(f"无效数据长度：收到 {len(recv_data)} 字节（预期16字节）")
                continue

            # 解析两个 double 数据
            # 使用 "!" 表示网络字节序（大端），"dd" 表示两个 double 类型
            data1, data2 = struct.unpack('!dd', recv_data)

            # 显示接收信息
            print(f"来自客户端 {client_addr[0]}:{client_addr[1]} 的数据：")
            print(f"原始字节：{recv_data}")
            print(f"解析数值：{data1:.15g} 和 {data2:.15g}")
            print("-" * 40)

        except KeyboardInterrupt:
            print("\n服务端已关闭")
            break
        except struct.error as e:
            print(f"数据解析失败：{str(e)}")
        except Exception as e:
            print(f"接收错误：{str(e)}")

if __name__ == "__main__":
    main()
