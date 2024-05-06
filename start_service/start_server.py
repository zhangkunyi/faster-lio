import socket

from process_manager import process_manager
from inspection_command import inspection_command

import subprocess
import time
if __name__ == "__main__":
    # 创建UDP套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_host = "192.168.144.8"  # 主机地址
    udp_port = 12345  # 端口号
    udp_socket.bind((udp_host, udp_port))

    while True:
        data, addr = udp_socket.recvfrom(1024)
        command = data.decode()
        if command.startswith("inspection"):
            inspection_command.do_command(command)
        elif command.startswith("process"):
            process_manager.do_command(command)
            
