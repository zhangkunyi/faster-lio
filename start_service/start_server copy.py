import subprocess
import os
import signal
import socket

def start_ros_script(script_path):
    try:
        #process = subprocess.run(['source', '/home/nv/quzhou-project/devel/setup.zsh'], shell=True, check=True)
        process = subprocess.Popen(['zsh', script_path])
        return process.pid 
    except subprocess.CalledProcessError as e:
        print(f"启动ROS脚本时发生错误：{e}")


def stop_ros_script(pid):
    try: 
        os.kill(pid,signal.SIGTERM)
        #pid.terminate()
        #subprocess.run(['pkill', sh_name])
        # output = subprocess.check_output(['pgrep', '-f', sh_name])
        # pids = output.decode().split('\n')[:-1]  # 获取所有进程PID并去除空值

        # # 关闭每个进程
        # for pid in pids:
        #     subprocess.run(['kill', pid])
    except ProcessLookupError as e:
        print(f"进程 {pid} 不存在：{e}")

# def send_udp_message(message, dest_ip, dest_port):
#     # 创建UDP套接字
#     udp_socket_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     #global udp_socket
#     try:
#         # 发送数据
#         message = "apm_start"
#         print(message)
#         message_encode = message.encode()
#         print(message_encode)
#         udp_socket_send.sendto(message.encode(), (dest_ip, dest_port))
#         print(f"已发送消息到 {dest_ip}:{dest_port}")
#     except Exception as e:
#         print(f"发送消息时发生错误：{e}")
#     finally:
#         # 关闭套接字
#         udp_socket.close()


# def ekf_callback(data):
#     # 提取位置和速度信息
#     position_x = data.pose.pose.position.x
#     position_y = data.pose.pose.position.y
#     position_z = data.pose.pose.position.z
#     linear_x = data.twist.twist.linear.x
#     linear_y = data.twist.twist.linear.y
#     linear_z = data.twist.twist.linear.z

#     # 将信息转换为float类型
#     position_x_float = float(position_x)
#     position_y_float = float(position_y)
#     position_z_float = float(position_z)
#     linear_x_float = float(linear_x)
#     linear_y_float = float(linear_y)
#     linear_z_float = float(linear_z)

#     # 构造消息字符串
#     message = f"Position: ({position_x_float}, {position_y_float}, {position_z_float}), Linear Velocity: ({linear_x_float}, {linear_y_float}, {linear_z_float})"

#     #rospy.loginfo("ekf received :%d",ekf)
#     dest_ip = "192.168.32.36"
#     dest_port = "10000"
#     send_udp_message(message,dest_ip,dest_port)


# def listener():
#     rospy.init_node('listener',anonymous=True)
#     rospy.Subscriber("/ekf_fuser/odom",Odometry,ekf_callback)
#    # rospy.Subscriber("/")
#     print("ddddddddddd")
    


if __name__ == "__main__":
    faster_lio_path = "/home/nv/quzhou-project/sh_files/faster_lio.sh"
    apm_path = "/home/nv/quzhou-project/sh_files/apm.sh" 
    camera_path = "/home/nv/quzhou-project/sh_files/camera_detect.sh"
    task_planner_path = "/home/nv/quzhou-project/sh_files/task_planner.sh"
    take_off_path = "/home/nv/quzhou-project/sh_files/take_off.sh"# 将路径替换为你的ROS脚本的路径
    # ros_id = start_ros_script(script_path)
# if restart == 1:
#     stop_ros_script(ros_id)
#     time.sleep(2)
#     start_ros_script(script_path)
# 创建UDP套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_host = "192.168.144.8"  # 主机地址
    udp_port = 12345  # 端口号
    udp_socket.bind((udp_host, udp_port))
   # listener()
    timeout = 0.3
    start_ros_script(faster_lio_path)
    #udp_socket.setblocking(False)
   # started = 0
    #if not started:
    # faster_lio_pid = start_ros_script(faster_lio_path)
    # apm_start_pid = start_ros_script(apm_path)  
        #camera_start_pid = start_ros_script(camera_path) 
       # started = 1  
    while True:
        # 接收UDP消息
        #print("aaaaaaaaaa")
        #rospy.spin()
        #ready = select.select([udp_socket],[],[],timeout)
        #if ready[0]:
        data, addr = udp_socket.recvfrom(1024)
        message = data.decode()
        print("receive message!!!")
        if message == "start_fasterlio":
            faster_lio_pid = start_ros_script(faster_lio_path)
            if faster_lio_pid:
                #import ipdb;ipdb.set_trace()
                udp_socket.sendto("ROS脚本已启动".encode(), addr)
        elif message == "px4_start":
            #print("bbbbbbbbbbbbbbbbbbbb")
            apm_start_pid = start_ros_script(apm_path)
            #print("cccccccccccccccccccc")
        elif message == "camera_start":
            camera_start_pid = start_ros_script(camera_path)
        elif message == "task_planner_start":
            task_planner_pid = start_ros_script(task_planner_path)
        elif message == "take_off_start":
            start_ros_script(take_off_path)        
        elif message == "stop":
            stop_ros_script(pid)
            udp_socket.sendto("ROS脚本已停止".encode(), addr)
        #elif message == "finish":
            
        # elif message == "apm_stop":
        #     stop_ros_script(apm_start_pid)
        #     print("aaaaaaaaaaaaaaaaa")
        # elif message == "camera_stop":
        #     stop_ros_script(camera_start_pid)
        # elif message == "task_planner_stop":
        #     stop_ros_script(task_planner_pid)
        else:
            udp_socket.sendto("无效命令".encode(), addr)
            
            
            
