
import subprocess

faster_lio_path_without_map = "/home/nv/quzhou-project/sh_files/faster_lio_construct_map.sh"
faster_lio_path = "/home/nv/quzhou-project/sh_files/faster_lio.sh"
task_planner_path = "/home/nv/quzhou-project/sh_files/task_planner.sh"
autopilot_ctrl_path = "/home/nv/quzhou-project/sh_files/apm.sh" 
camera_detect_path = "/home/nv/quzhou-project/sh_files/camera_detect.sh"
waypoint_record_path = "/home/nv/quzhou-project/sh_files/waypoint_record.sh"
rosbag_record_path = "/home/nv/quzhou-project/sh_files/rosbag_record.sh"
capture_image_path = "/home/nv/quzhou-project/sh_files/capture_image.sh"
record_video_path = "/home/nv/quzhou-project/sh_files/record_video.sh"
video_end_path = "/home/nv/quzhou-project/sh_files/video_end.sh"
fasterlio_pid = -100
Autopilot_ctrl_pid = -100
task_planner_pid = -100
rosbag_pid = ''
def start_ros_script(script_path):
    try:
        #process = subprocess.run(['source', '/home/nv/quzhou-project/devel/setup.zsh'], shell=True, check=True)
        process = subprocess.Popen(['zsh', script_path])
        print(f'process_pid = {process.pid}')
        return process.pid 
    except subprocess.CalledProcessError as e:
        print(f"启动ROS脚本时发生错误：{e}")


class ProcessManager:
    def __init__(self) -> None:
        pass

    def do_command(self, command):
        if command == "process_manager_open_faster_lio_with_map":
            self.open_faster_lio("with_map")
        elif command == "process_manager_open_faster_lio_without_map":
            self.open_faster_lio("without_map")
        elif command == "process_manager_open_task_planner":
            self.open_task_planner()
        elif command == "process_manager_open_autopilot_ctrl":
            self.open_autopilot_ctrl()
        elif command == "process_manager_open_camera_detect":
            self.open_camera_detect()
        elif command == "process_capture_image":
            self.capture_image()
        elif command == "process_waypoint_record":
            self.open_waypoint_recorder()
        elif command == "process_rosbag_record_start":
            self.open_rosbag_record()
        elif command == "process_rosbag_record_end":
            self.end_rosbag_record()
        elif command == "process_record_video":
            self.record_video()
        elif command == "process_video_end":
            self.video_end()

    def open_faster_lio(self, value):
        #subprocess.Popen(['killall','roslaunch'])
        global fasterlio_pid
        if(fasterlio_pid != -100):
            fasterlio_pid = str(fasterlio_pid)
            subprocess.Popen(['pkill','-P',fasterlio_pid])
        if value == "with_map":
            fasterlio_pid = start_ros_script(faster_lio_path)
        if value == "without_map":
            fasterlio_pid = start_ros_script(faster_lio_path_without_map)

    def open_task_planner(self, param=None):
        global task_planner_pid
        if(task_planner_pid != -100):
            task_planner_pid = str(task_planner_pid)
            subprocess.Popen(['pkill','-P',task_planner_pid])
        task_planner_pid = start_ros_script(task_planner_path)

    def open_autopilot_ctrl(self, param=None):
        global Autopilot_ctrl_pid
        if(Autopilot_ctrl_pid != -100):
            Autopilot_ctrl_pid = str(Autopilot_ctrl_pid)
            subprocess.Popen(['pkill','-P',Autopilot_ctrl_pid])
        Autopilot_ctrl_pid = start_ros_script(autopilot_ctrl_path)

    def open_camera_detect(self, param=None):
        start_ros_script(camera_detect_path)

    def capture_image(self):
        start_ros_script(capture_image_path)
    def open_waypoint_recorder(self,param=None):
        start_ros_script(waypoint_record_path)
    def open_rosbag_record(self,param=None):
        global rosbag_pid
        rosbag_pid = start_ros_script(rosbag_record_path)
    def end_rosbag_record(self,param=None):
        global rosbag_pid
        rosbag_pid = str(rosbag_pid)
        subprocess.Popen(['pkill','-P',rosbag_pid])
    def record_video(self):
        start_ros_script(record_video_path)
    def video_end(self):
        start_ros_script(video_end_path)
process_manager = ProcessManager()
