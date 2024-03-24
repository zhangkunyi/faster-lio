source devel/setup.zsh & sleep 1;

roslaunch realsense2_camera rs_camera_vins.launch & sleep 10;

#roslaunch ros_rtsp rtsp_streams.launch;

wait;


