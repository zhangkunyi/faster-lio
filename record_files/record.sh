source ./devel/setup.zsh & sleep 1;
rosbag record --tcpnodelay \
  /position_cmd \
  /ekf/ekf_odom_lidar \
  /mavros/setpoint_raw/attitude \

# /camera/color/camera_info \
# /camera/color/image_raw \
# /camera/depth/image_rect_raw \
# /camera/depth/camera_info \
# /camera/infra1/camera_info \
# /camera/infra1/image_rect_raw \
# /camera/infra2/camera_info \
# /camera/infra2/image_rect_raw \
# /camera/extrinsics/depth_to_color \
# /camera/extrinsics/depth_to_infra1 \
# /camera/extrinsics/depth_to_infra2 \
# /camera/realsense2_camera_manager/bond \
# /livox/imu \
# /livox/lidar \
# /mavros/imu/data \
# /mavros/imu/data_raw \

