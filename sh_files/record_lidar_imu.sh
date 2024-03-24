source ./devel/setup.zsh & sleep 1;
rosbag record --tcpnodelay \
/livox/imu \
/livox/lidar \
/mavros/imu/data \
/mavros/imu/data_raw \

