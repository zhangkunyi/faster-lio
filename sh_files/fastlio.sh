source devel/setup.zsh & sleep 1;

# fcu
# sudo chmod 777 /dev/ttyACM0 & sleep 1;
# roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 & sleep 1;
sudo chmod 777 /dev/ttyTHS0 & sleep 1;
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS0:921600 & sleep 1;
rosrun mavros mavsys message_interval --id=245   --rate=5;
rosrun mavros mavsys message_interval --id=147   --rate=5;
rosrun mavros mavsys message_interval --id=27    --rate=200;
rosrun mavros mavsys message_interval --id=31    --rate=200;
rosrun mavros mavsys message_interval --id=36    --rate=200;
rosrun mavros mavsys message_interval --id=65    --rate=100;
rosrun mavros mavsys message_interval --id=385   --rate=200;
rosrun mavros mavsys message_interval --id=11030 --rate=200;
# https://mavlink.io/en/messages/common.html
# https://github1s.com/mavlink/mavlink/blob/HEAD/message_definitions/v1.0/ardupilotmega.xml

# localization
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 5;
roslaunch fast_lio mapping_mid360.launch & sleep 5;

# # ekf
roslaunch ekf imu_and_lidar.launch & sleep 1;

# ctrl
# roslaunch px4ctrl run_ctrl.launch & sleep 1;


# # ego_planner
# roslaunch ego_planner run_lidar.launch & sleep 2;
# roslaunch ego_planner rviz.launch & sleep 2;


# traj
# roslaunch waypoint_trajectory_generator traj_gen.launch & sleep 1;

# monitor
# rosrun rviz rviz -d /home/fast/demo_ws/src/control_test/waypoint_trajectory_generator/launch/rviz_config/test_traj.rviz & sleep 1;

wait;
