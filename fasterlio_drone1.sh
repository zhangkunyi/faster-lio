#source ./devel/setup.zsh & sleep 1;


sudo chmod 777 /dev/tty* & sleep 1;


#roslaunch livox_ros_driver2 msg_MID360.launch & sleep 5;
#roslaunch faster_lio mapping_mid360_drone1.launch & sleep 5;

roslaunch ekf_fuser ekf_fuser_drone1.launch  localization_mode:=true & sleep 1;
#roslaunch ekf_fuser ekf_fuser_drone1.launch   & sleep 1;

wait;
