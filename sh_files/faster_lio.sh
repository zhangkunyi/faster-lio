#!/bin/zsh   # 这里指定了Bash作为默认Shell
source /home/nv/quzhou-project/devel/setup.zsh

# fcu
# sudo chmod 777 /dev/ttyACM0 & sleep 1;
# roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 & sleep 1;
sudo chmod 777 /dev/ttyTHS0 & sleep 1;
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS0:921600 & sleep 1;
rosrun mavros mavsys message_interval --id=245   --rate=5;
rosrun mavros mavsys message_interval --id=147   --rate=5;
rosrun mavros mavsys message_interval --id=27    --rate=200;
rosrun mavros mavsys message_interval --id=31    --rate=200;
rosrun mavros mavsys message_interval --id=65    --rate=100;

rosrun mavros mavsys message_interval --id=385   --rate=200;
rosrun mavros mavsys message_interval --id=11030 --rate=200;
rosrun mavros mavsys message_interval --id=100 --rate=100;
rosrun mavros mavsys message_interval --id=132 --rate=100;
# https://mavlink.io/en/messages/common.html
# https://github1s.com/mavlink/mavlink/blob/HEAD/message_definitions/v1.0/ardupilotmega.xml

# localization
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 5;
roslaunch faster_lio mapping_mid360.launch rviz:=true localization_mode:=true & sleep 5;

# # ekf
roslaunch ekf_fuser ekf_fuser.launch & sleep 1;

wait;
