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

# livox massage
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 5;

# realsense message
roslaunch realsense2_camera rs_record.launch & sleep 5;

wait;
