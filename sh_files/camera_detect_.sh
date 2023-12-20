source ./devel/setup.zsh & sleep 1;

roslaunch camera_detect camera_test.launch & sleep 1;

if [ "$1" = "true" ]; then

roslaunch aruco_ros single_test_copy.launch markerId:=582 markerSize:=0.15 & sleep 3;
rosrun image_view image_view image:=/aruco_single/result

else

roslaunch aruco_ros single_test_copy.launch markerId:=582 markerSize:=0.15;
fi

wait;
