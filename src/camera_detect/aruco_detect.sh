#! /bin/sh
if [ "$1" = "true" ]; then
roslaunch aruco_ros single_test.launch markerId:=582 markerSize:=0.135 & sleep 10
rosrun image_view image_view image:=/aruco_single/result
else
roslaunch aruco_ros single_test.launch markerId:=582 markerSize:=0.135
fi
