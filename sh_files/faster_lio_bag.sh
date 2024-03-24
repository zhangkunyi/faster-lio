source devel/setup.zsh & sleep 1;

# fcu
roslaunch faster_lio mapping_mid360.launch rviz:=true localization_mode:=false & sleep 5;

# # ekf
roslaunch ekf_fuser ekf_fuser.launch & sleep 1;

wait;
