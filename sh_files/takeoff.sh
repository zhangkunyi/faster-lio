source ./devel/setup.zsh & sleep 1;
rostopic pub -1  /autopilot_ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"

