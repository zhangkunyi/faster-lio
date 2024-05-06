#!/bin/zsh
rostopic pub -1 /capture_image std_msgs/UInt8 "data: 0" ;
wait;
