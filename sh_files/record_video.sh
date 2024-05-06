#!/bin/zsh
rostopic pub -1 /record_video std_msgs/UInt8 "data: 0" ;
wait;
