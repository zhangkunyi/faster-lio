#!/bin/zsh
rostopic pub -1 /video_end std_msgs/UInt8 "data: 0" ;
wait;
