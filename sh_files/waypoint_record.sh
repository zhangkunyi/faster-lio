#!/bin/zsh   # 这里指定了Bash作为默认Shell
source /home/nv/quzhou-project/devel/setup.zsh

roslaunch waypoints_recorder waypoints_recorder.launch & sleep 1;

wait;
