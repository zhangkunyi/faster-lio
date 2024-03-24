#!/bin/zsh   # 这里指定了Bash作为默认Shell
source /home/nv/quzhou-project/devel/setup.zsh

roslaunch px4ctrl run_ctrl.launch & sleep 1;

wait;
