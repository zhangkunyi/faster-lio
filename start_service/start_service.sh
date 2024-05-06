#!/bin/zsh
P_HOME=$(cd "$(dirname "$0")";pwd)
echo "P_HOME=$P_HOME"
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
export PATH=$PATH:
source /opt/ros/noetic/setup.zsh
roscore & sleep 2;

python3  /home/nv/quzhou-project/start_service/start_server.py & sleep 2;
python3 /home/nv/mqtt/ros2string.py
