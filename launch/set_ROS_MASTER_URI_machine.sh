#!/bin/bash

export ROS_MASTER_URI=http://<IP>:11311

source ~/catkin_ws/devel/setup.bash

exec "$@"
