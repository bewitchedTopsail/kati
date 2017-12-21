#!/bin/bash

unset ROS_IP
export ROS_IP=$(hostname -I)

source ~/catkin_ws/devel/setup.bash

exec "$@"



