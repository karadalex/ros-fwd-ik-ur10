#!/bin/bash

# setup gazebo environment
source "/usr/share/gazebo/setup.sh"
gzserver --verbose & npm start & 

source /opt/ros/kinetic/setup.bash
mkdir -p /ros_ws/src
cd /ros_ws
catkin_make
source devel/setup.bash
# run gzserver and gzweb
exec "$@"