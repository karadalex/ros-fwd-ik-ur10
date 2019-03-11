#!/bin/bash

# setup gazebo environment
source "/usr/share/gazebo/setup.sh"
# gzserver --verbose & npm start &
npm start &

source /opt/ros/kinetic/setup.bash
mkdir -p /ros_ws/src
cd /ros_ws
catkin_make
source devel/setup.bash

# Copy volume containing meshes in gazebo web server assets folder
cp -r /ros_ws/src/ur_description ~/gzweb/http/client/assets

# Entrypoint for command
exec "$@"