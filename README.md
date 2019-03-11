Forward & Inverse Kinematics, Trajectories, Visualizations in ROS
==================================================================

## Introduction

This is a project for the Robotics course at University if Patras (Spring Semester 2019).
ROS Forward, Inverse Kinematics, Trajectory planning, PtP-T, UR10 Robot

## Requirements

1. A working ROS environment
2. Gazebo Installed

## Run with Docker (Recommended)

To follow these instructions you must have installed:
- docker
- docker-compose

```
docker-compose up
```

## Setup ROS with Vagrant

Setup a fully configured ROS Virtual Machine with Vagrant and Virtualbox
- Install Vagrant
- Bootup ROS VM with `vagrant up`


## Instructions

Instructions for running packages locally or in the Vagrant VM

1. Create a ROS workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

2. Activate workspace environment 
```
source devel/setup.bash
```

3. Go to src folder, clone this repository there and set it up
```
cd ~/catkin_ws/src
git clone <THIS_REPOSITORY_URL> .
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

4. Install ROS dependencies
```
sudo apt-get install ros-melodic-tf2-sensor-msgs
sudo apt-get install -y ros-melodic-moveit
```

## User guide

To use the ROS docker containers with your local ROS environment run
```
export ROS_MASTER_URI=http://localhost:11311
```
and similarly to connect to the gazebo server and access the simulation locally run
```
export GAZEBO_MASTER_URI=localhost:11345
gzclient
```
The Gazebo instance running in the Docker container and your local Gazebo instance as well as
the ROS environments, must be of matching versions.

Get DH Parameters from ROS Parameter server
```
rosparam get dh_params
```
Connect to one of the ROS containers and run a simple movement for the UR10 robot
```
docker exec -it <CONTAINER_ID> bash
source /ros_ws/devel/setup.bash
rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], points: [{positions: [-3.14, -0.5, 0.7, 0.5, 0.0, 0.0], time_from_start: [1.0, 0.0]}]}'
```

## Python Virtual environments

```
cd kinematics_py
virtualenv -p /usr/bin/python3.6 venv
source venv/bin/activate
pip install -r requirements.txt
```

## JetBrains PyCharm

To configure PyCharm IDE to detect and use **rospy** library
1. In your catkin_workspace run `source devel/setup.bash`
2. After the ROS environment is enabled, start PyCharm from the same terminal `charm .`
3. It is recommended that you configure PyCharm to use a separate python environment

## Troubleshooting

## Notices

1. UR10 models are from [universal_robot](https://github.com/ros-industrial/universal_robot)