Forward & Inverse Kinematics, Trajectories, Visualizations in ROS
==================================================================

## Introduction

This is a project for the Robotics course at University if Patras (Spring Semester 2019)

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

## Python Virtual environments

```
cd kinematics_py
virtualenv -p /usr/bin/python3.6 venv
source venv/bin/activate
pip install -r requirements.txt
```
