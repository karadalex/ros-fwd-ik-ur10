Forward & Inverse Kinematics, Trajectories, Visualizations in ROS
==================================================================

## Introduction

This is a project for the Robotics course at University if Patras (Spring Semester 2019)

## Requirements

1. A working ROS environment

## Setup ROS with Vagrant

Setup a fully configured ROS Virtual Machine with Vagrant and Virtualbox
- Install Vagrant
- Bootup ROS VM with `vagrant up`

## Instructions

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

## Developers

1. Qt Creator
2. PySide2 `pip install PySide2`
