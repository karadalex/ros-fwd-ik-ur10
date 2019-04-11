#!/usr/bin/env python
from fwd_kinematics_sym import *
from inv_kinematics_sym import *
import rospy
from trajectory_msgs.msg import JointTrajectory


def handleForwardKinematics(msg):
    # TODO
    return msg

def handleInverseKinematics(msg):
    # TODO
    return msg

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('fwd_kinematics')

    # Subscribe to arm_controller topic to get JointTrajectory values
    sub = rospy.Subscriber('/arm_controller/command', JointTrajectory, handleForwardKinematics)

    # Get DH Parameters from ROS Parameter server or use defaults
    # if parameters don't exist in parameter server
    L = rospy.get_param('/dh_params/L', [0, 0.612, 0.572, 0, 0, 0])
    d = rospy.get_param('/dh_params/d', [0.128, 0, 0, 0.164, 0.116, 0.092])
    a = rospy.get_param('/dh_params/a', [0, -pi/2, 0, 0, pi/2, -pi/2])


    rospy.spin()
