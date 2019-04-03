#!/usr/bin/env python
import numpy.matlib
import numpy as np
from math import pi, sin, cos
from sympy import *
import rospy
from trajectory_msgs.msg import JointTrajectory


# Takes DH parameters and joint variables as input
# and returns transformation matrix of forward kinematics
def forward(theta, L, d, a):
    # List of transformation matrices of links
    # Each transformation is calculated with respect to previous link
    M_joints = []
    for i in range(6):
        M_joint_i = Matrix([
            [cos(theta[i]), -sin(theta[i]), 0, L[i]],
            [sin(theta[i]) * cos(a[i]), cos(theta[i]) * cos(a[i]), -sin(a[i]), -sin(a[i]) * d[i]],
            [sin(theta[i]) * sin(a[i]), cos(theta[i]) * sin(a[i]), cos(a[i]), cos(a[i]) * d[i]],
            [0, 0, 0, 1]
        ])
        M_joints.append(M_joint_i)

    # Transformation matrix of last join (Coordinate system {6})
    # with respect to Universal coordinate system {0}
    M = eye(4)
    for i in range(6):
        M = M * M_joints[i]

    return M


def handleJointValues(msg):
    return msg


if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('fwd_kinematics')

    # Subscribe to arm_controller topic to get JointTrajectory values
    sub = rospy.Subscriber('/arm_controller/command', JointTrajectory, handleJointValues)

    # Get DH Parameters from ROS Parameter server or use defaults
    # if parameters don't exist in parameter server
    L = rospy.get_param('/dh_params/L', [0, 0.612, 0.572, 0, 0, 0])
    d = rospy.get_param('/dh_params/d', [0.128, 0, 0, 0.164, 0.116, 0.092])
    a = rospy.get_param('/dh_params/a', [0, -pi/2, 0, 0, pi/2, -pi/2])

    # Joint angles variables
    theta = symbols('th1:7')

    M = forward(theta, L, d, a)
    pprint(M)
    # Test angles
    angles = [pi/4, pi/4, pi/2, pi/2, pi/4, 0]
    M_test = M.subs({ 'th'+str(i+1):angles[i] for i in range(6) })
    pprint(M_test)

    # Position vector in homogeneous coordinates with respect to coordinate system {6}
    p6 = [0, 0, 0, 1]
    # Position vector in homogeneous coordinates with respect to universal coordinate system {0}
    p0 = np.matmul(np.array(M_test).astype(np.float64), p6)
    print(p0)

    rospy.spin()
