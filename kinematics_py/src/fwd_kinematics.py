#!/usr/bin/env python
import numpy.matlib
import numpy as np
from math import pi, sin, cos
import rospy


# Takes DH parameters and joint variables as input
# and returns transformation matrix of forward kinematics
def forward(theta, L, d, a):
    # List of transformation matrices of links
    # Each transformation is calculated with respect to previous link
    M_joints = np.zeros((6, 4, 4))
    for i in range(6):
        M_joints[i] = [
            [cos(theta[i]), -sin(theta[i]), 0, L[i]],
            [sin(theta[i]) * cos(a[i]), cos(theta[i]) * cos(a[i]), -sin(a[i]), -sin(a[i]) * d[i]],
            [sin(theta[i]) * sin(a[i]), cos(theta[i]) * sin(a[i]), cos(a[i]), cos(a[i]) * d[i]],
            [0, 0, 0, 1]
        ]

    # Transformation matrix of last join (Coordinate system {6})
    # with respect to Universal coordinate system {0}
    M = np.matlib.identity(4)
    for i in range(6):
        M = np.matmul(M, M_joints[i])

    return M


if __name__ == "__main__":
    # Get DH Parameters from ROS Parameter server or use defaults
    # if parameters don't exist in parameter server
    L = rospy.get_param('/dh_params/L', [0, -0.612, -0.5723, 0, 0, 0])
    d = rospy.get_param('/dh_params/d', [0.1273, 0, 0, 0.163941, 0.1157, 0.0922])
    a = rospy.get_param('/dh_params/a', [pi/2, 0, 0, pi/2, -pi/2, 0])

    # Joint angles variables
    # theta = np.zeros(6)
    theta = [pi/4, pi/4, pi/2, pi/2, pi/4, 0]

    M = forward(theta, L, d, a)

    # Position vector in homogeneous coordinates with respect to coordinate system {6}
    p6 = [0, 0, 0, 1]

    # Position vector in homogeneous coordinates with respect to universal coordinate system {0}
    p0 = np.matmul(M, p6)

    print(M)
    print(p6)
    print(p0)
