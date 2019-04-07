#!/usr/bin/env python
import numpy.matlib
import numpy as np
from sympy import *

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

    return M, M_joints


if __name__ == "__main__":
    
    L = [0, 0.612, 0.572, 0, 0, 0]
    d = [0.128, 0, 0, 0.164, 0.116, 0.092]
    a = [0, -pi/2, 0, 0, pi/2, -pi/2]

    # Joint angles variables
    theta = symbols('th1:7')

    M, _ = forward(theta, L, d, a)
    pprint(M)
    # Test angles
    angles = [pi/4, pi/4, pi/2, pi/2, pi/4, 0]
    M_test = M.subs({ 'th'+str(i+1):angles[i] for i in range(6) })
    pprint(M_test)
    print(latex(M_test))
    print(latex(M))

    # Position vector in homogeneous coordinates with respect to coordinate system {6}
    p6 = [0, 0, 0, 1]
    # Position vector in homogeneous coordinates with respect to universal coordinate system {0}
    p0 = np.matmul(np.array(M_test).astype(np.float64), p6)
    print(p0)
    print(latex(p0))
