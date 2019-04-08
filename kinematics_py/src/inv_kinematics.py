#!/usr/bin/env python
import numpy.matlib
import numpy as np
from math import pi, sin, cos
from sympy import *
from fwd_kinematics import forward


# M must be 4x4
def invTransformation(M):
    R = M[0:3, 0:3]
    p = M[0:3, 3:4]
    invM = eye(4)
    invM[0:3, 0:3] = R.T
    invM[0:3, 3:4] = -R.T*p
    return invM

def inverseKin(M0_6, M_joints):
    ix,iy,iz,jx,jy,jz,kx,ky,kz,px,py,pz = symbols("ix iy iz jx jy jz kx ky kz px py pz")
    lhs = Matrix([
        [ix, jx, kx, px],
        [iy, jy, ky, py],
        [iz, jz, kz, pz],
        [0, 0, 0, 1]    
    ])

    rhs = M0_6
    qi = []
    th = symbols('th1:7')

    # Sets of known and unknown variables
    known = {ix,iy,iz,jx,jy,jz,kx,ky,kz,px,py,pz}
    unknown = set(th)
    solved = set()

    for i in range(5):
        lhs = lhs * invTransformation(M_joints[i]).T
        rhs = eye(4)
        for j in range(i+1,6):
            rhs = rhs * M_joints[j]
        eq = lhs - rhs
        pprint(lhs)
        pprint(rhs)
        for j in range(3):
            for k in range(4):
                vars = eq[j,k].free_symbols
                vars = vars.difference(known)
                vars = vars.difference(solved)
                print(vars)
                if len(vars) == 1:
                    varToBeSolved = vars.pop()
                    qi.append({varToBeSolved:solve(eq[j,k], varToBeSolved)})
                    solved.add(varToBeSolved)
        print("\n")


    print(qi)
    print(len(qi))


    return (1,2,3,1)

if __name__ == "__main__":
    
    # Compute forward kinematics
    L = [0, 0.612, 0.572, 0, 0, 0]
    d = [0.128, 0, 0, 0.164, 0.116, 0.092]
    a = [0, -pi/2, 0, 0, pi/2, -pi/2]
    theta = symbols('th1:7')
    M0_6, M_joints = forward(theta, L, d, a)

    print(inverseKin(M0_6, M_joints))
    