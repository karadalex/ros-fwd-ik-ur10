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
    th = symbols('th1:7')
    solutionSet = { k:[] for k in th }

    # Sets of known and unknown variables
    known = {ix,iy,iz,jx,jy,jz,kx,ky,kz,px,py,pz}
    unknown = set(th)
    solved = set()

    for epoch in range(1):
        lhs = Matrix([
            [ix, jx, kx, px],
            [iy, jy, ky, py],
            [iz, jz, kz, pz],
            [0, 0, 0, 1]    
        ])
        rhs = M0_6

        for i in range(5):
            lhs = invTransformation(M_joints[i]) * lhs
            rhs = invTransformation(M_joints[i]) * rhs
            # rhs = eye(4)
            # for j in range(i+1,6):
            #     rhs = rhs * M_joints[j]
            equations = lhs - rhs

            for j in range(3):
                for k in range(4):
                    eq = trigsimp(equations[j,k])
                    variables = eq.free_symbols
                    variables = variables.difference(known)
                    variables = variables.difference(solved)
                    print(eq)
                    print(variables)
                    print("\n")
                    if len(variables) == 1:
                        varToBeSolved = variables.pop()

                        print("Solving for "+str(varToBeSolved)+", equation:")
                        pprint(eq)
                        
                        # if (varToBeSolved == th[0] or varToBeSolved == th[1] or varToBeSolved == th[2]) and epoch < 3 : 
                        #     # Solving for ... takes too long!
                        #     solutionSet[varToBeSolved].append(eq)
                        # else:
                        solutions = solve(eq, varToBeSolved)
                        for sol in solutions:
                            print(sol)
                            solutionSet[varToBeSolved].append(sol)
                        
                        solved.add(varToBeSolved)
        
        print("\n")

    return solutionSet

if __name__ == "__main__":
    
    # Compute forward kinematics
    L = [0, 0.612, 0.572, 0, 0, 0]
    d = [0.128, 0, 0, 0.164, 0.116, 0.092]
    a = [0, -pi/2, 0, 0, pi/2, -pi/2]
    theta = symbols('th1:7')
    M0_6, M_joints = forward(theta, L, d, a)

    ikSolutions = inverseKin(M0_6, M_joints)
    print(ikSolutions)

    # Test angles
    angles = [pi/4, pi/4, pi/2, pi/2, pi/4, 0]
    M_test = M0_6.subs({ 'th'+str(i+1):angles[i] for i in range(6) })
    pprint(M_test)

    ix,iy,iz = M_test[0,0], M_test[1,0], M_test[2,0]
    jx,jy,jz = M_test[0,1], M_test[1,1], M_test[2,1]
    kx,ky,kz = M_test[0,2], M_test[1,2], M_test[2,2]
    px,py,pz = M_test[0,3], M_test[1,3], M_test[2,3]
    knownsDict = {'ix':ix,'iy':iy,'iz':iz,'jx':jx,'jy':jy,'jz':jz,'kx':kx,'ky':ky,'kz':kz,'px':px,'py':py, 'pz':pz}
    
    # th5
    print("Solutions for th5")
    th5_1 = ikSolutions[Symbol('th5')][0].subs(knownsDict).evalf()
    th5_2 = ikSolutions[Symbol('th5')][1].subs(knownsDict).evalf()
    print(th5_1, th5_2)

    # th1
    print("Solutions for th1")
    th1_1 = ikSolutions[Symbol('th1')][0].subs(knownsDict).subs({'th5':th5_1}).evalf()
    # th1_2 = ikSolutions[Symbol('th1')][0].subs(knownsDict).subs({'th5':th5_2}).evalf()
    # th1_3 = ikSolutions[Symbol('th1')][1].subs(knownsDict).subs({'th5':th5_1}).evalf()
    # th1_4 = ikSolutions[Symbol('th1')][1].subs(knownsDict).subs({'th5':th5_2}).evalf()
    print(th1_1)

    # th4
    print("Solutions for th4")
    # th4_1 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({'th5':th5_2, 'th1':th1_2}).evalf()
    # th4_2 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({'th5':th5_2, 'th1':th1_4}).evalf()
    # th4_3 = ikSolutions[Symbol('th4')][1].subs(knownsDict).subs({'th5':th5_2, 'th1':th1_2}).evalf()
    # th4_4 = ikSolutions[Symbol('th4')][1].subs(knownsDict).subs({'th5':th5_2, 'th1':th1_4}).evalf()
    # print(th4_1)

    # th6

    # th3

    # th2
    
    