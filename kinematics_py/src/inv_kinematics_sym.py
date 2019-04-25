#!/usr/bin/env python
import numpy.matlib
import numpy as np
from math import pi, sin, cos
from sympy import *
from fwd_kinematics_sym import forward


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

    targetTransf = Matrix([
        [ix, jx, kx, px],
        [iy, jy, ky, py],
        [iz, jz, kz, pz],
        [0, 0, 0, 1]    
    ])

    # Solve for th1
    lhs = invTransformation(M_joints[0]) * targetTransf * invTransformation(M_joints[5])
    rhs = eye(4)
    for i in range(1,5):
        rhs = rhs * M_joints[i]
    equations = lhs - rhs
    for j in range(3):
        for k in range(4):
            eq = equations[j,k].rewrite(tan)
            variables = eq.free_symbols
            variables = variables.difference(known)
            variables = variables.difference(solved)
            pprint(eq)
            print(variables)
            print("\n")
            if len(variables) == 1:
                varToBeSolved = variables.pop()
                print("Solving for "+str(varToBeSolved)+", equation:")
                pprint(eq)
                solutions = solve(eq, varToBeSolved)
                for sol in solutions:
                    print(sol)
                    solutionSet[varToBeSolved].append(sol)
                
                solved.add(varToBeSolved)
                # break double loop
                j,k = 3, 4
    
    # Solve for th5 and then th6
    lhs = invTransformation(M_joints[0]) * targetTransf
    rhs = eye(4)
    for i in range(1,6):
        rhs = rhs * M_joints[i]
    equations = lhs - rhs
    for epoch in range(2):
        for j in range(3):
            for k in range(4):
                eq = equations[j,k].rewrite(tan)
                variables = eq.free_symbols
                variables = variables.difference(known)
                variables = variables.difference(solved)
                pprint(eq)
                print(variables)
                print("\n")
                if len(variables) == 1:
                    varToBeSolved = variables.pop()
                    print("Solving for "+str(varToBeSolved)+", equation:")
                    pprint(eq)
                    solutions = solve(eq, varToBeSolved)
                    for sol in solutions:
                        print(sol)
                        solutionSet[varToBeSolved].append(sol)
                    
                    solved.add(varToBeSolved)
                    # break double loop
                    j,k = 3, 4

    # Solve for th2
    lhs = targetTransf * invTransformation(M_joints[5]) * invTransformation(M_joints[4]) * invTransformation(M_joints[3])
    rhs = eye(4)
    for i in range(0,3):
        rhs = rhs * M_joints[i]
    equations = lhs - rhs
    for j in range(3):
        for k in range(4):
            eq = trigsimp(equations[j,k])
            variables = eq.free_symbols
            variables = variables.difference(known)
            variables = variables.difference(solved)
            pprint(eq)
            print(variables)
            print("\n")
            if len(variables) == 1:
                varToBeSolved = variables.pop()
                print("Solving for "+str(varToBeSolved)+", equation:")
                pprint(eq)
                solutions = solve(eq, varToBeSolved)
                for sol in solutions:
                    print(simplify(sol))
                    solutionSet[varToBeSolved].append(sol)
                
                solved.add(varToBeSolved)
                # break double loop
                print(j, k)
                j,k = 3, 4

    # Solve for th3, th4
    lhs = invTransformation(M_joints[0]) * targetTransf * invTransformation(M_joints[5]) * invTransformation(M_joints[4]) * invTransformation(M_joints[3])
    rhs = eye(4)
    for i in range(1,3):
        rhs = rhs * M_joints[i]
    equations = lhs - rhs
    for j in range(3):
        for k in range(4):
            eq = trigsimp(equations[j,k])
            variables = eq.free_symbols
            variables = variables.difference(known)
            variables = variables.difference(solved)
            c1,s1,c2,s2,c5,s5,c6,s6 = symbols("c1 s1 c2 s2 c5 s5 c6 s6")
            t3,t4 = symbols("t3 t4")
            thTot = {th[2]:t3, th[3]:t4}
            eq = eq.subs({
                cos(th[0]):c1, sin(th[0]):s1,
                cos(th[1]):c2, sin(th[1]):s2,
                cos(th[4]):c5, sin(th[4]):s5,
                cos(th[5]):c6, sin(th[5]):s6
            })
            eq = simplify(eq)
            print(j,k)
            pprint(eq)
            print(variables)
            print("\n")
            if len(variables) == 1:
                thVar = variables.pop()
                tVar = thTot[thVar]
                eq = eq.rewrite(tan).subs({
                    tan(thVar/2):tVar
                })
                print("Solving for "+str(tVar)+", equation:")
                pprint(eq)
                solutions = solve(eq, tVar)
                for sol in solutions:
                    sol = sol.subs({
                        c1:cos(th[0]), s1:sin(th[0]),
                        c2:cos(th[1]), s2:sin(th[1]),
                        c5:cos(th[4]), s5:sin(th[4]),
                        c6:cos(th[5]), s6:sin(th[5])
                    })
                    # convert back to th
                    sol = trigsimp(2*atan(sol))
                    print(sol)
                    solutionSet[thVar].append(sol)
                
                solved.add(thVar)
                # break double loop
                print(j, k)
                j,k = 3, 4
    
    print("\n")

    return solutionSet

if __name__ == "__main__":
    
    # Compute forward kinematics
    L = [0, 0.612, 0.572, 0, 0, 0]
    d = [0, 0, 0, 0.164, 0.116, 0.092]
    a = [0, -pi/2, 0, 0, pi/2, -pi/2]
    theta = symbols('th1:7')
    M0_6, M_joints = forward(theta, L, d, a)

    ikSolutions = inverseKin(M0_6, M_joints)
    print(ikSolutions)

    # Test angles
    # angles = [pi/4, pi/4, pi/2, pi/2, pi/4, 0]
    angles = [8*pi/9, pi/4, pi/2, 0.7, 1.36, 0.5]
    M_test = M0_6.subs({ 'th'+str(i+1):angles[i] for i in range(6) })
    pprint(M_test)

    ix,iy,iz = M_test[0,0], M_test[1,0], M_test[2,0]
    jx,jy,jz = M_test[0,1], M_test[1,1], M_test[2,1]
    kx,ky,kz = M_test[0,2], M_test[1,2], M_test[2,2]
    px,py,pz = M_test[0,3], M_test[1,3], M_test[2,3]
    knownsDict = {'ix':ix,'iy':iy,'iz':iz,'jx':jx,'jy':jy,'jz':jz,'kx':kx,'ky':ky,'kz':kz,'px':px,'py':py, 'pz':pz}
    
    # th1
    print("Solutions for th1")
    th1_1 = ikSolutions[Symbol('th1')][0].subs(knownsDict).evalf()
    th1_2 = ikSolutions[Symbol('th1')][1].subs(knownsDict).evalf()
    print(th1_1)
    print(th1_2)
    print("\n")

    # th5
    print("Solutions for th5")
    th5_1 = ikSolutions[Symbol('th5')][0].subs(knownsDict).subs({'th1':th1_1}).evalf()
    th5_2 = ikSolutions[Symbol('th5')][0].subs(knownsDict).subs({'th1':th1_1}).evalf()
    th5_3 = ikSolutions[Symbol('th5')][1].subs(knownsDict).subs({'th1':th1_2}).evalf()
    th5_4 = ikSolutions[Symbol('th5')][1].subs(knownsDict).subs({'th1':th1_2}).evalf()
    print(th5_1)
    print(th5_2)
    print(th5_3)
    print(th5_4)
    print("\n")

    # th6
    print("Solutions for th6")
    th6_1 = ikSolutions[Symbol('th6')][0].subs(knownsDict).subs({'th1':th1_1, 'th5':th5_1}).evalf()
    th6_2 = ikSolutions[Symbol('th6')][0].subs(knownsDict).subs({'th1':th1_1, 'th5':th5_2}).evalf()
    th6_3 = ikSolutions[Symbol('th6')][0].subs(knownsDict).subs({'th1':th1_2, 'th5':th5_3}).evalf()
    th6_4 = ikSolutions[Symbol('th6')][0].subs(knownsDict).subs({'th1':th1_2, 'th5':th5_4}).evalf()
    th6_5 = ikSolutions[Symbol('th6')][1].subs(knownsDict).subs({'th1':th1_1, 'th5':th5_1}).evalf()
    th6_6 = ikSolutions[Symbol('th6')][1].subs(knownsDict).subs({'th1':th1_1, 'th5':th5_2}).evalf()
    th6_7 = ikSolutions[Symbol('th6')][1].subs(knownsDict).subs({'th1':th1_2, 'th5':th5_3}).evalf()
    th6_8 = ikSolutions[Symbol('th6')][1].subs(knownsDict).subs({'th1':th1_2, 'th5':th5_4}).evalf()
    print(th6_1)
    print(th6_2)
    print(th6_3)
    print(th6_4)
    print(th6_5)
    print(th6_6)
    print(th6_7)
    print(th6_8)
    print("\n")

    # th4
    print("Solutions for th4")
    th4_1 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_1, 'th6':th6_1
    }).evalf()
    th4_2 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_2, 'th6':th6_2
    }).evalf()
    th4_3 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_3, 'th6':th6_3
    }).evalf()
    th4_4 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_4, 'th6':th6_4
    }).evalf()
    th4_5 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_1, 'th6':th6_5
    }).evalf()
    th4_6 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_2, 'th6':th6_6
    }).evalf()
    th4_7 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_3, 'th6':th6_7
    }).evalf()
    th4_8 = ikSolutions[Symbol('th4')][0].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_4, 'th6':th6_8
    }).evalf()
    print(th4_1)
    print(th4_2)
    print(th4_3)
    print(th4_4)
    print(th4_5)
    print(th4_6)
    print(th4_7)
    print(th4_8)
    print("\n")

    # th3
    print("Solutions for th3")
    th3_1 = ikSolutions[Symbol('th3')][1].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_1, 'th6':th6_1
    }).evalf()
    th3_2 = ikSolutions[Symbol('th3')][1].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_2, 'th6':th6_2
    }).evalf()
    th3_3 = ikSolutions[Symbol('th3')][1].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_3, 'th6':th6_3
    }).evalf()
    th3_4 = ikSolutions[Symbol('th3')][1].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_4, 'th6':th6_4
    }).evalf()
    th3_5 = ikSolutions[Symbol('th3')][1].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_1, 'th6':th6_5
    }).evalf()
    th3_6 = ikSolutions[Symbol('th3')][1].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_2, 'th6':th6_6
    }).evalf()
    th3_7 = ikSolutions[Symbol('th3')][1].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_3, 'th6':th6_7
    }).evalf()
    th3_8 = ikSolutions[Symbol('th3')][1].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_4, 'th6':th6_8
    }).evalf()
    print(th3_1)
    print(th3_2)
    print(th3_3)
    print(th3_4)
    print(th3_5)
    print(th3_6)
    print(th3_7)
    print(th3_8)
    print("\n")

    # th2
    print("Solutions for th2")
    th2_1 = ikSolutions[Symbol('th2')][1].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_1, 'th6':th6_1
    }).evalf()
    th2_2 = ikSolutions[Symbol('th2')][1].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_2, 'th6':th6_2
    }).evalf()
    th2_3 = ikSolutions[Symbol('th2')][1].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_3, 'th6':th6_3
    }).evalf()
    th2_4 = ikSolutions[Symbol('th2')][1].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_4, 'th6':th6_4
    }).evalf()
    th2_5 = ikSolutions[Symbol('th2')][1].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_1, 'th6':th6_5
    }).evalf()
    th2_6 = ikSolutions[Symbol('th2')][1].subs(knownsDict).subs({
        'th1':th1_1, 'th5':th5_2, 'th6':th6_6
    }).evalf()
    th2_7 = ikSolutions[Symbol('th2')][1].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_3, 'th6':th6_7
    }).evalf()
    th2_8 = ikSolutions[Symbol('th2')][1].subs(knownsDict).subs({
        'th1':th1_2, 'th5':th5_4, 'th6':th6_8
    }).evalf()
    print(th2_1)
    print(th2_2)
    print(th2_3)
    print(th2_4)
    print(th2_5)
    print(th2_6)
    print(th2_7)
    print(th2_8)
    print("\n")
    
    