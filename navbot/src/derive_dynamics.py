#!/usr/bin/env python3
import numpy as np
import sympy as sym

def derivative(f,x):
    return sym.Matrix([f]).jacobian(x)

phi, p, theta, q, psi, r = sym.symbols('phi p theta q psi r')
x, Vx, y, Vy, z, Vz = sym.symbols('x Vx y Vy z Vz')
u1, u2, u3, u4 = sym.symbols('u1 u2 u3 u4')
Jx, Jy, Jz, m, g, L, c = sym.symbols('Jx, Jy, Jz, m, g, L, c')
subber = {x:1, y:2, z:3, Vx:4, Vy:5, Vz:6, phi:7, theta:8, psi:9, p:10, q:11, r:12, Jx:1, Jy:1, Jz:1, g:-10,c:1,L:1,u1:13,u2:14,u3:15,u4:16}


X = [x,y,z,Vx,Vy,Vz,phi,theta,psi,p,q,r]
U = [u1,u2,u3,u4]

J = [[Jx, 0, 0],
     [0, Jy, 0],
     [0, 0, Jz]]
J = sym.Matrix(J)

e3 = [[0],[0],[1]]
e3 = sym.Matrix(e3)

R = [ [sym.cos(theta)*sym.cos(psi), sym.cos(theta)*sym.sin(psi), -sym.sin(theta)],
      [sym.sin(theta)*sym.cos(psi)*sym.sin(phi) - sym.sin(psi)*sym.cos(phi), sym.sin(theta)*sym.sin(psi)*sym.sin(phi) + sym.cos(psi)*sym.cos(phi), sym.cos(theta)*sym.sin(phi)],
      [sym.sin(theta)*sym.cos(psi)*sym.cos(phi) + sym.sin(psi)*sym.sin(phi), sym.sin(theta)*sym.sin(psi)*sym.cos(phi) - sym.cos(psi)*sym.sin(phi), sym.cos(theta)*sym.cos(phi)]]
R = sym.Matrix(R).subs(subber)

W = [ [1, sym.sin(phi)*sym.tan(theta), sym.cos(phi)*sym.tan(theta)],
      [0, sym.cos(phi), -sym.sin(phi)],
      [0, sym.sin(phi)/sym.cos(theta), sym.cos(phi)/sym.cos(theta)]]
W = sym.Matrix(W)

V_B = sym.Matrix([[Vx], [Vy], [Vz]]).subs(subber)

Omega = sym.Matrix([[p], [q], [r]]).subs(subber)

p_dot = R.T@V_B
V_Bdot = np.cross(-Omega,V_B,axis=0) + g*R@e3
Theta_dot = W@Omega
Omega_dot = J.inv()@(np.cross(-Omega,J@Omega,axis=0))
filler = np.zeros((4,1))

Bc = [[0,0,0,0],
      [0,0,0,0],
      [0,0,0,0],
      [0,0,0,0],
      [0,0,0,0],
      [-1/m,-1/m,-1/m,-1/m],
      [0,0,0,0],
      [0,0,0,0],
      [0,0,0,0],
      [0,-L/Jx,0,L/Jx],
      [L/Jy,0,-L/Jy,0],
      [c/-Jz,c/Jz,c/-Jz,c/Jz]]

Bc = np.matrix(Bc)
F = np.matrix([U]).T
BcF = Bc@F

xdot = np.vstack((p_dot, V_Bdot, Theta_dot, Omega_dot,filler)) + np.vstack((BcF,filler))

# print("f: \n", xdot)

###########
# Uncommented Code is for checking values of dfdx (it's big and important)
###########


# dfdx = (derivative(xdot,X).subs(subber))
dfdx = np.matrix(derivative(xdot,X))
dfdu = np.matrix(derivative(xdot,U))



# dfdx = [val[0][0].evalf() for val in [row for row in dfdx]]
# check = []
# for i in range(12):
#     check.append(dfdx[12*i:12*(i+1)])

# print(np.matrix(check))


# print("\ndfdx:\n",dfdx)
# print("\ndfdu:\n",dfdu)

# xdot = [val.tolist()[0][0].subs(subber).evalf() for val in xdot[0:12]]
# print(xdot)


