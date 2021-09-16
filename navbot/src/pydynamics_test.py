#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.function_base import diff
import scipy
from scipy.integrate import odeint
from dynamics import navbot

navbot = navbot()

# print(navbot.f([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16],0))

dt = 0.01
xu0 = [25,30,5,0,0,0,0,0,0,0,0,0,0,0,0,0]
t_vec = list(np.arange(0,0.25,dt))
# print(navbot.f(xu0,0).T.tolist()[0])

ans = odeint(navbot.f,xu0,t_vec)
z_vec1 = [val[2] for val in ans]
ans[-1][12:] = [-10/4,-10/4,-10/4,-10/4]
t_vec = list(np.arange(0.25,0.5,dt))
ans2 = odeint(navbot.f,ans[-1],t_vec)
z_vec2 = [val[2] for val in ans2]
z_vec = z_vec1 + z_vec2[1:]


t_vec = list(np.arange(0,0.5-dt,dt))
X = np.matrix([xu0[0:12]]).T
U = np.matrix([xu0[12:]]).T
X0 = X
U0=U
A = navbot.A(X,dt)
B = navbot.B(dt)
c = navbot.c(X,U,dt)
# print(navbot.dfdx(X))
# 5/0

discrete = [X.T.tolist()[0]]
for k in range(len(z_vec)-1):
    if abs(t_vec[k+1] - 0.25) < 0.00005:
        U[0] = -10/4
        U[1] = -10/4
        U[2] = -10/4
        U[3] = -10/4
        A = navbot.A(X,dt)
        B = navbot.B(dt)
        c = navbot.c(X,U,dt)
        X0 = X
        U0 = U
    
    xnew = X0 + A@(X - X0) + B@(U - U0) + c
    X = np.matrix(xnew)
    discrete.append(X.T.tolist()[0])


z_vecd = [val[2] for val in discrete]
plt.plot(t_vec,z_vec)
plt.plot(t_vec,z_vecd)
plt.show()


    