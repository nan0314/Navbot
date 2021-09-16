#!/usr/bin/env python3
import numpy as np

class navbot:

    g = -10

    def __init__(self, Jx=1, Jy=1, Jz=1,L=1,c=1,m=1):
        
        J = [[Jx, 0, 0],
             [0, Jy, 0],
             [0, 0, Jz]]
        
        e3 = [[0],[0],[1]]

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

        self.J = np.matrix(J)
        self.Jx = Jx
        self.Jy = Jy
        self.Jz = Jz
        self.e3 = np.matrix(e3)
        self.Bc = np.matrix(Bc)

    # Body inertial frame
    def R_BI(self,phi, theta, psi):

        R = [ [np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
              [np.sin(theta)*np.cos(psi)*np.sin(phi) - np.sin(psi)*np.cos(phi), np.sin(theta)*np.sin(psi)*np.sin(phi) + np.cos(psi)*np.cos(phi), np.cos(theta)*np.sin(phi)],
              [np.sin(theta)*np.cos(psi)*np.cos(phi) + np.sin(psi)*np.sin(phi), np.sin(theta)*np.sin(psi)*np.cos(phi) - np.cos(psi)*np.sin(phi), np.cos(theta)*np.cos(phi)]]

        return np.matrix(R)
    
    def W(self,phi,theta,psi):

        W = [ [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
              [0, np.cos(phi), -np.sin(phi)],
              [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]]

        return np.matrix(W)

    # linear state matrix
    def dfdx(self,X):

        x = X[0]
        y = X[1]
        z = X[2]
        Vx = X[3]
        Vy = X[4]
        Vz = X[5]
        phi = X[6]
        theta = X[7]
        psi = X[8]
        p = X[9]
        q = X[10]
        r = X[11]
        sin = np.sin
        cos = np.cos
        tan = np.tan

        dfdx = [ [0, 0, 0, np.cos(psi)*np.cos(theta), sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi), sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi), Vy*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi)) + Vz*(-sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi)), -Vx*sin(theta)*cos(psi) + Vy*sin(phi)*cos(psi)*cos(theta) + Vz*cos(phi)*cos(psi)*cos(theta), -Vx*sin(psi)*cos(theta) + Vy*(-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi)) + Vz*(sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi)), 0, 0, 0],
                 [0, 0, 0, sin(psi)*cos(theta), sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi), -sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi), Vy*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi)) + Vz*(-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi)), -Vx*sin(psi)*sin(theta) + Vy*sin(phi)*sin(psi)*cos(theta) + Vz*sin(psi)*cos(phi)*cos(theta), Vx*cos(psi)*cos(theta) + Vy*(sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi)) + Vz*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi)), 0, 0, 0],
                 [0, 0, 0, -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta), Vy*cos(phi)*cos(theta) - Vz*sin(phi)*cos(theta), -Vx*cos(theta) - Vy*sin(phi)*sin(theta) - Vz*sin(theta)*cos(phi), 0, 0, 0, 0],
                 [0, 0, 0, 0, r, -q, 0, -self.g*cos(theta), 0, 0, -Vz, Vy],
                 [0, 0, 0, -r, 0, p, self.g*cos(phi)*cos(theta), -self.g*sin(phi)*sin(theta), 0, Vz, 0, -Vx],
                 [0, 0, 0, q, -p, 0, -self.g*sin(phi)*cos(theta), -self.g*sin(theta)*cos(phi), 0, -Vy, Vx, 0],
                 [0, 0, 0, 0, 0, 0, q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta), q*(tan(theta)**2 + 1)*sin(phi) + r*(tan(theta)**2 + 1)*cos(phi), 0, 1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
                 [0, 0, 0, 0, 0, 0, -q*sin(phi) - r*cos(phi), 0, 0, 0, cos(phi), -sin(phi)],
                 [0, 0, 0, 0, 0, 0, q*cos(phi)/cos(theta) - r*sin(phi)/cos(theta), q*sin(phi)*sin(theta)/cos(theta)**2 + r*sin(theta)*cos(phi)/cos(theta)**2, 0, 0, sin(phi)/cos(theta), cos(phi)/cos(theta)],
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (self.Jx*r - self.Jz*r)/self.Jx, (self.Jy*q - self.Jz*q)/self.Jx],
                 [0, 0, 0, 0, 0, 0, 0, 0, 0,(-self.Jx*r + self.Jz*r)/self.Jy, 0, (-self.Jx*p + self.Jz*p)/self.Jy],
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, (self.Jx*q - self.Jy*q)/self.Jz, (self.Jx*p - self.Jy*p)/self.Jz, 0]]

        dfdx = np.matrix(dfdx,dtype=float)
        return dfdx

    def dfdu(self):
        return self.Bc

    def f(self,XU,t):

        V_B = np.matrix([[XU[3]], [XU[4]], [XU[5]]])

        Omega = np.matrix([[XU[9]], [XU[10]], [XU[11]]])

        R_BI = self.R_BI(XU[6],XU[7],XU[8])

        W = self.W(XU[6],XU[7],XU[8])

        p_dot = R_BI.T@V_B
        V_Bdot = np.cross(-Omega,V_B,axis=0) + self.g*R_BI@self.e3
        Theta_dot = W@Omega
        Omega_dot = np.linalg.inv(self.J)@(np.cross(-Omega,self.J@Omega,axis=0))
        filler = np.zeros((4,1))

        F = np.matrix([[XU[12]],[XU[13]],[XU[14]],[XU[15]]])
        BcF = self.Bc@F

        xdot = np.vstack((p_dot, V_Bdot, Theta_dot, Omega_dot,filler)) + np.vstack((BcF,filler))

        return xdot.T.tolist()[0]

    def A(self,X,dt):

        A = np.eye(12) + dt*self.dfdx(X)
        return A

    def B(self,dt):

        B = dt*self.dfdu()
        return B
    
    def c(self,X,U,dt):
        xu = np.vstack((X,U)).T.tolist()[0]
        xdot = np.matrix(self.f(xu,0)[0:12]).T
        return dt*xdot


