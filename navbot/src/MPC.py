import numpy as np
import cvxpy as cvx

class MPC:

    def __init__(self, N=20, dt=0.1, Q=5*np.eye(12),R=0.3*np.eye(4),umax=25):

        self.N = N
        self.dt = dt
        self.Q = Q
        self.R = R
        self.umax = umax
    
    def ricatti(self,A,B):
        P_inf = self.Q

        while True:

            K = -(self.R + B.T*P_inf*B).I * B.T*P_inf*A
            P = self.Q + A.T*P_inf * (A + B*K)

            diff = abs(P_inf - P).sum()
            P_inf = P
            
            if diff < 1e-4:
                break
        
        return P

    def feasibility_constraints(self,X_k,U_k):
        constraints = []
        constraints.append(cvx.norm(U_k,"inf") <= self.umax)
        constraints.append(X_k[6] <= np.pi)
        constraints.append(X_k[6] >= -np.pi)
        constraints.append(X_k[7] <= np.pi/2 - 1e-8)
        constraints.append(X_k[7] >= -np.pi/2 + 1e-8)
        constraints.append(X_k[8] <= np.pi - 1e-8)
        constraints.append(X_k[8] >= -np.pi + 1e-8)

        return constraints

    
    def control(self,x,u,A,B,c,goal):

        # Dictionary to store X_k, U_k (predicted state/control)
        X = {}
        U = {}

        # declare cost (objective) and constraint lists
        cost = []
        constraints = []

        n = self.Q.shape[0]
        m = self.R.shape[0]

        P = self.ricatti(A,B)

        # Compute Horizon
        for k in range(self.N-1):

            X[k] = cvx.Variable((n,1))
            U[k] = cvx.Variable((m,1))

            # Add cost terms
            cost.append( cvx.quad_form(X[k] - goal,self.Q) ) # state cost
            # cost.append( cvx.quad_form(U[k],R) ) # control cost

            # add constraints
            constraints += self.feasibility_constraints(X[k],U[k])

            if k == 0:
                constraints.append(X[k] == x)
            else:
                constraints.append(X[k] == x + A@(X[k-1]-x) + B@(U[k-1]-u) + c)
            
        # final state
        X[self.N-1] = cvx.Variable((n,1))
        cost.append( cvx.quad_form(X[k] - goal,P) ) # state cost
        # cost.append( cvx.quad_form(U[k],R) ) # control cost

        constraints += self.feasibility_constraints(X[self.N-1],U[self.N-2])
        constraints.append(X[self.N-1] == x + A@(X[self.N-2]-x) + B@(U[self.N-2]-u) + c)

        objective = cvx.Minimize(cvx.sum(cost))
        prob = cvx.Problem(objective, constraints)
        prob.solve()

        return U[0].value