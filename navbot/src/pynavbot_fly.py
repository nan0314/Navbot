#!/usr/bin/env python
import rospy
import std_msgs
import tf
import geometry_msgs
import nav_msgs
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import numpy as np
import cvxpy as cvx
import scipy
from scipy.integrate import odeint
from dynamics import navbot

def riccati(Q,R,A,B):
    P_inf = np.zeros(Q.shape)

    while True:

        K = -(R + B.T*P_inf*B).I * B.T*P_inf*A
        P = Q + A.T*P_inf * (A + B*K)

        diff = abs(P_inf - P).sum()
        P_inf = P
        
        if diff < 1e-4:
            break
    
    return P

def pynavbot_fly():
    pose_pub = rospy.Publisher('pose', Odometry, queue_size=10)
    rospy.init_node('pynavbot_fly', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pose_msg = Odometry()

    pose_msg.header.frame_id = "world"

    # initialize state and control
    xu =  [30,30,5,0,0,0,0,0,0,0,0,0,0,0,0,0]
    x = np.matrix([xu[0:12]]).T
    u = np.matrix([xu[12:]]).T

    # set up linear model
    model = navbot()

    # control parameters
    N = 20 # control horizon
    dt = 0.1 # time step
    Q = 5*np.eye(12)
    R = 0.3**0.5 * np.eye(4)
    goal = np.matrix([[25,30,5,0,0,0,0,0,0,0,0,0]]).T
    umax = 25

    # send initial pose
    quaternion = tf.transformations.quaternion_from_euler(xu[6], xu[7], xu[8])
    pose_msg.pose.pose.position.x = xu[0]
    pose_msg.pose.pose.position.y = xu[1]
    pose_msg.pose.pose.position.z = xu[2]
    pose_msg.pose.pose.orientation.x = quaternion[0]
    pose_msg.pose.pose.orientation.y = quaternion[1]
    pose_msg.pose.pose.orientation.z = quaternion[2]
    pose_msg.pose.pose.orientation.w = quaternion[3]

    pose_pub.publish(pose_msg)


    while not rospy.is_shutdown():

        # Linearize model around operating point
        A = model.A(x,dt)
        B = model.B(dt)
        c = model.c(x,u,dt)
        P = riccati(Q,R,A,B)

        # Execute MCP
        X = {}
        U = {}
        cost = []
        constraints = []

        n = Q.shape[0]
        m = R.shape[0]

        for k in range(N-1):

            X[k] = cvx.Variable((n,1))
            U[k] = cvx.Variable((m,1))

            # cost.append( 5000*cvx.norm(((x0 - goal).T@(x0 - goal) / cvx.norm(x0 - goal)**2 - np.eye(12))@(X[k] - goal)))
            cost.append( cvx.quad_form(X[k] - goal,Q) ) # state cost
            # cost.append( cvx.quad_form(U[k],R) ) # control cost

            constraints.append(cvx.norm(U[k],"inf") <= umax)
            constraints.append(X[k][6] <= np.pi)
            constraints.append(X[k][6] >= -np.pi)
            constraints.append(X[k][7] <= np.pi/2 - 1e-8)
            constraints.append(X[k][7] >= -np.pi/2 + 1e-8)
            constraints.append(X[k][8] <= np.pi - 1e-8)
            constraints.append(X[k][8] >= -np.pi + 1e-8)

            if k == 0:
                constraints.append(X[k] == x)
            else:
                constraints.append(X[k] == x + A@(X[k-1]-x) + B@(U[k-1]-u) + c)
            
        X[N-1] = cvx.Variable((n,1))
        # cost.append( 5000*cvx.norm(((x0 - goal).T@(x0 - goal) / cvx.norm(x0 - goal)**2 - np.eye(12))@(X[k] - goal)))
        cost.append( cvx.quad_form(X[k] - goal,P) ) # state cost
        # cost.append( cvx.quad_form(U[k],R) ) # control cost

        constraints.append(cvx.norm(U[k],"inf") <= umax)
        constraints.append(X[N-1][6] <= np.pi)
        constraints.append(X[N-1][6] >= -np.pi)
        constraints.append(X[N-1][7] <= np.pi/2 - 1e-8)
        constraints.append(X[N-1][7] >= -np.pi/2 + 1e-8)
        constraints.append(X[N-1][8] <= np.pi - 1e-8)
        constraints.append(X[N-1][8] >= -np.pi + 1e-8)
        constraints.append(X[N-1] == x + A@(X[N-2]-x) + B@(U[N-2]-u) + c)

        objective = cvx.Minimize(cvx.sum(cost))
        prob = cvx.Problem(objective, constraints)
        prob.solve()
        
        u = U[0].value
        xu[12:] = [u[0],u[1],u[2],u[3]]

        t_vec = np.arange(0,dt,dt/10)
        xu = odeint(model.f, xu, t_vec)[-1]

        x = np.matrix([xu[0:12]]).T
        u = np.matrix([xu[12:]]).T

        quaternion = tf.transformations.quaternion_from_euler(xu[6], xu[7], xu[8])
        pose_msg.pose.pose.position.x = xu[0]
        pose_msg.pose.pose.position.y = xu[1]
        pose_msg.pose.pose.position.z = xu[2]
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]

        pose_pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        pynavbot_fly()
    except rospy.ROSInterruptException:
        pass