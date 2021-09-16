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
from MPC import MPC
    

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

    dt = 0.1
    goal = np.matrix([[25,30,5,0,0,0,0,0,0,0,0,0]]).T

    # set up linear model
    model = navbot()
    controller = MPC()

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
        
        # get control for current operating point
        u = controller.control(x,u,A,B,c,goal)

        # simulate nonlinear dynamics (take time step)
        xu[12:] = [u[0],u[1],u[2],u[3]]
        t_vec = np.arange(0,dt,dt/10)
        xu = odeint(model.f, xu, t_vec)[-1]

        x = np.matrix([xu[0:12]]).T
        u = np.matrix([xu[12:]]).T

        # publish updated drone locaation
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