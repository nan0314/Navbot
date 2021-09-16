#!/usr/bin/env python
import rospy
import std_msgs
import geometry_msgs
import nav_msgs
import tf_conversions
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import cvxpy as cvx
import scipy
import tf2_ros
from scipy.integrate import odeint
from dynamics import navbot
from MPC import MPC

def handle_pose(x):

    pose = PoseStamped()
    pose.header.frame_id="world"

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "navbot"
    t.transform.translation.x = x[0]
    t.transform.translation.y = x[1]
    t.transform.translation.z = x[2]
    q = tf_conversions.transformations.quaternion_from_euler(x[6], x[7], x[8])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    pose.pose.position.x = x[0]
    pose.pose.position.y = x[1]
    pose.pose.position.z = x[2]
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    return pose



def pynavbot_fly():
    path_pub = rospy.Publisher('navbot_path', Path, queue_size=10)
    rospy.init_node('pynavbot_fly', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    path_msg = Path()
    path_msg.header.frame_id = "world"

    # initialize state and control
    xu =  [30,30,5,0,0,0,0,0,0,0,0,0,0,0,0,0]
    x = np.matrix([xu[0:12]]).T
    u = np.matrix([xu[12:]]).T

    dt = 0.1
    goal = np.matrix([[25,30,5,0,0,0,0,0,0,0,0,0]]).T

    # set up linear model
    model = navbot()
    controller = MPC(dt=dt)

    # send initial pose
    pose = handle_pose(xu)
    path_msg.poses.append(pose)
    path_pub.publish(path_msg)
    path_pub.publish(path_msg)


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
        pose = handle_pose(xu)
        path_msg.poses.append(pose)
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        pynavbot_fly()
    except rospy.ROSInterruptException:
        pass