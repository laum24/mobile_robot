#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from numpy.linalg import inv

global wl, wr, rad, d, dt, x, y, theta
v = 0
w = 0
x = 0
y = 0
theta = 0

# Wheel radius and distance
rad = 0.05
d = 0.19

# Define the callback functions
def vel_callback(msg):
    global v, w
    v = msg.linear.x
    w = msg.angular.z

def wheels_local_velocity():
    global v, w
    R = np.array([[rad/2, rad/2], [rad/d, -rad/d]])
    R_inv = inv(R)
    velocities_vector = np.array([v, w])
    multiply_R_Vel = np.dot(velocities_vector, R_inv)
    return multiply_R_Vel[0], multiply_R_Vel[1]

def jacobiano():
    global x, y, theta
    pub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
    pub_vel_left = rospy.Publisher("/wl", Float32, queue_size=10)
    pub_vel_right = rospy.Publisher("/wr", Float32, queue_size=10)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        wl, wr = wheels_local_velocity()

        # Update pose
        x += (rad/2) * np.cos(theta) * wl + (rad/2) * np.cos(theta) * wr
        y += (rad/2) * np.sin(theta) * wl + (rad/2) * np.sin(theta) * wr
        theta += (rad/d) * wl - (rad/d) * wr

        # Publish pose
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = np.sin(theta/2)
        msg.pose.orientation.w = np.cos(theta/2)
        pub.publish(msg)

        # Publish wheel velocities
        msg1 = Float32()
        msg1.data = wl
        pub_vel_left.publish(msg1)
        msg2 = Float32()
        msg2.data = wr
        pub_vel_right.publish(msg2)

        rate.sleep()

if __name__ == '_main_':
    rospy.init_node("puzzlebot_sim")
    rospy.loginfo("Node puzzlebot_sim has been started")

    rospy.Subscriber("/cmd_vel", Twist, vel_callback)

    try:
        jacobiano()
    except rospy.ROSInterruptException:
        pass