#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

class Localisation:
    def __init__(self):
        rospy.init_node("localisation")
        rospy.loginfo("Node localisation has been started")
        # Initialize variables
        self.wl = 0.0
        self.wr = 0.0
        self.theta = 0.0
        self.x = 0.0
        self.y=0.0
        #ROBOT CONSTANTS
        self.r = 0.05  # wheel radius [m]
        self.L = 0.19  # wheel separation [m]


        # Setup Subscriber
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        rospy.Subscriber("/wr", Float32, self.wr_callback)

        # Setup Publisher
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        # Rate control
        self.loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def Odometry(self):
        while rospy.get_time() == 0: 
            print("no simulated time has been received yet") 
        print("Got time") 

        previous_time = rospy.get_time() 
        rate = rospy.Rate(20) #20Hz 
        
        while not rospy.is_shutdown():
            dt = rospy.get_time() - previous_time
            # Getting linear and angular velocities
            self.v = self.r*(self.wl+self.wr)/2.0
            self.w = self.r*(self.wr-self.wl)/self.L

            # Updating the robot's pose
            self.x += self.v * np.cos(self.theta) * dt
            self.y += self.v * np.sin(self.theta) *dt
            self.theta += self.w * dt

            # Creating odometry message
            odometry_msg = Odometry()
            odometry_msg.header.stamp = rospy.Time.now()
            odometry_msg.pose.pose.position.x = self.x
            odometry_msg.pose.pose.position.y = self.y
            odometry_msg.pose.pose.orientation.z = self.theta
            self.odom_pub.publish(odometry_msg)
            previous_time = rospy.get_time()
            rate.sleep()


if __name__ == '__main__':
    localisation = Localisation()
    localisation.Odometry()