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

        # Constants for error model
        self.kr = 0.00001  # Error coefficient for the right wheel
        self.kl = 0.00001  # Error coefficient for the left wheel

        # Setup Subscriber
        rospy.Subscriber("/puzzlebot_1/wr", Float32, self.wl_callback)
        rospy.Subscriber("/puzzlebot_1/wl", Float32, self.wr_callback)

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

            #Define Jacobian Matrix H_k
            H_k = np.array([
                [1, 0, -dt * self.v * np.sin(self.theta)],
                [0, 1, dt * self.v * np.cos(self.theta)],
                [0, 0, 1]
            ])

            #Define the error matrix Q_k
            Q_k = np.diag([self.kr * abs(self.wr) * dt,
                           self.kl * abs(self.wl) * dt,
                           (self.kr * abs(self.wr) + self.kl * abs(self.wl)) * dt])
            
            # Update covariance matrix using the previous covariance matrix
            if not hasattr(self, 'sigma'):
                self.sigma = np.eye(3)  # Initializes the covariance matrix if it hasn't been defined

            # Predict the new covariance matrix
            self.sigma = H_k.dot(self.sigma).dot(H_k.T) + Q_k

            # Extend 3x3 matrix to 6x6 for ROS compatibility
            sigma_full = np.zeros((6, 6))
            sigma_full[:3, :3] = self.sigma  # Fill in the 3x3 position covariance
            sigma_full[3, 3] = 0.001  # Small value for orientation around x (roll)
            sigma_full[4, 4] = 0.001  # Small value for orientation around y (pitch)
            sigma_full[5, 5] = 0.001  # Small value for orientation around z (yaw)

            # Creating odometry message
            odometry_msg = Odometry()
            odometry_msg.header.stamp = rospy.Time.now()
            odometry_msg.header.frame_id = "odom"
            odometry_msg.child_frame_id = "base_link"
            odometry_msg.pose.pose.position.x = self.x
            odometry_msg.pose.pose.position.y = self.y
            odometry_msg.pose.pose.orientation.z = self.theta
            odometry_msg.twist.twist.linear.x = self.v
            odometry_msg.twist.twist.angular.z = self.w
            self.odom_pub.publish(odometry_msg)
            odometry_msg.pose.covariance = sigma_full.flatten().tolist()  # Set the pose covariance matrix as a list
            previous_time = rospy.get_time()
            rate.sleep()

if __name__ == '__main__':
    localisation = Localisation()
    localisation.Odometry()