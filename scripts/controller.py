#!/usr/bin/env python
import rospy  
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Int32, Float32, Bool
from nav_msgs.msg import Odometry
import numpy as np
import time


class Controller():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)  # Function called before killing the node

        ############ ROBOT CONSTANTS ################  
        r = 0.05  # wheel radius [m] 
        L = 0.19  # wheel separation [m] 

        ###########  INITIALIZE VARIABLES ################ 
        self.thetar = 0.0  # [rads]
        self.xr = 0.0  # [m] posicion del robot a lo largo del eje x
        self.yr = 0.0  # [m] posicion del robot a lo largo del eje y
        self.v = 0.0  # [m/s] velocidad lineal del robot
        self.w = 0.0  # [rad/s] velocidad angular del robot

        # WHEELS SPEED 
        self.wl = 0.0  # [rad/s] velocidad angular de la rueda izq
        self.wr = 0.0  # [rad/s] velocidad angular de la rueda derecha

        # DISTANCES
        self.d = 100000.0
        self.d_min = 0.05  # [m] min distance to the goal

        # Flags to request the yaml positions
        self.flag = 1

        # Objective points
        self.xG = 0.0
        self.yG = 0.0

        # Linear velocity errors
        self.e_theta = 0.0

        # Control gains for angular velocity
        self.kp = 0.1

        # Control gains for linear velocity
        self.kp1 = 0.4

        ###########  PUBLISHERS AND SUBSCRIBERS ################
        time.sleep(4)
        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=10)
        self.flag_pub = rospy.Publisher('/flag', Int32, queue_size=10)
        rospy.Subscriber('/flag_lidar', Bool, self.lidar_cb)
        rospy.Subscriber('/puzzlebot_1/wl', Float32, self.wl_cb)
        rospy.Subscriber('/puzzlebot_1/wr', Float32, self.wr_cb)
        rospy.Subscriber('/pose', Pose, self.pose_cb)
        rospy.Subscriber('/odom', Odometry, self.odometry_cb)

        # Velocity message
        self.v_msg = Twist()

        self.state = 'GO_TO_GOAL'

        while rospy.get_time() == 0:
            print("No simulated time has been received yet")
        print("Got time controller")

        self.previous_time = rospy.get_time()
        self.rate = rospy.Rate(20)

        print("Node initialized")

        while not rospy.is_shutdown():
            if self.state == 'GO_TO_GOAL':
                self.go_to_goal()
            else:
                self.avoid_obstacles()
            
            self.rate.sleep()


    def avoid_obstacles(self):
        rospy.loginfo("Avoiding obstacle")
        #self.v = 0.05  # Move forward a bit to follow the wall
        #self.w = 0.3   # Turn to follow the wall
        self.v = 0.09  # Move forward a bit to follow the wall
        self.w = -0.5   # Turn to follow the wall
        self.v_msg.linear.x = self.v
        self.v_msg.angular.z = self.w
        self.pub_cmd_vel.publish(self.v_msg)

    def go_to_goal(self):
        rospy.loginfo("going to goal")
        if self.d >= self.d_min and self.flag == 0:
            delta_t = rospy.get_time() - self.previous_time
            self.previous_time = rospy.get_time()  # Update time after use it
            self.v = 0.05 * (self.wl + self.wr) / 2.0
            self.w = 0.05 * (self.wr - self.wl) / 0.19


            # Distance from the goal
            self.d = np.sqrt((self.xG - self.xr)**2 + (self.yG - self.yr)**2)
            print("xG:" + str(self.xG))
            print("yG:" + str(self.yG))
            print("xr:" + str(self.xr))
            print("yr:" + str(self.yr))

            # Angle to reach the goal
            theta_g = np.arctan2(self.yG - self.yr, self.xG - self.xr)
            print("theta goal", theta_g)

            e_theta = theta_g - self.thetar

            if abs(e_theta) < 0.03:
                self.v = self.kp * self.d
                self.w = 0
            else:
                self.w = self.kp1 * e_theta
                self.v = 0
                print("w ", self.w)
                print(e_theta)
                print("xr:", str(self.xr))
                print("yr:", str(self.yr))

            if abs(self.d) < 0.03 and abs(e_theta) <= 0.03:
                self.v = 0
                self.w = 0

            self.v_msg.linear.x = self.v
            self.v_msg.angular.z = self.w

        else:
            print("Stop")
            self.flag = 1
            self.flag_pub.publish(self.flag)
            self.v_msg.linear.x = 0.0
            self.v_msg.angular.z = 0.0
            self.d = 100000.0

        self.pub_cmd_vel.publish(self.v_msg)

    def pose_cb(self, msg):
        self.xG = msg.position.x
        self.yG = msg.position.y
        self.flag = 0
        self.flag_pub.publish(self.flag)
        print("Callback pose done in controller node")

    def wl_cb(self, wl):
        # This function receives the left wheel speed from the encoders
        self.wl = wl.data

    def wr_cb(self, wr):
        # This function receives the left wheel speed from the encoders
        self.wr = wr.data

    def odometry_cb(self, msg):
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        self.thetar = msg.pose.pose.orientation.z

    def lidar_cb(self, msg):
        self.flag_lidar = msg.data

        if self.flag_lidar == True:
            self.state = 'AVOID_OBSTACLE'
        else:
            self.state = 'GO_TO_GOAL'

    def cleanup(self):
        # This function is called just before finishing the node
        v_msg = Twist()
        self.pub_cmd_vel.publish(v_msg)


if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    Controller()