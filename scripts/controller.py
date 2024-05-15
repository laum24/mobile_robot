#!/usr/bin/env python
import rospy  
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Int32, Float32 
from nav_msgs.msg import Odometry
import numpy as np
import time

# Code based on Minichallenge 2

class Controller():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) #Function called before killing the node
        ############ ROBOT CONSTANTS ################  
        r=0.05 #wheel radius [m] 
        L=0.19 #wheel separation [m] 

        ###########  INITIALIZE VARIABLES ################ 
        self.thetar = 0.0 #[rads]
        self.xr = 0.0 #[m] posicion del robot a lo largo del eje x
        self.yr = 0.0 #[m] posicion del robot a lo largo del eje y
        v = 0.0 #[m/s] velocidad lineal del robot
        w = 0.0 #[rad/s] velocidad angular del robot

        # WHEELS SPEED 
        self.wl = 0.0 #[rad/s] velocidad angular de la rueda izq
        self.wr = 0.0 #[rad/s] velocidad angular de la rueda derecha

        # DISTANCES
        d = 100000.0
        d_min = 0.05 #[m] min distance to the goal
        e_theta_min = np.pi/90

        # Vels max and Min
        w_max = 0.5
        w_min = 0.05
        v_max = 0.4
        v_min = 0.05

        #Flags to request the yaml positions
        self.flag = 1

        #Objective points
        self.xG = 0.0
        self.yG = 0.0

        #Linear velocity errors
        e0 = 0.0
        e1 = 0.0
        e2 = 0.0

        #Angular velocity errors
        e3 = 0.0
        e4 = 0.0
        e5 = 0.0

        #Control input for angular velocity
        u0 = 0.0
        u1 = 0.0

        #Control input for linear velocity
        u2 = 0.0
        u3 = 0.0

        #Control gains for angular velocity
        kp = 0.3
        ki = 0.0
        kd = 0.0

        #Control gains for linear velocity
        kp1 = 0.3
        ki1 = 0.0
        kd1 = 0.0

        #Time for controlling angular velocity
        Ts = rospy.get_param("/time", 0.1)
        #Time for controlling linear velocity
        Ts1 = rospy.get_param("/time", 0.1)

        #Angular velocity controller
        K1 = kp + Ts * ki + kd/Ts
        K2 = -kp - 2.0 * kd / Ts
        K3 = kd/Ts

        #Linear velocity controller
        K4 = kp1 + Ts1 * ki1 + kd1/Ts1
        K5 = -kp1 - 2.0 * kd1 / Ts1
        K6 = kd1/Ts1

        ###########  PUBLISHERS AND SUBSCRIBERS ################
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.flag_pub = rospy.Publisher('/flag', Int32, queue_size=10)
        rospy.Subscriber('/wl', Float32, self.wl_cb)
        rospy.Subscriber('/wr', Float32, self.wr_cb)
        rospy.Subscriber('/pose', Pose, self.pose_cb)
        rospy.Subscriber('/odom', Odometry, self.odometry_cb)

        #Velocity message
        v_msg = Twist()

        while rospy.get_time()== 0:
            print("No simulated time has been received yet")
        print("Got time controller")

        previous_time = rospy.get_time()
        rate = rospy.Rate(20)

        print("Node initialized")

        while not rospy.is_shutdown():
            if d >= d_min and self.flag==0:
                delta_t = rospy.get_time() - previous_time
                previous_time = rospy.get_time() #Update time after use it
                self.v = r*(self.wl+self.wr)/2.0
                self.w = r*(self.wr-self.wl)/L
                # Distance from the goal
                d = np.sqrt((self.xG - self.xr)**2 + (self.yG - self.yr)**2)
                print("xG:" + str(self.xG))
                print("yG:" + str(self.yG))
                e3 = d

                #Angle to reach the goal
                theta_g = np.arctan2(self.yG - self.yr, self.xG - self.xr)

                #Crop e_Theta from -pi to pi
                theta_g = np.arctan2(np.sin(theta_g), np.cos(theta_g))

                #Angle's error
                e_theta = theta_g - self.thetar
                e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

                #Controlling time and distance errors, taking theta goal as reference
                e0 = e_theta

                u0 = K1*e0 + K2*e1 + K3*e2 + u1
                u2 = K4*e3 + K5*e4 + K6*e5 + u3

                if(u0>1):
                    u0 = 1
                elif(u0 < -1):
                    u0 = -1

                if(u2>1):
                    u2 = 1
                elif(u2 < -1):
                    u2 = -1

                #Error for controlling angular velocity
                e2 = e1
                e1 = e0
                #Input for angular velocity
                u1 = u0

                #Error for controlling linear velocity
                e5 = e4
                e4 = e3
                #Input for linear velocity
                u3 = u2

                m_input = u0
                m_input1 = u2

                print("m_input angular: " + str(m_input))
                print("m_input linear: " + str(m_input1))

                if(abs(e_theta) > e_theta_min):
                    w = abs(m_input) * e_theta
                    v = 0
                else:
                    v = abs(m_input1) * d
                    w = abs(m_input) * e_theta

                if(w > w_max):
                    w = w_max
                elif(w < -w_max):
                    w = -w_max
                elif(w < w_min and w>0):
                    w = w_min
                elif(w > -w_min and w<0):
                    w = -w_min

                if(v > v_max):
                    v = v_max
                elif(v < -v_max):
                    v = -v_max
                elif(v <  v_min and v > 0):
                    v = v_min
                elif(v > -v_min and v < 0): 
                    v = -v_min
                
                v_msg.linear.x = v
                v_msg.angular.z = w

            else:
                print("Stop")
                self.flag = 1
                self.flag_pub.publish(self.flag)
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                d = 100000.0

            self.pub_cmd_vel.publish(v_msg)
            rate.sleep()

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
        global position_x, position_y, theta
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        self.thetar = msg.pose.pose.orientation.z



    
    def cleanup(self):
        # This function is called just before finishing the node
        v_msg = Twist()
        self.pub_cmd_vel.publish(v_msg)

if __name__== "__main__":
    rospy.init_node("controller", anonymous=True)
    Controller()