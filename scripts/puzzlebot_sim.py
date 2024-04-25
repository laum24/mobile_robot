#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from numpy.linalg import inv
from geometry_msgs.msg import Twist, PoseStamped

class PuzzlebotCinematicModel:
    def __init__(self):
        rospy.init_node("puzzlebot_sim")
        rospy.loginfo("puzzlebot sim has started")

        # Initialize variables
        self.x = 0
        self.y = 0
        self.psi = 0
        self.v = 0
        self.w = 0
        self.r_wheel = 0.05
        self.wheelbase = 0.19

        # Setup the publishers
        self.wl_pub = rospy.Publisher("/wl",Float32,queue_size=10)
        self.wr_pub = rospy.Publisher("/wr",Float32,queue_size=10)
        self.pose_pub = rospy.Publisher("/pose",PoseStamped,queue_size=10)

        # Setup the subscribers
        rospy.Subscriber("/cmd_vel", Twist, self.cmdvel_callback)

        # Rate 
        self.loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    # Define the callback functions
    def cmdvel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def wrap_to_Pi(theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
    
    def kinematic_model(self):
        while not rospy.is_shutdown():
            matrix = np.array([[self.r_wheel/2, self.r_wheel/2],
                    [self.r_wheel/self.wheelbase, -self.r_wheel/self.wheelbase]])
            inv_martrix = inv(matrix)
            velocities_array = np.array([self.v,self.w]) #Linear velocity, Angular velocity (psi/theta)
            phi_array = np.dot(inv_martrix,velocities_array) #Angular velocities of each wheel
            self.wl = phi_array[0]
            self.wr = phi_array[1]

            dt = 0.01 # Integration step
            jacobiano = np.array([[self.r_wheel*np.cos(self.psi)/2,self.r_wheel*np.cos(self.psi)/2],
                        [self.r_wheel*np.sin(self.psi)/2, self.r_wheel*np.sin(self.psi)/2],
                        [self.r_wheel/self.wheelbase, -self.r_wheel/self.wheelbase]])
            pose_array = np.dot(jacobiano,phi_array)

            # Update pose
            self.x += pose_array[0] * dt
            self.y += pose_array[1] * dt
            self.psi += pose_array[2] * dt

            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = self.x
            pose_msg.pose.position.y = self.y
            pose_msg.pose.orientation.z = self.psi
            self.pose_pub.publish(pose_msg)

            # Publish wheel velocities
            wl_msg = Float32()
            wl_msg.data = self.wl
            self.wl_pub.publish(wl_msg)

            wr_msg = Float32()
            wr_msg.data = self.wr
            self.wr_pub.publish(wr_msg)

            self.loop_rate.sleep()
            
if __name__ == '__main__':
    controller = PuzzlebotCinematicModel()
    controller.kinematic_model()