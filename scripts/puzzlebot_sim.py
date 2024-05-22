#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from numpy.linalg import inv

class PuzzlebotController:
    def __init__(self):
        rospy.init_node("puzzlebot_sim")
        rospy.loginfo("Node puzzlebot_sim has been started")

        # Initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.w = 0
        self.rad = 0.05 
        self.d = 0.19

        # Setup Publishers
        #self.pub_pose = rospy.Publisher("/pose", PoseStamped, queue_size=10)
        self.pub_vel_left = rospy.Publisher("/puzzlebot_1/wl", Float32, queue_size=10)
        self.pub_vel_right = rospy.Publisher("/puzzlebot_1/wr", Float32, queue_size=10)

        # Setup Subscriber
        rospy.Subscriber("/puzzlebot_1/base_controller/cmd_vel", Twist, self.vel_callback)

        # Rate control
        self.loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    def vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def jacobiano(self):
        while not rospy.is_shutdown():
            R = np.array([[self.rad/2, self.rad/2],[self.rad/self.d, -self.rad/self.d]])
            R_inversa = inv(R)
            velocities_vector = np.array([self.v, self.w])
            multiply_R_Vel = np.dot(R_inversa, velocities_vector)
            self.wl = multiply_R_Vel[0]
            self.wr = multiply_R_Vel[1]

            dt = 0.01  # frequency
            matriz_jacobiano = np.array([[self.rad*np.cos(self.theta)/2, self.rad*np.cos(self.theta)/2],
                                          [self.rad*np.sin(self.theta)/2, self.rad*np.sin(self.theta)/2],
                                          [self.rad/2, -self.rad/self.d]])

            vel_inercial = np.dot(matriz_jacobiano, multiply_R_Vel)
            
            # Update position
            self.x += vel_inercial[0] *dt
            self.y += vel_inercial[1] *dt
            self.theta += vel_inercial[2] *dt

            # Publish Pose
            '''
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = self.x
            pose_msg.pose.position.y = self.y
            pose_msg.pose.orientation.z = self.theta
            self.pub_pose.publish(pose_msg)
            '''

            # Publish Velocities
            vel_left_msg = Float32()
            vel_left_msg.data = self.wl
            self.pub_vel_left.publish(vel_left_msg)

            vel_right_msg = Float32()
            vel_right_msg.data = self.wr
            self.pub_vel_right.publish(vel_right_msg)

            self.loop_rate.sleep()

if __name__ == '__main__':
    controller = PuzzlebotController()
    controller.jacobiano()