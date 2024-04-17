#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from numpy.linalg import inv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


#Declare Variables to be used

# Setup Variables to be used

# Declare the input Message

# Declare the  process output message


#Define the callback functions
def cmd_vel_callback(msg):
    global v,w #linear velocity and angular velocity
    v = msg.linear.x
    w = msg.angular.z

def wl_callback(msg):
     global wl #Left wheel angular velocity
     wl = msg.data

def wr_callback(msg):
     global wr    #Rigth wheel angular velocity
     wr = msg.data

  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

def kinematic_model():
     global r_wheel, wheelbase
     global phi_array #Angular velocities of each wheel
     matrix = np.array([r_wheel/2, r_wheel/2],
                             [r_wheel/wheelbase, -r_wheel/wheelbase]) 
     velocities_array = np.array([v,w]) #Lineal velocity, Angular velocity (psi/theta)
     r_wheel = 0.05
     wheelbase = 0.19
     inv_martrix = inv(matrix)
     phi_array = np.dot(inv_martrix,velocities_array) #Angular velocities of each wheel
     rospy.init_node()
     rate = rospy.Rate(10)
     wl_pub = rospy.Publisher("/wl",Float32,queue_size=10)
     wr_pub = rospy.Publisher("/wr",Float32,queue_size=10)
     while not rospy.is_shutdown():
          wl = phi_array[0][0]
          wr = phi_array[1][0]
          wl_pub.publish(wl)
          wr_pub.publish(wr)
          rate.sleep()


def pose():
     dt = 1/100.0
     psi = wrap_to_Pi(0.0) #Angle rotation in Z
     jacobiano = np.array([r_wheel*(np.cos(psi)/2),r_wheel*(np.cos(psi)/2),
                            r_wheel*(np.sin(psi)/2), r_wheel*(np.sin(psi)/2),
                            r_wheel/wheelbase, -r_wheel/wheelbase])
     pose_array = np.dot(jacobiano,phi_array) #Position array

     pose_pub = rospy.Publisher("/pose",PoseStamped,queue_size=10)
     rate = rospy.Rate(10)
     while not rospy.is_shutdown():
        pose_array += pose_array * dt
        #Create a PoseStamped msg
        pose_msg = PoseStamped()
        #Set the pose values
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = pose_array[0][0]
        pose_msg.pose.position.y = pose_array[1][0]
        pose_msg.pose.orientation.z = pose_array[2][0] 
        pose_pub.publish(pose_msg)

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))


    # Setup the Subscribers
    cmd_vel_pub = rospy.Subscriber("cmd_vel",Twist,callback=cmd_vel_callback)

    #Setup de publishers

    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)
        
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node