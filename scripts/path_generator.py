#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Pose

class Path_Gen():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) # This function will be called before killing the node
        self.punto = rospy.get_param("/path")

        ######## PUBLISHERS AND SUBSCRIBERS #############
        self.pose_pub = rospy.Publisher('pose', Pose, queue_size= 1)
        rospy.Subscriber("flag", Int32, self.flag_cb)

        ###### CONSTANTS AND VARIABLES #########
        self.pose = Pose()
        self.x = 0.0 # x[m] Goal position
        self.y = 0.0 # y[m] Goal position

        self.position = 0
        self.recorrido = 0
        self.cb_status = 0
        self.i = 0

        while rospy.get_time() == 0:
            print("no simulated time has been received yet")
        print("Got time path generator")

        start_time = rospy.get_time() #Get the current time in float seconds

        state = "Program Pose"

        print("state: ", state)

        r = rospy.Rate(80) #10 Hz

        while not rospy.is_shutdown():
            #flag to know when to send the message
            if self.cb_status == 1 and self.recorrido<len(self.punto):
                self.pose.position.x = self.punto[self.i][0]
                self.pose.position.y = self.punto[self.i][1]
                self.i = self.i + 1
                self.recorrido = self.recorrido + 1
                self.cb_status = 0
                self.pose_pub.publish(self.pose)
            
    def flag_cb(self, msg):
        self.cb_status = msg.data
        if(self.recorrido>len(self.punto)-1):
            print("Proceso terminado ")
        else:
            print("Se recibio la bandera: " + str(self.cb_status))
        
    def cleanup(self):
        self.i = 0
        self.recorrido = 0
        self.pose = Pose()
        self.pose_pub.publish(self.pose)

    
if __name__ == "__main__":
    rospy.init_node("generador", anonymous=True)
    Path_Gen()