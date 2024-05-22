#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time

class Lidar():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.flag_lidar = Bool()
        self.ranges = []
        self.intensities = []
        self.distance_to_obstacle_threshold = 0.7
        
        while rospy.get_time() == 0:
            print("No simulated time has been received yet")
        print("Got time lidar")

        
        rospy.Subscriber('/puzzlebot_1/scan', LaserScan, self.laser_cb)
        self.flag_lidar_pub = rospy.Publisher('/flag_lidar', Bool, queue_size=10)
        self.rate = rospy.Rate(20)
        print("Node lidar initialized")

        while not rospy.is_shutdown():
            if self.ranges:
                time.sleep(0.1)
                # Filter ranges to get only the front sector
                front_ranges = self.get_front_ranges(self.ranges)
                if front_ranges:  # Check if there are any readings
                    self.range_min = min(front_ranges)
                    print(self.range_min)
                    if self.range_min < self.distance_to_obstacle_threshold:
                        self.flag_lidar.data = True
                    else:
                        self.flag_lidar.data = False
                    self.flag_lidar_pub.publish(self.flag_lidar)
            self.rate.sleep()


    def laser_cb(self, msg):
        self.ranges = msg.ranges  # Corrected variable name
        self.intensities = msg.intensities

    def get_front_ranges(self, ranges):
        # Assuming the LIDAR has a 360-degree field of view and we're interested in the front 60 degrees
        # Adjust start_index and end_index based on the LIDAR's resolution and desired front sector
        start_angle = -30  # degrees
        end_angle = 30  # degrees

        num_readings = len(ranges)
        angle_increment = 360.0 / num_readings

        start_index = int((start_angle + 180) / angle_increment)
        end_index = int((end_angle + 180) / angle_increment)

        # Handle wrap-around for indices
        if start_index < 0:
            start_index += num_readings
        if end_index < 0:
            end_index += num_readings

        if start_index <= end_index:
            return ranges[start_index:end_index+1]
        else:
            # If the range crosses the 0 index
            return ranges[start_index:] + ranges[:end_index+1]

    def cleanup(self):
        # This function is called just before finishing the node
        pass

if __name__ == "__main__":
    rospy.init_node("lidar", anonymous=True)
    Lidar()