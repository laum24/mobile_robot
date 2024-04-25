#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

def odometry_callback(msg):
    global position_x, position_y, theta
    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    # Extracting yaw angle from the orientation quaternion
    theta = msg.pose.pose.orientation.z
    linear_velocity = msg.twist.twist.linear.x
    angular_velocity = msg.twist.twist.angular.z 

def publish_transforms():
    rospy.init_node('transform_broadcaster', anonymous=True)
    rate = rospy.Rate(50)  # 50 Hz
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        # Update the position of the base link
        odom_to_baselink = geometry_msgs.msg.TransformStamped()
        odom_to_baselink.header.stamp = rospy.Time.now()
        odom_to_baselink.header.frame_id = "odom"
        odom_to_baselink.child_frame_id = "base_link"
        odom_to_baselink.transform.translation.x = position_x
        odom_to_baselink.transform.translation.y = position_y
        odom_to_baselink.transform.translation.z = 0.0
        
        # Calculate quaternion from euler angles
        quaternion = quaternion_from_euler(0, 0, theta)
        # Assign the orientation_q values to the transform
        odom_to_baselink.transform.rotation.x = quaternion[0]
        odom_to_baselink.transform.rotation.y = quaternion[1]
        odom_to_baselink.transform.rotation.z = quaternion[2]
        odom_to_baselink.transform.rotation.w = quaternion[3]
        
        tf_broadcaster.sendTransform(odom_to_baselink)

        # Update the position of the wheel links (left and right)
        wl_link_to_base = geometry_msgs.msg.TransformStamped()
        wl_link_to_base.header.stamp = rospy.Time.now()
        wl_link_to_base.header.frame_id = "base_link"
        wl_link_to_base.child_frame_id = "wl_link"
        wl_link_to_base.transform.translation.x = 0.0  # Adjust as needed
        wl_link_to_base.transform.translation.y = 0.0
        wl_link_to_base.transform.translation.z = 0.0
        wl_link_to_base.transform.rotation.w = 1.0  # No rotation
        
        wr_link_to_base = geometry_msgs.msg.TransformStamped()
        wr_link_to_base.header.stamp = rospy.Time.now()
        wr_link_to_base.header.frame_id = "base_link"
        wr_link_to_base.child_frame_id = "wr_link"
        wr_link_to_base.transform.translation.x = 0.0  # Adjust as needed
        wr_link_to_base.transform.translation.y = 0.0
        wr_link_to_base.transform.translation.z = 0.0
        wr_link_to_base.transform.rotation.w = 1.0  # No rotation
        tf_broadcaster.sendTransform([wl_link_to_base, wr_link_to_base])

        rate.sleep()

if __name__ == '__main__':
    # Rate control
    #loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))
    position_x = 0.0
    position_y = 0.0
    theta = 0.0

    rospy.Subscriber("/odom", Odometry, odometry_callback)

    try:
        publish_transforms()
    except rospy.ROSInterruptException:
        pass
