#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def odometry_callback(msg):
    global position_x, position_y, theta, v, w
    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    v = msg.twist.twist.linear.x
    w = msg.twist.twist.angular.z

def publish_transforms():
    rospy.init_node('transform_broadcaster', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Initialize static transform broadcaster
    static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Broadcast a fixed transform between "world" and "odom"
    odom_to_world_transform = geometry_msgs.msg.TransformStamped()
    odom_to_world_transform.header.stamp = rospy.Time.now()
    odom_to_world_transform.header.frame_id = "odom"
    odom_to_world_transform.child_frame_id = "world"
    odom_to_world_transform.transform.translation.x = 0.0
    odom_to_world_transform.transform.translation.y = 0.0
    odom_to_world_transform.transform.translation.z = 0.0
    odom_to_world_transform.transform.rotation.x = 0.0
    odom_to_world_transform.transform.rotation.y = 0.0
    odom_to_world_transform.transform.rotation.z = 0.0
    odom_to_world_transform.transform.rotation.w = 1.0
    static_tf_broadcaster.sendTransform(odom_to_world_transform)

    while not rospy.is_shutdown():
        # Update the position of the base link
        world_to_base_link = geometry_msgs.msg.TransformStamped()
        world_to_base_link.header.stamp = rospy.Time.now()
        world_to_base_link.header.frame_id = "world"
        world_to_base_link.child_frame_id = "base_link"
        world_to_base_link.transform.translation.x = position_x
        world_to_base_link.transform.translation.y = position_y
        world_to_base_link.transform.translation.z = 0.0
        
        # Calculate quaternion from euler angles
        quaternion = quaternion_from_euler(0, 0, theta)
        # Assign the quaternion values to the transform
        world_to_base_link.transform.rotation.x = quaternion[0]
        world_to_base_link.transform.rotation.y = quaternion[1]
        world_to_base_link.transform.rotation.z = quaternion[2]
        world_to_base_link.transform.rotation.w = quaternion[3]
        
        tf_broadcaster.sendTransform(world_to_base_link)

        # Update the position of the left wheel
        base_link_to_wl_link = geometry_msgs.msg.TransformStamped()
        base_link_to_wl_link.header.stamp = rospy.Time.now()
        base_link_to_wl_link.header.frame_id = "base_link"
        base_link_to_wl_link.child_frame_id = "wl_link"
        base_link_to_wl_link.transform.translation.x = 0.05  # Adjust according to your robot's dimensions
        base_link_to_wl_link.transform.translation.y = 0.0955
        base_link_to_wl_link.transform.translation.z = 0.05
        base_link_to_wl_link.transform.rotation.x = 0.0
        base_link_to_wl_link.transform.rotation.y = 0.0
        base_link_to_wl_link.transform.rotation.z = 0.0
        base_link_to_wl_link.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(base_link_to_wl_link)

        # Update the position of the right wheel
        base_link_to_wr_link = geometry_msgs.msg.TransformStamped()
        base_link_to_wr_link.header.stamp = rospy.Time.now()
        base_link_to_wr_link.header.frame_id = "base_link"
        base_link_to_wr_link.child_frame_id = "wr_link"
        base_link_to_wr_link.transform.translation.x = 0.05  # Adjust according to your robot's dimensions
        base_link_to_wr_link.transform.translation.y = -0.0955
        base_link_to_wr_link.transform.translation.z = 0.05
        base_link_to_wr_link.transform.rotation.x = 0.0
        base_link_to_wr_link.transform.rotation.y = 0.0
        base_link_to_wr_link.transform.rotation.z = 0.0
        base_link_to_wr_link.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(base_link_to_wr_link)

        rate.sleep()

if __name__ == '__main__':
    position_x = 0.0
    position_y = 0.0
    theta = 0.0
    v = 0.0
    w = 0.0
    
    rospy.Subscriber("/odom", Odometry, odometry_callback)

    try:
        publish_transforms()
    except rospy.ROSInterruptException:
        pass
