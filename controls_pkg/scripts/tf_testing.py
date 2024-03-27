#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Bool, Header, Int64MultiArray

import time
import math
import numpy as np

def updateTf(transform_brodcaster, tf_buffer, delta_s = 0.1, delta_theta = np.pi / 8):
    
    try:
        transform = tf_buffer.lookup_transform("base_link", "world", rospy.Time())

        transform.header.stamp = rospy.Time.now()
        
        quaternion = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]
        
        (roll, pitch, yaw) = tf_trans.euler_from_quaternion(quaternion)

        # transform.transform.translation.x += delta_s * np.cos(yaw + delta_theta / 2.0)
        # transform.transform.translation.y += delta_s * np.sin(yaw + delta_theta / 2.0)
        # transform.transform.translation.z += 0.0  # No change in Z
        
        transform.transform.translation.x += 0.1
        transform.transform.translation.y += 0.2
        transform.transform.translation.z += 0.0  # No change in Z


        # Update yaw
        yaw += delta_theta

        # Convert euler angles back to quaternion
        quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)

        # Update transform rotation quaternion
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # Publish the updated transform
        transform_brodcaster.sendTransform(transform)
        
        print(transform)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to update transform: %s", str(e))

def pubInitialTf(transform_broadcaster):
    target_frame = "base_link"  # Update this with your target frame
    source_frame = "world"  # Update this with your source frame

    # Initialize the initial transform (e.g., at the start they are assumed to be the same)
    transform = TransformStamped()
    transform.header.frame_id = target_frame
    transform.child_frame_id = source_frame
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    transform_broadcaster.sendTransform(transform)
        
def main():
    rospy.init_node('tf_testing_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    tf_buffer = tf2_ros.Buffer()
    transform_broadcaster = tf2_ros.TransformBroadcaster()
    listener = tf2_ros.TransformListener(tf_buffer)

    pubInitialTf(transform_broadcaster)
    
    while not rospy.is_shutdown():
        
        updateTf(transform_broadcaster, tf_buffer)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
