#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

def input_handler():
    input_str = input("Enter x y z coordinates separated by spaces: ")
    coords = input_str.split()
    if len(coords) != 3:
        rospy.logwarn("Invalid input format. Please enter x y z coordinates separated by spaces.")
        return None
    try:
        point_stamped_msg = PointStamped()

        point_stamped_msg.header = Header()
        point_stamped_msg.header.stamp = rospy.Time.now()
        point_stamped_msg.point.x = float(coords[0])
        point_stamped_msg.point.y = float(coords[1])
        point_stamped_msg.point.z = float(coords[2])

        return point_stamped_msg
    
    except ValueError:
        rospy.logwarn("Invalid input format. Please enter valid numerical values for x y z coordinates.")
        return None

def main():
    rospy.init_node('point_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    pub = rospy.Publisher('marker_position_topic', PointStamped, queue_size=10)

    while not rospy.is_shutdown():
        point = input_handler()
        if point:
            pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
