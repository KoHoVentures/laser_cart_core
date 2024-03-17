#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def input_handler():
    input_str = input("Enter x y z coordinates separated by spaces: ")
    coords = input_str.split()
    if len(coords) != 3:
        rospy.logwarn("Invalid input format. Please enter x y z coordinates separated by spaces.")
        return None
    try:
        x = float(coords[0])
        y = float(coords[1])
        z = float(coords[2])
        return Point(x=x, y=y, z=z)
    except ValueError:
        rospy.logwarn("Invalid input format. Please enter valid numerical values for x y z coordinates.")
        return None

def main():
    rospy.init_node('point_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    pub = rospy.Publisher('marker_position_topic', Point, queue_size=10)

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
