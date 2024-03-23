#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from std_msgs.msg import Bool

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
    rospy.init_node('auto_command_position_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Create publishers
    marker_set_pos_publisher = rospy.Publisher('marker_position_topic', Point, queue_size=10)
    marker_get_pos_publisher = rospy.Publisher('get_marker_cur_pos', Bool, queue_size=10)

    tst_msg = Bool()
    tst_msg.data = True

    # Create subscriber
    stepper_state_subscriber = rospy.Subscriber('cur_pos', Point, curPosCallback)

    while not rospy.is_shutdown():
        # point = points_list[0]

        point = input_handler()
        if point:
            marker_set_pos_publisher.publish(point)
            marker_get_pos_publisher.publish(tst_msg)
            
        rate.sleep()

def curPosCallback(msg):
    steps_per_mm = 80 # TODO match with arduino
    Asteps = msg.x
    Bsteps = msg.y

    x = (Asteps + Bsteps) / (2 * steps_per_mm)
    y = (Asteps - Bsteps) / (2 * steps_per_mm)
    z = 0.0 # TODO

    print(str(x) + " " + str(y) + " " + str(z))

def stepper_callback(msg):
    # Define your callback function for stepper_enable_disable_topic subscriber
    pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
