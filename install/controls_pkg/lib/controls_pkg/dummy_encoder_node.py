#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import math

class DummyEncoderNode:
    def __init__(self):
        rospy.init_node('dummy_encoder_node')
        
        # Publisher
        self.pub = rospy.Publisher('/wheels_rad_topic', PointStamped, queue_size=10)

        # ROS param
        self.node_freq =  int(rospy.get_param('~node_freq'))
        self.rate = rospy.Rate(self.node_freq)  # Adjust the rate as needed

        
        self.forward_distance = float(rospy.get_param('~forward_distance'))  # Distance to move forward (in meters)
        self.turn_angle = float(rospy.get_param('~turn_angle'))  # Radius of the wheels  # Angle to turn (in radians)
        self.straight_speed = float(rospy.get_param('~straight_speed'))  # Speed to move straight (in m/s)
        self.turn_speed = float(rospy.get_param('~turn_speed'))  # Speed to turn (in rad/s)

        self.wheel_radius = float(rospy.get_param('~wheel_radius'))  # Radius of the wheels
        self.base_width = float(rospy.get_param('~frame_base_length')) # Width of the robot base (distance between wheels)
        
        self.ticks_per_revolution = int(rospy.get_param('~ticks_per_revolution')) # Width of the robot base (distance between wheels)
        
        self.left_encoder_value = 0.0
        self.right_encoder_value = 0.0
        
    def compute_encoder_speeds(self, linear_velocity, angular_velocity):
        # Convert linear and angular velocities to wheel speeds
        left_wheel_speed = (linear_velocity - (angular_velocity * self.base_width / 2)) / self.wheel_radius
        right_wheel_speed = (linear_velocity + (angular_velocity * self.base_width / 2)) / self.wheel_radius

        # Convert wheel speeds to encoder values
        left_encoder_value_per_second = self.convert_speed_to_encoder(left_wheel_speed)
        right_encoder_value_per_second = self.convert_speed_to_encoder(right_wheel_speed)

        return left_encoder_value_per_second, right_encoder_value_per_second

    def convert_speed_to_encoder(self, wheel_speed):
        # Convert wheel speed (rad/s) to encoder value
        ticks_per_second = wheel_speed * self.ticks_per_revolution / (2 * math.pi)
        return ticks_per_second

    def move_forward(self, delta_time):

        left_encoder_value_per_second, right_encoder_value_per_second = self.compute_encoder_speeds(self.straight_speed, 0.0)
        
        self.left_encoder_value += left_encoder_value_per_second * delta_time
        self.right_encoder_value += right_encoder_value_per_second * delta_time

        return (self.left_encoder_value, self.right_encoder_value)

    def turn(self, delta_time):

        left_encoder_value_per_second, right_encoder_value_per_second = self.compute_encoder_speeds(0.0, self.turn_speed)
        
        self.left_encoder_value += left_encoder_value_per_second * delta_time
        self.right_encoder_value += right_encoder_value_per_second * delta_time

        return (self.left_encoder_value, self.right_encoder_value)

    def pubEncoderVals(self):
        wheel_values = PointStamped()
        wheel_values.header.stamp = rospy.Time.now()
        wheel_values.header.frame_id = "wheel_odom"  # Modify this frame ID as needed
        wheel_values.point = Point(self.left_encoder_value, self.right_encoder_value, 0.0)  # Moving forward, angle moved is 0
        self.pub.publish(wheel_values)
    
    def run(self):
        while not rospy.is_shutdown():
            # Move forward
            print("Moving forward")
            for _ in range(int(self.forward_distance / self.straight_speed)):
                self.move_forward(1/self.node_freq)
                self.pubEncoderVals()
                self.rate.sleep()

            # # Turn
            print("Turning")
            for _ in range(int(self.turn_angle / self.turn_speed)):
                self.turn(1/self.node_freq)
                self.pubEncoderVals()
                self.rate.sleep()           

if __name__ == '__main__':
    try:
        node = DummyEncoderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
        
