#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import math

class DummyEncoderNode:
    def __init__(self):
        rospy.init_node('dummy_encoder_node')
        
        self.pub = rospy.Publisher('/wheels_rad_topic', PointStamped, queue_size=10)
        self.node_freq = 10
        self.rate = rospy.Rate(self.node_freq)  # Adjust the rate as needed

        self.forward_distance = 1.0  # Distance to move forward (in meters)
        self.turn_angle = 90.0  # Angle to turn (in degrees)
        self.straight_speed = 0.25  # Speed to move straight (in m/s)
        self.turn_speed = 0.1  # Speed to turn (in rad/s)

        self.left_encoder_value = 0.0
        self.right_encoder_value = 0.0
        self.wheel_radius = 0.05  # Radius of the wheels
        self.base_width = 0.3 # Width of the robot base (distance between wheels)
        self.ticks_per_revolution = 1000  # Encoder ticks per wheel revolution

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


    def run(self):
        while not rospy.is_shutdown():
            # Move forward
            for _ in range(int(self.forward_distance / self.straight_speed)):
                self.move_forward(1/self.node_freq)
                self.rate.sleep()

            # Turn
            for _ in range(int(self.turn_angle / self.turn_speed)):
                self.turn(1/self.node_freq)
                self.rate.sleep()
            

            wheel_values = PointStamped()
            wheel_values.header.stamp = rospy.Time.now()
            wheel_values.header.frame_id = "wheel_odom"  # Modify this frame ID as needed
            wheel_values.point = Point(self.left_encoder_value, self.right_encoder_value, 0.0)  # Moving forward, angle moved is 0
            self.pub.publish(wheel_values)

if __name__ == '__main__':
    node = DummyEncoderNode()
    node.run()
