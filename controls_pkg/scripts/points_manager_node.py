#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Bool, Header, Int64MultiArray

import time
import math
import numpy as np

class PointsManagerNode():
    def __init__(self):
        
        rospy.init_node('points_manager_node', anonymous=True)

        # ROS params
        self.min_dist_to_transition = rospy.get_param('~min_dist_to_transition', '0.003')
        self.steps_per_mm = rospy.get_param('~steps_per_mm', '80') # From arduino
        self.wheel_radius = rospy.get_param('~wheel_radius', '0.01')
        self.frame_base_length = rospy.get_param('~frame_base_length', '0.02')
        self.points_threshold = rospy.get_param('~points_threshold', '0.001')

        self.tf_buffer = tf2_ros.Buffer()
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        points_list = [[0.01, 0.15, 0.01], [0.1, 0.1, 0.02]] # in m
        # points_list = self.populate_points_fixed_threshold(points_list, self.points_threshold)
        
        self.points_list = []

        for point in points_list:
            self.points_list.append(self.initPoint(point))

        self.home_point = self.initPoint([0.0, 0.0, 0.0])

        self.rate = rospy.Rate(10)  # 10hz

        # Create publishers
        self.marker_set_pos_publisher = rospy.Publisher('marker_position_topic', PointStamped, queue_size=10)
        self.marker_get_pos_publisher = rospy.Publisher('get_marker_cur_pos', Bool, queue_size=10)
        self.stepper_enable_disable_publisher = rospy.Publisher('stepper_enable_disable_topic', Bool, queue_size=10)

        self.get_pos_msg = Bool()
        self.enable_stepper_msg = Bool()
    
        self.get_pos_msg.data = True
        self.enable_stepper_msg.data = True

        self.cur_pos = PointStamped()

        # Create subscriber
        stepper_state_subscriber = rospy.Subscriber('cur_pos', PointStamped, self.curPosCallback)
        encoder_state_subscriber = rospy.Subscriber('wheels_rad_topic', PointStamped, self.encoderCallback)

        self.onStartup()
        self.pubInitialTf()

    def enable_stepper(self, val):

        self.enable_stepper_msg.data = val
        # Disable stepper
        self.stepper_enable_disable_publisher.publish(self.enable_stepper_msg)
        

    def interpolate_points(self, point1, point2, num_intermediate_points):
        interpolated_points = []
        for i in range(1, num_intermediate_points + 1):
            alpha = i / (num_intermediate_points + 1)
            interpolated_point = [
                point1[0] * (1 - alpha) + point2[0] * alpha,
                point1[1] * (1 - alpha) + point2[1] * alpha,
                point1[2] * (1 - alpha) + point2[2] * alpha
            ]
            interpolated_points.append(interpolated_point)
        return interpolated_points

    def populate_points_fixed_threshold(self, points_list, threshold):
        new_points = []
        for i in range(len(points_list) - 1):
            point1 = points_list[i]
            new_points.append(point1)  # Include original point
            point2 = points_list[i + 1]
            distance = max(abs(point2[0] - point1[0]), abs(point2[1] - point1[1]), abs(point2[2] - point1[2]))
            num_intermediate_points = int(distance / threshold)
            if num_intermediate_points > 0:
                intermediate_points = self.interpolate_points(point1, point2, num_intermediate_points)
                new_points.extend(intermediate_points)
        new_points.append(points_list[-1])  # Include last original point
        return new_points
    
    def initPoint(self, given_point):
        point_stamped_msg = PointStamped()

        point_stamped_msg.header = Header()
        point_stamped_msg.header.stamp = rospy.Time.now()

        point_stamped_msg.point.x = float(given_point[0])
        point_stamped_msg.point.y = float(given_point[1])
        point_stamped_msg.point.z = float(given_point[2])

        return point_stamped_msg
    

    def run(self):

        cur_ind = 0

        while not rospy.is_shutdown():

            #TEST self.updateTf()
            # self.updateTf()
            
            goal_point = self.points_list[cur_ind]
            self.marker_set_pos_publisher.publish(goal_point) # TODO: publishes move at node freq so might wanna change this
            # print("Goal point: "+str(goal_point))
            self.marker_get_pos_publisher.publish(self.get_pos_msg) # Populates self.cur_pos with the latest position

            pos_reached = self.pointReached2D(self.cur_pos, goal_point, self.min_dist_to_transition)

            if pos_reached:
                cur_ind += 1
                print("Moving to point number " + str(cur_ind + 1))
            
            if cur_ind >= len(self.points_list):
                print("Point list complete, going home")
                self.goHome()
                self.onShutDown()

                print("Killing node")
                return
                
            self.rate.sleep()

        # Node done running

    def onStartup(self):
        print("Enabling steppers from pi")
        start_time = time.time()
        spam_time = 1
        spam_freq = 10 # Hz
        while (time.time() - start_time) < spam_time:  # Loop for 3 seconds
            self.enable_stepper(True)
            time.sleep(1/spam_freq)  # Sleep for 1 second

    def goHome(self):
        start_time = time.time()
        spam_time = 1
        spam_freq = 10 # Hz

        # WARNING, bad code with magic numbers
        while (time.time() - start_time) < spam_time:
            self.marker_set_pos_publisher.publish(self.home_point)
            time.sleep(1/spam_freq)  # Sleep for 1 second

    def onShutDown(self):
        time.sleep(7) # Wait for 7 seconds
        start_time = time.time()
        spam_time = 1
        spam_freq = 10 # Hz

        print("Disabling steppers from pi")
        while (time.time() - start_time) < spam_time:
            self.enable_stepper(False)
            time.sleep(1/spam_freq)  # Sleep for 1 second

        time.sleep(7) #TODO REMOVE

    def pointReached2D(self, cur_pos, goal_pos, point_threshold):

        x_diff = cur_pos.point.x - goal_pos.point.x
        y_diff = cur_pos.point.y - goal_pos.point.y

        if(math.sqrt(x_diff**2 + y_diff**2) <= point_threshold):
            return True
        else:
            return False

    def pubInitialTf(self):
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

        self.transform_broadcaster.sendTransform(transform)

    def curPosCallback(self, msg):

        Asteps = msg.point.x
        Bsteps = msg.point.y

        x = ((Asteps + Bsteps) / (2 * self.steps_per_mm)) / 1000.0
        y = ((Asteps - Bsteps) / (2 * self.steps_per_mm)) / 1000.0
        z = 0.0 # TODO

        #print("Current position (in m): "+str(x) + " " + str(y) + " " + str(z))

        self.cur_pos.header = msg.header
        self.cur_pos.point.x = x
        self.cur_pos.point.y = y
        self.cur_pos.point.z = z

    def encoderCallback(self, msg):
        # Define your callback function
        print("Encoder received, left: "+str(msg.point.x) + ", right: "+str(msg.point.y))

        theta_l = msg.point.x # hacky, should have been erray but debug taking too long
        theta_r = msg.point.y # hacky, should have been erray but debug taking too long

        delta_s = (self.wheel_radius / 2) * (theta_r + theta_l)
        delta_theta = (self.wheel_radius / self.frame_base_length) * (theta_r - theta_l)

        self.updateTf(delta_s, delta_theta)

    def updateTf(self, delta_s = 0.1, delta_theta = np.pi / 8):
        
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "world", rospy.Time())

            transform.header.stamp = rospy.Time.now()
            
            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            (roll, pitch, yaw) = tf_trans.euler_from_quaternion(quaternion)

            transform.transform.translation.x += delta_s * np.cos(yaw + delta_theta / 2.0)
            transform.transform.translation.y += delta_s * np.sin(yaw + delta_theta / 2.0)
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
            self.transform_broadcaster.sendTransform(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Failed to update transform: %s", str(e))
        
if __name__ == '__main__':
    try:
        node = PointsManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    # finally:
    #     print("Dsiabling steppers")
    #     node.onShutDown()
