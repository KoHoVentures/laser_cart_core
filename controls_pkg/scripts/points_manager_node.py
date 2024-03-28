#!/usr/bin/env python

import rospy
import tf2_ros

import tf.transformations as tf_trans
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, Header, Int64MultiArray

import time
import math
import numpy as np
from scipy.spatial.transform import Rotation

class PointsManagerNode():
    def __init__(self):
        
        rospy.init_node('points_manager_node', anonymous=True)

        # ROS params
        self.min_dist_to_transition = float(rospy.get_param('~min_dist_to_transition'))
        self.steps_per_mm = int(rospy.get_param('~steps_per_mm')) # From arduino
        self.wheel_radius = float(rospy.get_param('~wheel_radius'))
        self.frame_base_length = float(rospy.get_param('~frame_base_length'))
        self.points_threshold = float(rospy.get_param('~points_threshold'))

        self.ticks_per_revolution = int(rospy.get_param('~ticks_per_revolution')) # Width of the robot base (distance between wheels)
        
        
        self.tf_buffer = tf2_ros.Buffer()
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.points_list = [[0.01, 0.15, 0.0], [0.1, 0.1, 0.0]] # in m
        # points_list = self.populate_points_fixed_threshold(points_list, self.points_threshold)

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

        self.first_update_tf = True
        
        # Create subscriber
        stepper_state_subscriber = rospy.Subscriber('cur_pos', PointStamped, self.curPosCallback)
        encoder_state_subscriber = rospy.Subscriber('wheels_rad_topic', PointStamped, self.encoderCallback)

        # Other initilizations
        self.prev_theta_l = 0.0
        self.prev_theta_r = 0.0
        
        self.onStartup()

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
    
    def initPoint(self, given_point, frame_id):
        point_stamped_msg = PointStamped()

        point_stamped_msg.header = Header()
        point_stamped_msg.header.frame_id = frame_id
        point_stamped_msg.header.stamp = rospy.Time.now()

        point_stamped_msg.point.x = float(given_point[0])
        point_stamped_msg.point.y = float(given_point[1])
        point_stamped_msg.point.z = float(given_point[2])

        return point_stamped_msg
    

    def run(self):

        cur_ind = 0

        self.points_list_ros = []

        for point in self.points_list:
            self.points_list_ros.append(self.initPoint(point, frame_id = "world"))

        self.home_point = self.initPoint([0.0, 0.0, 0.0], frame_id="world")
        
        while not rospy.is_shutdown():
            
            goal_point_world = self.points_list_ros[cur_ind]
            try:
                transform = self.tf_buffer.lookup_transform("world", "base_link", rospy.Time())
                goal_point = self.tf_buffer.transform(goal_point_world, "base_link")
                # rospy.loginfo("Transformed point: (%f, %f, %f) in frame: %s", goal_point_world.point.x, goal_point_world.point.y, goal_point_world.point.z, goal_point_world.header.frame_id)
            except tf2_ros.TransformException as ex:
                rospy.logerr("Failed to transform point: %s", ex)
                continue
            
            self.marker_set_pos_publisher.publish(goal_point) # TODO: publishes move at node freq so might wanna change this
            # print("Goal point: "+str(goal_point))
            self.marker_get_pos_publisher.publish(self.get_pos_msg) # Populates self.cur_pos with the latest position

            pos_reached = self.pointReached2D(self.cur_pos, goal_point, self.min_dist_to_transition)

            if pos_reached:
                cur_ind += 1
                print("Moving to point number " + str(cur_ind + 1))
            
            if cur_ind >= len(self.points_list_ros):
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

    def getInitialTf(self):
        target_frame = "world"  # Update this with your target frame
        source_frame = "base_link"  # Update this with your source frame

        # Initialize the initial transform (e.g., at the start they are assumed to be the same)
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = source_frame
        transform.child_frame_id = target_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        return transform
        # self.transform_broadcaster.sendTransform(transform)

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
        #print("Encoder received, left: "+str(msg.point.x) + ", right: "+str(msg.point.y))

        delta_theta_l = msg.point.x - self.prev_theta_l # hacky, should have been erray but debug taking too long
        delta_theta_r = msg.point.y - self.prev_theta_r# hacky, should have been erray but debug taking too long
        
        self.prev_theta_l = msg.point.x
        self.prev_theta_r = msg.point.y
        
        # # Cap encoder vals to ticks_per_revolution
        # delta_theta_l = delta_theta_l % self.ticks_per_revolution
        # delta_theta_r = delta_theta_r % self.ticks_per_revolution
        
        # Convert encoder values to radians
        delta_theta_l = delta_theta_l * 2 * math.pi / self.ticks_per_revolution
        delta_theta_r = delta_theta_r * 2 * math.pi / self.ticks_per_revolution
        
        # print("Theta l: "+str(delta_theta_l)+" | Theta r: "+str(delta_theta_r))
            
        delta_s = (self.wheel_radius / 2) * (delta_theta_r + delta_theta_l)
        delta_theta = (self.wheel_radius / self.frame_base_length) * (delta_theta_r - delta_theta_l)

        # print("Delta s: "+str(delta_s)+" | Delta theta: "+str(delta_theta))
        
        self.updateTf(delta_s, delta_theta)

    def updateTf(self, delta_s = 0.1, delta_theta = np.pi / 8):
        
        if(self.first_update_tf):
            self.first_update_tf = False
            transform = self.getInitialTf()
            self.transform_broadcaster.sendTransform(transform)
            return
        
        try:
            tf_cur = self.tf_buffer.lookup_transform("world", "base_link", rospy.Time())

            roll, pitch, yaw = tf_trans.euler_from_quaternion([tf_cur.transform.rotation.x,
                                                               tf_cur.transform.rotation.y,
                                                               tf_cur.transform.rotation.z,
                                                               tf_cur.transform.rotation.w])
            
            tf_cur.transform.translation.x += delta_s * np.cos(yaw + delta_theta / 2.0)
            tf_cur.transform.translation.y += delta_s * np.sin(yaw + delta_theta / 2.0)
            tf_cur.transform.translation.z = 0.0  # No change in Z
            
            tf_cur_homogen = self.quaternion_translation_to_homogeneous(np.array([tf_cur.transform.rotation.x,
                                                                                  tf_cur.transform.rotation.y,
                                                                                  tf_cur.transform.rotation.z,
                                                                                  tf_cur.transform.rotation.w]),
                                                                        np.array([tf_cur.transform.translation.x,
                                                                                  tf_cur.transform.translation.y,
                                                                                  tf_cur.transform.translation.z]))
            yaw = delta_theta
            
            # Convert euler angles back to quaternion
            q_rot = tf_trans.quaternion_from_euler(0.0, 0.0, yaw)
            tf_rot_homogen = self.quaternion_translation_to_homogeneous(np.array(q_rot),
                                                                        np.array([0.0, 0, 0]))
            
            tf_transformed_homogen = np.matmul(tf_cur_homogen, tf_rot_homogen)
            
            tf_transformed_quat, tf_transformed_trans = self.homogeneous_to_quaternion_and_translation(tf_transformed_homogen)

            
            # Updated tf
            tf_transformed = TransformStamped()
            
            tf_transformed.header.stamp = rospy.Time.now()
            tf_transformed.header.frame_id = tf_cur.header.frame_id
            tf_transformed.child_frame_id = tf_cur.child_frame_id
            
            tf_transformed.transform.translation.x = tf_transformed_trans[0]
            tf_transformed.transform.translation.y = tf_transformed_trans[1]
            tf_transformed.transform.translation.z = tf_transformed_trans[2]

            # Update transform rotation quaternion
            tf_transformed.transform.rotation.x = tf_transformed_quat[0]
            tf_transformed.transform.rotation.y = tf_transformed_quat[1]
            tf_transformed.transform.rotation.z = tf_transformed_quat[2]
            tf_transformed.transform.rotation.w = tf_transformed_quat[3]
            
            # Publish the updated transform
            self.transform_broadcaster.sendTransform(tf_transformed)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Failed to update transform: %s", str(e))
    
    def quaternion_translation_to_homogeneous(self, quaternion, translation):
        """
        Convert quaternion and translation values to a homogeneous transformation matrix.

        Parameters:
        - quaternion: numpy array representing the quaternion [x, y, z, w]
        - translation: numpy array representing the translation [tx, ty, tz]

        Returns:
        - homogeneous_matrix: 4x4 numpy array representing the homogeneous transformation matrix
        """
        # Convert quaternion to rotation matrix
        rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

        # Create homogeneous transformation matrix
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = rotation_matrix
        homogeneous_matrix[:3, 3] = translation

        return homogeneous_matrix
    
    def homogeneous_to_quaternion_and_translation(self, matrix):
        # Extract rotation matrix from the homogeneous transformation matrix
        rotation_matrix = matrix[:3, :3]

        # Convert the rotation matrix to quaternion
        quaternion = Rotation.from_matrix(rotation_matrix).as_quat()

        # Extract translation vector from the homogeneous transformation matrix
        translation = matrix[:3, 3]

        return quaternion, translation

if __name__ == '__main__':
    try:
        node = PointsManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    # finally:
    #     print("Dsiabling steppers")
    #     node.onShutDown()
