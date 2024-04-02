#!/usr/bin/env python

import rospy
import tf2_ros

import tf.transformations as tf_trans
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import Bool, Header, Int64MultiArray

import time
import math
import numpy as np
import csv
from scipy.spatial.transform import Rotation

class Point:
    def __init__(self, point_xyz, frame="world", move_type=0):
        self.x = point_xyz[0]
        self.y = point_xyz[1]
        self.z = point_xyz[2]
        
        self.point_stamped_msg = PointStamped()

        self.point_stamped_msg.header = Header()
        self.point_stamped_msg.header.frame_id = frame
        self.point_stamped_msg.header.stamp = rospy.Time.now()

        self.point_stamped_msg.point.x = float(point_xyz[0])
        self.point_stamped_msg.point.y = float(point_xyz[1])
        self.point_stamped_msg.point.z = float(point_xyz[2])

        self.move_type = move_type
    
    def __repr__(self):
        return f"Point object with value {self.x}, {self.y}, {self.z}, {self.point_stamped_msg.header.frame_id}, {self.move_type} \n"
        
class PointsManagerNode:
    def __init__(self):
        
        # Initilaize ros ndoe
        rospy.init_node('points_manager_node', anonymous=True)

        # ROS params
        self.min_dist_to_transition = float(rospy.get_param('~min_dist_to_transition'))
        self.steps_per_mm = int(rospy.get_param('~steps_per_mm')) # From arduino
        self.wheel_radius = float(rospy.get_param('~wheel_radius'))
        self.frame_base_length = float(rospy.get_param('~frame_base_length'))
        self.points_threshold = float(rospy.get_param('~points_threshold'))

        self.ticks_per_revolution = int(rospy.get_param('~ticks_per_revolution')) # Width of the robot base (distance between wheels)
        filename = rospy.get_param('~points_file_name')
        self.sim_enabled = rospy.get_param('~sim_enabled')
        self.interpolation_enabled = rospy.get_param('~interpolation_enabled')
        self.pen_wait_duration = float(rospy.get_param('~pen_wait_duration'))
        self.node_freq = int(rospy.get_param('~node_freq'))
        
        # ROS tfs
        self.tf_buffer = tf2_ros.Buffer()
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.points_list = []
        # Extract points from csv
        with open(filename, 'r') as file:
            # Create a CSV reader object
            csv_reader = csv.DictReader(file)
            
            # Iterate over each row in the CSV file
            for row in csv_reader:
                new_point = [float(row['X']), float(row['Y']), 0.0]
                move = int(row['Move'])
                self.points_list.append(Point(new_point, frame="world", move_type=int(row['Move'])))
        
        # Interpolation
        if(self.interpolation_enabled):
            self.points_list = self.populate_points_fixed_threshold(self.points_list, self.points_threshold)
        
        # Append home point at start for a different move type
        self.points_list.insert(0, Point(point_xyz = [0.01, 0.01, 0.0], frame = "world", move_type=self.points_list[0].move_type - 1)) # Append home at the start
        
        # Append home point at end
        self.home_point = Point([0.01, 0.01, 0.0], "pointer_link", self.points_list[-1].move_type + 1)
        self.points_list.append(self.home_point)
        
        
        self.rate = rospy.Rate(self.node_freq)  # 10hz

        # ROS publishers
        self.marker_set_pos_publisher = rospy.Publisher('marker_position_topic', PointStamped, queue_size=10)
        self.marker_get_pos_publisher = rospy.Publisher('get_marker_cur_pos', Bool, queue_size=10)
        self.stepper_enable_disable_publisher = rospy.Publisher('stepper_enable_disable_topic', Bool, queue_size=10)
        self.pen_enable_disable_publisher = rospy.Publisher('pen_enable_disable_topic', Bool, queue_size=10)
        
        # ROS subscriber
        stepper_state_subscriber = rospy.Subscriber('cur_pos', PointStamped, self.curPosCallback)
        encoder_state_subscriber = rospy.Subscriber('wheels_rad_topic', PointStamped, self.encoderCallback)

        # Initialization

        self.get_pos_msg = Bool()
        self.enable_stepper_msg = Bool()
        self.enable_pen_msg = Bool()
    
        self.get_pos_msg.data = True
        self.enable_stepper_msg.data = True
        self.enable_pen_msg.data = True

        self.cur_pos = PointStamped()

        self.first_update_tf = True
        self.pen_pos_changed = False
        self.encoders_enabled = False
        
        self.prev_theta_l = 0.0
        self.prev_theta_r = 0.0
        
        self.onStartup()
        
        
    def enable_disable_stepper(self, val):
        """ Enable or disable steppers

        Args:
            val (bool): true enables steppers, false disables them
        """
        self.enable_stepper_msg.data = val
        self.stepper_enable_disable_publisher.publish(self.enable_stepper_msg)
    
    def enable_disable_pen(self, val):
        """Enable or disable pen

        Args:
            val (bool): true enables pens, false disables it
        """
        self.enable_pen_msg.data = val
        self.pen_enable_disable_publisher.publish(self.enable_pen_msg)
        

    def interpolate_points(self, point1, point2, num_intermediate_points):
        """_summary_

        Args:
            point1 (Point): point1
            point2 (Point): point2
            num_intermediate_points (int): number of intermediate points

        Returns:
            interpolated_points: list of interpolated Points
        """
        interpolated_points = []
        for i in range(1, num_intermediate_points + 1):
            alpha = i / (num_intermediate_points + 1)
            interpolated_point_xyz = [
                point1.x * (1 - alpha) + point2.x * alpha,
                point1.y * (1 - alpha) + point2.y * alpha,
                point1.z * (1 - alpha) + point2.z * alpha
            ]
            
            interpolated_point = Point(interpolated_point_xyz, frame = "world", move_type=point1.move_type)
            
            interpolated_points.append(interpolated_point)
        return interpolated_points

    def populate_points_fixed_threshold(self, points_list, threshold):
        new_points = []
        for i in range(len(points_list) - 1):
            point1 = points_list[i]
            new_points.append(point1)  # Include original point
            point2 = points_list[i + 1]
            distance = max(abs(point2.x - point1.x), abs(point2.y - point1.y), abs(point2.z - point1.z))
            num_intermediate_points = int(distance / threshold)
            if num_intermediate_points > 0:
                intermediate_points = self.interpolate_points(point1, point2, num_intermediate_points)
                new_points.extend(intermediate_points)
        new_points.append(points_list[-1])  # Include last original point
        return new_points
    

    def run(self):
        """
        Node run function
        """
        
        self.cur_point_world = self.points_list[0]
        cur_ind = 1
        
        while not rospy.is_shutdown():
            
            goal_point_world = self.points_list[cur_ind]
            
            # Enable pen if move type matches
            if(self.cur_point_world.move_type == goal_point_world.move_type):
                self.enable_disable_pen(True)
                if(not self.pen_pos_changed):
                    self.pen_pos_changed = True
                    time.sleep(self.pen_wait_duration)
            else:
                self.enable_disable_pen(False) # Disable pen if move types dont match
                self.pen_pos_changed = False
                
            # Transform points to pointer link
            try: 
                if(goal_point_world.point_stamped_msg.header.frame_id == "pointer_link"): 
                    tf_homogen = np.eye(4)
                else:    
                    tf_cur = self.tf_buffer.lookup_transform("world", "pointer_link", rospy.Time())
                    
                    tf_homogen = self.quaternion_translation_to_homogeneous(np.array([tf_cur.transform.rotation.x,
                                                                                    tf_cur.transform.rotation.y,
                                                                                    tf_cur.transform.rotation.z,
                                                                                    tf_cur.transform.rotation.w]),
                                                                            np.array([tf_cur.transform.translation.x,
                                                                                    tf_cur.transform.translation.y,
                                                                                    tf_cur.transform.translation.z]))
                    tf_homogen = np.linalg.inv(tf_homogen)

                goal_transformed = np.matmul(tf_homogen, np.array([goal_point_world.x,
                                            goal_point_world.y,
                                            goal_point_world.z,
                                            1]))
                goal_point_pointer_link = Point(goal_transformed[:3], "pointer_link", goal_point_world.move_type)

            except tf2_ros.TransformException as ex:
                
                if(self.encoders_enabled):
                    rospy.logerr("Failed to transform point: %s", ex)
                else:
                    rospy.logerr("Encoders not enabled yet")
                continue
            
            self.marker_set_pos_publisher.publish(goal_point_pointer_link.point_stamped_msg) # TODO: publishes move at node freq so might wanna change this

            self.marker_get_pos_publisher.publish(self.get_pos_msg) # Populates self.cur_pos with the latest position
            
            # Resey cur pos only (only for sim)
            if(self.sim_enabled):
                self.cur_pos.point.x = 0
                self.cur_pos.point.y = 0
                self.cur_pos.point.z = 0
            
            # Check if pos was reached
            pos_reached = self.pointReached2D(self.cur_pos, goal_point_pointer_link, self.min_dist_to_transition)

            if pos_reached:
                self.cur_point_world = goal_point_world
                cur_ind += 1
                print("Moving to point number " + str(cur_ind + 1))
            
            if cur_ind >= len(self.points_list):
                print("Point list complete, going home")
                # self.goHome()
                #self.onShutDown()

                print("Killing node")
                return
                
            self.rate.sleep()

        # Node done running

    def onStartup(self):
        """Enable steppers and disable pen on startup - hacky implementation since ROS serial drops sometimes on startup
        """
        
        print("Enabling steppers from pi")
        start_time = time.time()
        spam_time = 1
        spam_freq = 10 # Hz
        while (time.time() - start_time) < spam_time:  # Loop for 3 seconds
            self.enable_disable_stepper(True) # ENable steppers
            self.enable_disable_pen(False) # Disable pen
            time.sleep(1/spam_freq)  # Sleep for 1 second

    def pointReached2D(self, cur_pos, goal_point, point_threshold):

        x_diff = cur_pos.point.x - goal_point.x
        y_diff = cur_pos.point.y - goal_point.y

        dist = math.sqrt(x_diff**2 + y_diff**2)
        
        if(dist <= point_threshold):
            return True
        else:
            return False

    def getInitialTf(self):
        """
        get initila tf ro be defined between world and base link
        Returns:
            transform: initial tf between world and base link
        """
        target_frame = "world"
        source_frame = "base_link"

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

    def curPosCallback(self, msg):
        """
        ROS call back to store current position from encoder step values

        Args:
            msg (_type_): _description_
        """
        Asteps = msg.point.x
        Bsteps = msg.point.y

        x = ((Asteps + Bsteps) / (2 * self.steps_per_mm)) / 1000.0
        y = ((Asteps - Bsteps) / (2 * self.steps_per_mm)) / 1000.0
        z = 0.0 # TODO

        self.cur_pos.header = msg.header
        self.cur_pos.point.x = x
        self.cur_pos.point.y = y
        self.cur_pos.point.z = z

    def encoderCallback(self, msg):
        # Encoder call back
        
        delta_theta_l = msg.point.x - self.prev_theta_l
        delta_theta_r = msg.point.y - self.prev_theta_r
        
        self.prev_theta_l = msg.point.x
        self.prev_theta_r = msg.point.y
        
        # # Cap encoder vals to ticks_per_revolution
        # delta_theta_l = delta_theta_l % self.ticks_per_revolution
        # delta_theta_r = delta_theta_r % self.ticks_per_revolution
        
        # Convert encoder values to radians
        delta_theta_l = delta_theta_l * 2 * math.pi / self.ticks_per_revolution
        delta_theta_r = delta_theta_r * 2 * math.pi / self.ticks_per_revolution
        
        delta_s = (self.wheel_radius / 2) * (delta_theta_r + delta_theta_l)
        delta_theta = (self.wheel_radius / self.frame_base_length) * (delta_theta_r - delta_theta_l)

        # Update tf of base_link
        self.updateTf(delta_s, delta_theta)

    def updateTf(self, delta_s = 0.1, delta_theta = np.pi / 8):
        """
        Forward kinematics to update tf from baselink to world

        Args:
            delta_s (float, optional): delta x in robot frame. Defaults to 0.1.
            delta_theta (float, optional): delta theta in robot frame. Defaults to np.pi/8.
        """
        if(self.first_update_tf):
            # First tf if nothing was published before
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
            
            self.encoders_enabled = True
            
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
