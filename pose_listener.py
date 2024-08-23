#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri
import threading
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf.transformations as tf_trans

class PoseListener:
    def __init__(self):
        # Global variables to store the latest poses and velocities
        self.latest_info_c_a_tool0 = {
            'position': None,
            'orientation_euler': None,
            'linear_velocity': None,
            'angular_velocity': None
        }

        self.latest_info_w_a_tool0 = {
            'position': None,
            'orientation_euler': None,
            'linear_velocity': None,
            'angular_velocity': None
        }

        self.previous_pose_c_a_tool0 = None
        self.previous_pose_w_a_tool0 = None

        self.lock_c_a_tool0 = threading.Lock()
        self.lock_w_a_tool0 = threading.Lock()

        # Initialize the ROS node
        rospy.init_node('pose_listener', anonymous=True)

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    def quaternion_to_euler(self, quat):
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        euler = tf_trans.euler_from_quaternion(quaternion)
        return euler

    # Calculate linear velocity
    def calculate_linear_velocity(self, prev_pose, curr_pose, delta_time):
        dx = curr_pose.pose.position.x - prev_pose.pose.position.x
        dy = curr_pose.pose.position.y - prev_pose.pose.position.y
        dz = curr_pose.pose.position.z - prev_pose.pose.position.z

        linear_velocity = np.array([dx/delta_time, dy/delta_time, dz/delta_time])
        return linear_velocity

    # Calculate angular velocity based on Euler angles
    def calculate_angular_velocity(self, prev_euler, curr_euler, delta_time):
        droll = curr_euler[0] - prev_euler[0]
        dpitch = curr_euler[1] - prev_euler[1]
        dyaw = curr_euler[2] - prev_euler[2]

        angular_velocity = np.array([droll/delta_time, dpitch/delta_time, dyaw/delta_time])
        return angular_velocity

    # Callback function for /capture_c_a_tool0/pose
    def c_a_tool0_callback(self, msg):
        curr_euler = self.quaternion_to_euler(msg.pose.orientation)

        with self.lock_c_a_tool0:
            if self.previous_pose_c_a_tool0 is not None:
                delta_time = (msg.header.stamp - self.previous_pose_c_a_tool0.header.stamp).to_sec()

                if delta_time > 0:
                    prev_euler = self.quaternion_to_euler(self.previous_pose_c_a_tool0.pose.orientation)
                    linear_velocity = self.calculate_linear_velocity(self.previous_pose_c_a_tool0, msg, delta_time)
                    angular_velocity = self.calculate_angular_velocity(prev_euler, curr_euler, delta_time)

                    self.latest_info_c_a_tool0['linear_velocity'] = linear_velocity
                    self.latest_info_c_a_tool0['angular_velocity'] = angular_velocity

            self.latest_info_c_a_tool0['position'] = msg.pose.position
            self.latest_info_c_a_tool0['orientation_euler'] = curr_euler

            self.previous_pose_c_a_tool0 = msg

    # Callback function for /capture_w_a_tool0/pose
    def w_a_tool0_callback(self, msg):
        curr_euler = self.quaternion_to_euler(msg.pose.orientation)

        with self.lock_w_a_tool0:
            if self.previous_pose_w_a_tool0 is not None:
                delta_time = (msg.header.stamp - self.previous_pose_w_a_tool0.header.stamp).to_sec()

                if delta_time > 0:
                    prev_euler = self.quaternion_to_euler(self.previous_pose_w_a_tool0.pose.orientation)
                    linear_velocity = self.calculate_linear_velocity(self.previous_pose_w_a_tool0, msg, delta_time)
                    angular_velocity = self.calculate_angular_velocity(prev_euler, curr_euler, delta_time)

                    self.latest_info_w_a_tool0['linear_velocity'] = linear_velocity
                    self.latest_info_w_a_tool0['angular_velocity'] = angular_velocity

            self.latest_info_w_a_tool0['position'] = msg.pose.position
            self.latest_info_w_a_tool0['orientation_euler'] = curr_euler

            self.previous_pose_w_a_tool0 = msg

    def start_listening(self):
        # Start the listeners in separate threads
        thread_c_a = threading.Thread(target=self._listen_c_a_tool0)
        thread_w_a = threading.Thread(target=self._listen_w_a_tool0)

        thread_c_a.start()
        thread_w_a.start()

    def _listen_c_a_tool0(self):
        rospy.Subscriber('/capture_c_a_tool1/pose', PoseStamped, self.c_a_tool0_callback)
        rospy.spin()  # Keeps the thread alive

    def _listen_w_a_tool0(self):
        rospy.Subscriber('/capture_ot_offset/pose', PoseStamped, self.w_a_tool0_callback)
        rospy.spin()  # Keeps the thread alive

    def get_latest_c_a_tool0_info(self):
        with self.lock_c_a_tool0:
            return self.latest_info_c_a_tool0

    def get_latest_w_a_tool0_info(self):
        with self.lock_w_a_tool0:
            return self.latest_info_w_a_tool0
