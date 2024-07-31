#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri
import rospy
from geometry_msgs.msg import Pose
import tf
import numpy as np
import pybullet as p

class PoseListener:
    def __init__(self):
        self.wall_arm_pose = None
        self.ceiling_arm_pose = None
        self.prev_wall_arm_pose = None
        self.prev_ceiling_arm_pose = None

        # rospy.init_node('pose_listener', anonymous=True)
        self.prev_time = rospy.Time.now()
        rospy.Subscriber('/wall_arm_ee_pose_capture', Pose, self.wall_arm_pose_callback)
        rospy.Subscriber('/ceiling_arm_ee_pose_capture', Pose, self.ceiling_arm_pose_callback)

    def wall_arm_pose_callback(self, data):
        self.prev_wall_arm_pose = self.wall_arm_pose
        self.wall_arm_pose = data

    def ceiling_arm_pose_callback(self, data):
        self.prev_ceiling_arm_pose = self.ceiling_arm_pose
        self.ceiling_arm_pose = data

    def get_latest_poses(self):
        rospy.sleep(0.5)  # Allow time for callbacks to process
        position_wall_arm = [self.wall_arm_pose.position.x, self.wall_arm_pose.position.y, self.wall_arm_pose.position.z]
        position_ceiling_arm = [self.ceiling_arm_pose.position.x, self.ceiling_arm_pose.position.y, self.ceiling_arm_pose.position.z]

        # euler_wall_arm = tf.transformations.euler_from_quaternion([self.wall_arm_pose.orientation.x,
        #                                                             self.wall_arm_pose.orientation.y,
        #                                                             self.wall_arm_pose.orientation.z,
        #                                                             self.wall_arm_pose.orientation.w])
        euler_wall_arm = p.getEulerFromQuaternion([self.wall_arm_pose.orientation.x,
                                                            self.wall_arm_pose.orientation.y,
                                                            self.wall_arm_pose.orientation.z,
                                                            self.wall_arm_pose.orientation.w])
        # euler_ceiling_arm = tf.transformations.euler_from_quaternion([self.ceiling_arm_pose.orientation.x,
        #                                                     self.ceiling_arm_pose.orientation.y,
        #                                                     self.ceiling_arm_pose.orientation.z,
        #                                                     self.ceiling_arm_pose.orientation.w])
        euler_ceiling_arm = p.getEulerFromQuaternion([self.ceiling_arm_pose.orientation.x,
                                                            self.ceiling_arm_pose.orientation.y,
                                                            self.ceiling_arm_pose.orientation.z,
                                                            self.ceiling_arm_pose.orientation.w])
        position_wall_arm = np.array(position_wall_arm)
        position_ceiling_arm = np.array(position_ceiling_arm)
        euler_wall_arm = np.array(euler_wall_arm)
        euler_ceiling_arm = np.array(euler_ceiling_arm)
        return [position_wall_arm, euler_wall_arm], [position_ceiling_arm, euler_ceiling_arm]

    def calculate_velocity(self):
        current_time = rospy.Time.now()
        time_diff = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        if self.prev_wall_arm_pose is None or self.prev_ceiling_arm_pose is None:
            return None, None  # Cannot calculate velocity without previous pose

        # Calculate wall arm velocities
        pos_diff_wall = np.array([self.wall_arm_pose.position.x - self.prev_wall_arm_pose.position.x,
                                  self.wall_arm_pose.position.y - self.prev_wall_arm_pose.position.y,
                                  self.wall_arm_pose.position.z - self.prev_wall_arm_pose.position.z])
        vel_linear_wall = pos_diff_wall / time_diff

        quat_current_wall = [self.wall_arm_pose.orientation.x,
                             self.wall_arm_pose.orientation.y,
                             self.wall_arm_pose.orientation.z,
                             self.wall_arm_pose.orientation.w]
        quat_prev_wall = [self.prev_wall_arm_pose.orientation.x,
                          self.prev_wall_arm_pose.orientation.y,
                          self.prev_wall_arm_pose.orientation.z,
                          self.prev_wall_arm_pose.orientation.w]
        euler_current_wall = tf.transformations.euler_from_quaternion(quat_current_wall)
        euler_prev_wall = tf.transformations.euler_from_quaternion(quat_prev_wall)
        euler_diff_wall = np.array(euler_current_wall) - np.array(euler_prev_wall)
        vel_angular_wall = euler_diff_wall / time_diff

        # Calculate ceiling arm velocities
        pos_diff_ceiling = np.array([self.ceiling_arm_pose.position.x - self.prev_ceiling_arm_pose.position.x,
                                     self.ceiling_arm_pose.position.y - self.prev_ceiling_arm_pose.position.y,
                                     self.ceiling_arm_pose.position.z - self.prev_ceiling_arm_pose.position.z])
        vel_linear_ceiling = pos_diff_ceiling / time_diff

        quat_current_ceiling = [self.ceiling_arm_pose.orientation.x,
                                self.ceiling_arm_pose.orientation.y,
                                self.ceiling_arm_pose.orientation.z,
                                self.ceiling_arm_pose.orientation.w]
        quat_prev_ceiling = [self.prev_ceiling_arm_pose.orientation.x,
                             self.prev_ceiling_arm_pose.orientation.y,
                             self.prev_ceiling_arm_pose.orientation.z,
                             self.prev_ceiling_arm_pose.orientation.w]
        euler_current_ceiling = tf.transformations.euler_from_quaternion(quat_current_ceiling)
        euler_prev_ceiling = tf.transformations.euler_from_quaternion(quat_prev_ceiling)
        euler_diff_ceiling = np.array(euler_current_ceiling) - np.array(euler_prev_ceiling)
        vel_angular_ceiling = euler_diff_ceiling / time_diff

        return (vel_linear_wall, vel_angular_wall), (vel_linear_ceiling, vel_angular_ceiling)

if __name__ == '__main__':
    pose_listener = PoseListener()
    wall_arm_pose, ceiling_arm_pose = pose_listener.get_latest_poses()
    vel_wall, vel_ceiling = pose_listener.calculate_velocity()

    print("Wall arm pose: ", wall_arm_pose)
    print("Ceiling arm pose: ", ceiling_arm_pose)
    
    print("Wall arm velocity: ", vel_wall)
    print("Ceiling arm velocity: ", vel_ceiling)