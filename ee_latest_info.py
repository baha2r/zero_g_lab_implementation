#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf.transformations as tf_trans

# Global variables to store the latest poses and velocities
latest_info_c_a_tool0 = {
    'position': None,
    'orientation_euler': None,
    'linear_velocity': None,
    'angular_velocity': None
}

latest_info_w_a_tool0 = {
    'position': None,
    'orientation_euler': None,
    'linear_velocity': None,
    'angular_velocity': None
}

previous_pose_c_a_tool0 = None
previous_pose_w_a_tool0 = None

# Convert quaternion to Euler angles (roll, pitch, yaw)
def quaternion_to_euler(quat):
    quaternion = [quat.x, quat.y, quat.z, quat.w]
    euler = tf_trans.euler_from_quaternion(quaternion)
    return euler

# Calculate linear velocity
def calculate_linear_velocity(prev_pose, curr_pose, delta_time):
    dx = curr_pose.pose.position.x - prev_pose.pose.position.x
    dy = curr_pose.pose.position.y - prev_pose.pose.position.y
    dz = curr_pose.pose.position.z - prev_pose.pose.position.z

    linear_velocity = np.array([dx/delta_time, dy/delta_time, dz/delta_time])
    return linear_velocity

# Calculate angular velocity based on Euler angles
def calculate_angular_velocity(prev_euler, curr_euler, delta_time):
    droll = curr_euler[0] - prev_euler[0]
    dpitch = curr_euler[1] - prev_euler[1]
    dyaw = curr_euler[2] - prev_euler[2]

    angular_velocity = np.array([droll/delta_time, dpitch/delta_time, dyaw/delta_time])
    return angular_velocity

# Callback function for /capture_c_a_tool0/pose
def c_a_tool0_callback(msg):
    global latest_info_c_a_tool0, previous_pose_c_a_tool0

    curr_euler = quaternion_to_euler(msg.pose.orientation)

    if previous_pose_c_a_tool0 is not None:
        delta_time = (msg.header.stamp - previous_pose_c_a_tool0.header.stamp).to_sec()

        if delta_time > 0:
            prev_euler = quaternion_to_euler(previous_pose_c_a_tool0.pose.orientation)
            linear_velocity = calculate_linear_velocity(previous_pose_c_a_tool0, msg, delta_time)
            angular_velocity = calculate_angular_velocity(prev_euler, curr_euler, delta_time)

            latest_info_c_a_tool0['linear_velocity'] = linear_velocity
            latest_info_c_a_tool0['angular_velocity'] = angular_velocity

    latest_info_c_a_tool0['position'] = msg.pose.position
    latest_info_c_a_tool0['orientation_euler'] = curr_euler

    previous_pose_c_a_tool0 = msg

# Callback function for /capture_w_a_tool0/pose
def w_a_tool0_callback(msg):
    global latest_info_w_a_tool0, previous_pose_w_a_tool0

    curr_euler = quaternion_to_euler(msg.pose.orientation)

    if previous_pose_w_a_tool0 is not None:
        delta_time = (msg.header.stamp - previous_pose_w_a_tool0.header.stamp).to_sec()

        if delta_time > 0:
            prev_euler = quaternion_to_euler(previous_pose_w_a_tool0.pose.orientation)
            linear_velocity = calculate_linear_velocity(previous_pose_w_a_tool0, msg, delta_time)
            angular_velocity = calculate_angular_velocity(prev_euler, curr_euler, delta_time)

            latest_info_w_a_tool0['linear_velocity'] = linear_velocity
            latest_info_w_a_tool0['angular_velocity'] = angular_velocity

    latest_info_w_a_tool0['position'] = msg.pose.position
    latest_info_w_a_tool0['orientation_euler'] = curr_euler

    # print(previous_pose_w_a_tool0)

    previous_pose_w_a_tool0 = msg

def start_pose_listeners():
    rospy.init_node('pose_listener', anonymous=True)

    # Subscribers for the pose topics
    rospy.Subscriber('/capture_c_a_tool0/pose', PoseStamped, c_a_tool0_callback)
    rospy.Subscriber('/capture_w_a_tool0/pose', PoseStamped, w_a_tool0_callback)

    rospy.spin()  # Keep the node running

def get_latest_c_a_tool0_info():
    return latest_info_c_a_tool0

def get_latest_w_a_tool0_info():
    return latest_info_w_a_tool0

if __name__ == '__main__':
    try:
        start_pose_listeners()
    except rospy.ROSInterruptException:
        pass
