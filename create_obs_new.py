#!/usr/bin/env python3

import os
import numpy as np
from pose_listener import PoseListener
import time
import rospy

# Setting up ROS Master URI
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri

# Function to calculate relative position
def calculate_relative_position(position_w_a, position_c_a):
    return np.array([
        position_w_a.x - position_c_a.x,
        position_w_a.y - position_c_a.y,
        position_w_a.z - position_c_a.z
    ])

# Function to calculate relative orientation
def calculate_relative_orientation(euler_w_a, euler_c_a):
    return np.array([
        euler_w_a[0] - euler_c_a[0],
        euler_w_a[1] - euler_c_a[1],
        euler_w_a[2] - euler_c_a[2]
    ])

# Function to calculate relative velocity
def calculate_relative_velocity(velocity_w_a, velocity_c_a):
    return velocity_w_a - velocity_c_a

# Function to calculate minimum distance between cuboids
def min_distance_between_cuboids(cuboid1, cuboid2):
    """Calculate the minimum distance between two cuboids."""
    c1, (dx1, dy1, dz1) = cuboid1
    c2, (dx2, dy2, dz2) = cuboid2
    dist_x = abs(c2[0] - c1[0])
    dist_y = abs(c2[1] - c1[1])
    dist_z = abs(c2[2] - c1[2])
    sign_x = 1 if c2[0] > c1[0] else -1
    sign_y = 1 if c2[1] > c1[1] else -1
    sign_z = 1 if c2[2] > c1[2] else -1
    total_dx = dx1 + dx2
    total_dy = dy1 + dy2
    total_dz = dz1 + dz2
    sep_x = dist_x - total_dx if dist_x > total_dx else 0
    sep_y = dist_y - total_dy if dist_y > total_dy else 0
    sep_z = dist_z - total_dz if dist_z > total_dz else 0
    min_dist = np.array([-sep_x*sign_x, -sep_y*sign_y, -sep_z*sign_z])
    return min_dist

# Function to create the 39-element NumPy array
def create_robot_obs_array(pose_listener):
    rate = rospy.Rate(500)  # 10 Hz
    # Get the latest information from both robots
    c_a_info = pose_listener.get_latest_c_a_tool0_info()
    w_a_info = pose_listener.get_latest_w_a_tool0_info()

    # If data is not yet available, return an array of zeros
    if c_a_info['position'] is None or w_a_info['position'] is None:
        return np.zeros(39)
    
    while c_a_info['linear_velocity'] is None or w_a_info['linear_velocity'] is None:
        c_a_info = pose_listener.get_latest_c_a_tool0_info()
        w_a_info = pose_listener.get_latest_w_a_tool0_info()
        time.sleep(1)
        print("Waiting for velocity data...")

    # Position arrays
    position_w_a = np.array([w_a_info['position'].x, w_a_info['position'].y, w_a_info['position'].z])
    position_w_a[2] -= 0.025
    position_c_a = np.array([c_a_info['position'].x, c_a_info['position'].y, c_a_info['position'].z])

    # Orientation (Euler angles) arrays
    orientation_w_a = np.array(w_a_info['orientation_euler'])
    orientation_c_a = np.array(c_a_info['orientation_euler'])

    # Velocity arrays
    lin_vel_w_a = np.array(w_a_info['linear_velocity'])
    ang_vel_w_a = np.array(w_a_info['angular_velocity'])
    ang_vel_w_a = np.array([0,0,0])

    orientation_c_a[1] -= orientation_w_a[1]
    # self._observation[4] -= blockEul[1]
    orientation_w_a = np.array([0,0,0])

    lin_vel_c_a = np.array(c_a_info['linear_velocity'])
    ang_vel_c_a = np.array(c_a_info['angular_velocity'])

    # Relative position and orientation
    relative_position = calculate_relative_position(w_a_info['position'], c_a_info['position'])
    relative_orientation = calculate_relative_orientation(orientation_w_a, orientation_c_a)

    # Relative velocities
    relative_lin_vel = calculate_relative_velocity(lin_vel_w_a, lin_vel_c_a)
    relative_ang_vel = calculate_relative_velocity(ang_vel_w_a, ang_vel_c_a)

    # Assuming the cuboids are defined by their center positions and half-extents (dx, dy, dz)
    # Define the cuboids based on the robot end effectors' positions and assumed dimensions
    cuboid_w_a = (position_w_a, (0.075, 0.075, 0.02))  # Replace with actual dimensions
    cuboid_c_a = (position_c_a, (0.055, 0.055, 0.02))  # Replace with actual dimensions

    # Calculate minimum distance between the cuboids
    min_distance = min_distance_between_cuboids(cuboid_c_a, cuboid_w_a)
    # min_distance[0] = min_distance[0] - 0.05

    # Concatenate all the components into a single array
    robot_info_array = np.concatenate([
        position_c_a,
        orientation_c_a,
        lin_vel_c_a,
        ang_vel_c_a,
        position_w_a,
        orientation_w_a,
        relative_position,
        relative_orientation,
        lin_vel_w_a,
        ang_vel_w_a,
        relative_lin_vel,
        relative_ang_vel,
        min_distance
    ])

    rate.sleep()

    return robot_info_array

if __name__ == '__main__':
    # Initialize PoseListener and start listening
    pose_listener = PoseListener()
    pose_listener.start_listening()

    for i in range(3):

        # Give the listeners some time to start and collect data
        time.sleep(1)

        # Create and print the robot observation array
        robot_info_array = create_robot_obs_array(pose_listener)
        print("Robot Info Array:", robot_info_array)
