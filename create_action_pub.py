#!/usr/bin/env python3
import os
import socket
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri
docker_ip = socket.gethostbyname(socket.gethostname())
print("Docker IP:", docker_ip)
os.environ['ROS_IP'] = docker_ip

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
import tf
from stable_baselines3 import SAC
from create_obs_new import create_robot_obs_array
from pose_listener import PoseListener
import time
import pickle

def load_model():
    """Load the pre-trained SAC model."""
    model = SAC.load("./models/trained_agent/best_model.zip")
    return model

def get_action(model, obs):
    """Predict the next action based on the observation using the SAC model."""
    action, _states = model.predict(obs, deterministic=True)
    return action

# Callback function for the subscriber
def pose_callback(msg):
    global current_pose
    current_pose = msg

def return_action():
    model = load_model()
    pose_listener = PoseListener()
    pose_listener.start_listening()

    # rospy.init_node('pose_incrementer', anonymous=True)

    # Subscriber to the pose topic
    rospy.Subscriber('/capture_c_a_tool1/pose', PoseStamped, pose_callback)

    # Publisher for the updated pose
    pose_pub = rospy.Publisher('/Way_pts_target_pose', PoseStamped, queue_size=10)

    action_pun = rospy.Publisher('/action_topic', TwistStamped, queue_size=10)

    # delta_prev = None

    rate = rospy.Rate(10)  # 10 Hz
    time.sleep(1)
    while not rospy.is_shutdown():
        global current_pose
        obs = create_robot_obs_array(pose_listener)
        # print("obs: ", obs)
        action = get_action(model, obs)
        delta = list(action)

        # if delta_prev is None:
        #     delta = delta
        # else:
        #     delta = [(a + b) / 2 for a, b in zip(delta, delta_prev)]

        # Create a new PoseStamped message
        new_pose = PoseStamped()
        action = TwistStamped()

        action.header = Header(stamp=rospy.Time.now(), frame_id=current_pose.header.frame_id)
        action.twist.linear.x = delta[0]
        action.twist.linear.y =  delta[1]
        action.twist.linear.z =  delta[2]

        action.twist.angular.x = delta[3]
        action.twist.angular.y =  delta[4]
        action.twist.angular.z =  delta[5]
        # delta = [0, 1, 0, 0, 0, 0]

        # Incrementally update the pose with the delta
        new_pose.header = Header(stamp=rospy.Time.now(), frame_id=current_pose.header.frame_id)
        new_pose.pose.position.x = current_pose.pose.position.x + delta[0] * 0.01
        new_pose.pose.position.y = current_pose.pose.position.y + delta[1] * 0.01
        new_pose.pose.position.z = current_pose.pose.position.z + delta[2] * 0.01

        # Update the orientation based on the angular velocity
        quat = (
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
        )

        # Convert quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(quat)

        # Apply angular velocity to Euler angles
        new_euler = [
            euler[0] + delta[3] * .1,
            euler[1] + delta[4] * .1,
            euler[2] + delta[5] * .1,
        ]

        # Convert back to quaternion
        new_quat = tf.transformations.quaternion_from_euler(*new_euler)

        new_pose.pose.orientation.x = new_quat[0]
        new_pose.pose.orientation.y = new_quat[1]
        new_pose.pose.orientation.z = new_quat[2]
        new_pose.pose.orientation.w = new_quat[3]

        # Publish the new pose
        pose_pub.publish(new_pose)
        action_pun.publish(action)

        # delta_prev = delta

        rate.sleep()
        


if __name__ == '__main__':
    return_action()