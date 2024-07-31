#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
from stable_baselines3 import SAC
from robotiqGymEnv import robotiqGymEnv
import pybullet as p
from Observation import get_ee_poses

p.connect(p.DIRECT)

# Set ROS master URI
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri

def create_env():
    """Create the gym environment for the Robotiq arm."""
    env = robotiqGymEnv()
    return env

def load_model():
    """Load the pre-trained SAC model."""
    model = SAC.load("/root/zerog/zero_g_lab_implementation/models/trained_agent/best_model.zip")
    return model

def get_action(model, obs):
    """Predict the next action based on the observation using the SAC model."""
    action, _states = model.predict(obs, deterministic=True)
    return action

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

def get_observation():
    """Get the current observation from get_ee_poses."""
    poses = get_ee_poses()
    wall_arm_pose = (poses["wall_arm_current_pose"].position, poses["wall_arm_current_pose"].orientation)
    ceiling_arm_pose = (poses["ceiling_arm_current_pose"].position, poses["ceiling_arm_current_pose"].orientation)
    vel_wall = (np.zeros(3), np.zeros(3))  # Placeholder for actual velocity calculation
    vel_ceiling = (np.zeros(3), np.zeros(3))  # Placeholder for actual velocity calculation

    obs = np.zeros(39)
    obs[:3] = [wall_arm_pose[0].x, wall_arm_pose[0].y, wall_arm_pose[0].z]
    obs[3:6] = [wall_arm_pose[1].x, wall_arm_pose[1].y, wall_arm_pose[1].z]
    obs[6:9] = vel_wall[0]
    obs[9:12] = vel_wall[1]
    obs[12:15] = [ceiling_arm_pose[0].x, ceiling_arm_pose[0].y, ceiling_arm_pose[0].z]
    obs[15:18] = [ceiling_arm_pose[1].x, ceiling_arm_pose[1].y, ceiling_arm_pose[1].z]
    rel_pose = np.array([ceiling_arm_pose[0].x, ceiling_arm_pose[0].y, ceiling_arm_pose[0].z]) - \
               np.array([wall_arm_pose[0].x, wall_arm_pose[0].y, wall_arm_pose[0].z])
    obs[18:21] = rel_pose
    rel_orientation = np.array([ceiling_arm_pose[1].x, ceiling_arm_pose[1].y, ceiling_arm_pose[1].z]) - \
                      np.array([wall_arm_pose[1].x, wall_arm_pose[1].y, wall_arm_pose[1].z])
    obs[21:24] = rel_orientation
    obs[24:27] = vel_ceiling[0]
    obs[27:30] = vel_ceiling[1]
    rel_vel = np.array(vel_ceiling[0]) - np.array(vel_wall[0])
    obs[30:33] = rel_vel
    rel_ang_vel = np.array(vel_ceiling[1]) - np.array(vel_wall[1])
    obs[33:36] = rel_ang_vel
    min_dist = min_distance_between_cuboids((obs[:3], (0.05, 0.0518, 0.06)),
                                            (obs[12:15], (0.1, 0.1, 0.02)))
    obs[36:39] = min_dist
    return obs

class ActionCompletionListener:
    def __init__(self):
        self.action_completed = False
        rospy.Subscriber('/action_completion', Bool, self.callback)

    def callback(self, msg):
        self.action_completed = msg.data

    def wait_for_completion(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.action_completed:
                self.action_completed = False
                return
            rate.sleep()

def publish_action(action):
    pub = rospy.Publisher('/robotiq_action', Float64MultiArray, queue_size=10)
    action_msg = Float64MultiArray(data=action)
    rospy.sleep(0.5)  # Give some time for the publisher to establish the connection
    pub.publish(action_msg)

def main():
    rospy.init_node('robotiq_controller', anonymous=True)
    model = load_model()
    rate = rospy.Rate(10)
    action_completion_listener = ActionCompletionListener()
    
    while not rospy.is_shutdown():
        obs = get_observation()
        action = get_action(model, obs)
        action[3:6] = np.zeros(3)
        rospy.loginfo(f"Action: {action}")
        rospy.loginfo(f"Observation: {obs}")
        publish_action(action)
        action_completion_listener.wait_for_completion()
        rate.sleep()

if __name__ == '__main__':
    main()
