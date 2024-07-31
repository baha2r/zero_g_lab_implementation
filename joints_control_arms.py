#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

def publish_joint_positions_arm(arm_name, joint_positions):
    pub = rospy.Publisher('/{}_pos_joint_traj_controller/command'.format(arm_name), JointTrajectory, queue_size=10)
    rospy.init_node('control_arms', anonymous=True)
    rospy.sleep(1)

    traj = JointTrajectory()
    traj.joint_names = [
        '{}_shoulder_pan_joint'.format(arm_name),
        '{}_shoulder_lift_joint'.format(arm_name),
        '{}_elbow_joint'.format(arm_name),
        '{}_wrist_1_joint'.format(arm_name),
        '{}_wrist_2_joint'.format(arm_name),
        '{}_wrist_3_joint'.format(arm_name),
        # 'wall_track_slider_joint',
    ]
    
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(1.0)  # 1 second to reach the target
    traj.points.append(point)
    
    rospy.loginfo("Publishing joint positions to {}".format(arm_name))
    rospy.loginfo(traj)
    pub.publish(traj)
    rospy.sleep(3)  # Wait a bit for the message to be sent

def publish_track_slider(arm_name, joint_position):
    rospy.init_node('control_arms', anonymous=True)
    rospy.sleep(1)
    pub = rospy.Publisher('/{}_track_slider_position_trajectory_controller/command'.format(arm_name), JointTrajectory, queue_size=10)
    rospy.sleep(1)

    traj = JointTrajectory()
    traj.joint_names = [
        '{}_track_slider_joint'.format(arm_name),
    ]

    point = JointTrajectoryPoint()
    point.positions = [joint_position]
    point.time_from_start = rospy.Duration(1.0)  # 1 second to reach the target
    traj.points.append(point)

    rospy.loginfo("Publishing track slider position to {}".format(arm_name))
    rospy.loginfo(traj)
    pub.publish(traj)
    rospy.sleep(3)  # Wait a bit for the message to be sent

if __name__ == '__main__':
    # Default joint positions
    # -J ceiling_arm_elbow_joint -1.57
    # -J ceiling_arm_shoulder_lift_joint -1.57
    # -J ceiling_arm_shoulder_pan_joint 0.0 
    # -J ceiling_arm_wrist_1_joint -1.57
    # -J ceiling_arm_wrist_2_joint -1.57 
    # -J ceiling_arm_wrist_3_joint 1.57
    # -J ceiling_track_slider_joint 0.0
    # -J wall_arm_elbow_joint 0.0 
    # -J wall_arm_shoulder_lift_joint 0.0 
    # -J wall_arm_shoulder_pan_joint 0.0 
    # -J wall_arm_wrist_1_joint -1.57 
    # -J wall_arm_wrist_2_joint 1.57 
    # -J wall_arm_wrist_3_joint 0.0
    # -J wall_track_slider_joint 0.0
    wall_arm_positions    = [4.7449846267700195, -1.9148956737914027, -1.3253941535949707, -1.4894262117198487, -1.5693534056292933, 1.014766995106832326]
    ceiling_arm_positions = [-1.57, -1.57, 0.0, -1.57, 1.57, 1.57]

    wall_track_slider_position = 1.1
    ceiling_track_slider_position = 0.6
    
    # Publish to arm joints
    # publish_joint_positions_arm('wall_arm', wall_arm_positions)
    # publish_joint_positions_arm('ceiling_arm', ceiling_arm_positions)

    # Publish to track slider
    publish_track_slider('wall', wall_track_slider_position)
    # publish_track_slider('ceiling', ceiling_track_slider_position)
