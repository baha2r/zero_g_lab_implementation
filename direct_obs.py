#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import tf
import sys
from geometry_msgs.msg import Pose

def initialize_robot():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('observation', anonymous=True)
    robot = moveit_commander.RobotCommander()
    return robot

def initialize_move_group(group_name):
    return moveit_commander.MoveGroupCommander(group_name)

def transform_pose(tf_listener, pose, from_frame, to_frame):
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = from_frame
    pose_stamped.pose = pose

    try:
        tf_listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
        transformed_pose_stamped = tf_listener.transformPose(to_frame, pose_stamped)
        return transformed_pose_stamped.pose
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
        rospy.logerr(ex)
        return None

def get_ee_poses():
    robot = initialize_robot()
    tf_listener = tf.TransformListener()

    ceiling_arm_control = initialize_move_group("ceiling_arm")
    wall_arm_control = initialize_move_group("wall_arm")

    planning_frame = ceiling_arm_control.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)
    capture_frame = "capture_frame"

    rospy.sleep(2)  # Ensure everything is set up

    ceiling_arm_current_pose = ceiling_arm_control.get_current_pose().pose
    ceiling_arm_transformed_pose = transform_pose(tf_listener, ceiling_arm_current_pose, planning_frame, capture_frame)
    wall_arm_current_pose = wall_arm_control.get_current_pose().pose
    wall_arm_transformed_pose = transform_pose(tf_listener, wall_arm_current_pose, planning_frame, capture_frame)

    return {
        "ceiling_arm_current_pose": ceiling_arm_current_pose,
        "ceiling_arm_transformed_pose": ceiling_arm_transformed_pose,
        "wall_arm_current_pose": wall_arm_current_pose,
        "wall_arm_transformed_pose": wall_arm_transformed_pose
    }

if __name__ == '__main__':
    try:
        poses = get_ee_poses()
        rospy.loginfo(poses)
    except rospy.ROSInterruptException:
        pass
