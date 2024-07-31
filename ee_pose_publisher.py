#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri

import rospy
import moveit_commander
import geometry_msgs.msg
import tf
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

def initialize_robot():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('observation', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    return robot, scene

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

def ee_pose_publisher():
    rospy.init_node('observation', anonymous=True)
    robot, scene = initialize_robot()
    tf_listener = tf.TransformListener()

    ceiling_arm_control = initialize_move_group("ceiling_arm")
    wall_arm_control = initialize_move_group("wall_arm")

    planning_frame = ceiling_arm_control.get_planning_frame()
    capture_frame = "capture_frame"

    ceiling_arm_ee_pose_pub = rospy.Publisher('ceiling_arm_ee_pose', Pose, queue_size=10)
    ceiling_arm_ee_pose_capture_pub = rospy.Publisher('ceiling_arm_ee_pose_capture', Pose, queue_size=10)
    wall_arm_ee_pose_pub = rospy.Publisher('wall_arm_ee_pose', Pose, queue_size=10)
    wall_arm_ee_pose_capture_pub = rospy.Publisher('wall_arm_ee_pose_capture', Pose, queue_size=10)

    rospy.sleep(2)  # Wait for a moment to ensure everything is set up

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        ceiling_arm_current_pose = ceiling_arm_control.get_current_pose().pose
        ceiling_arm_transformed_pose = transform_pose(tf_listener, ceiling_arm_current_pose, planning_frame, capture_frame)
        wall_arm_current_pose = wall_arm_control.get_current_pose().pose
        wall_arm_transformed_pose = transform_pose(tf_listener, wall_arm_current_pose, planning_frame, capture_frame)

        if ceiling_arm_transformed_pose:
            # rospy.loginfo("Publishing ee_pose and ee_pose_capture")
            ceiling_arm_ee_pose_pub.publish(ceiling_arm_current_pose)
            ceiling_arm_ee_pose_capture_pub.publish(ceiling_arm_transformed_pose)
            wall_arm_ee_pose_pub.publish(wall_arm_current_pose)
            wall_arm_ee_pose_capture_pub.publish(wall_arm_transformed_pose)

        rate.sleep()

if __name__ == '__main__':
    try:
        ee_pose_publisher()
    except rospy.ROSInterruptException:
        pass
