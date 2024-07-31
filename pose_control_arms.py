#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
import tf
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def initialize_robot():
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    return robot, scene

def initialize_move_group(group_name):
    # rospy.slee
    return moveit_commander.MoveGroupCommander(group_name)

def print_info(wall_arm_control, ceiling_arm_control):

    # We can get the name of the reference frame for this robot:
    wall_arm_planning_frame = wall_arm_control.get_planning_frame()
    ceiling_arm_planning_frame = ceiling_arm_control.get_planning_frame()
    print("============ wall_arm Reference frame: %s" % wall_arm_planning_frame)

    # We can also print the name of the end-effector link for this group:
    wall_arm_ee_link = wall_arm_control.get_end_effector_link()
    ceiling_arm_ee_link = ceiling_arm_control.get_end_effector_link()
    print("============ wall_arm End effector: %s" % wall_arm_ee_link)

    wall_arm_current_joints = wall_arm_control.get_current_joint_values()
    ceiling_arm_current_joints = ceiling_arm_control.get_current_joint_values()
    print("============ ceiling arm Joint values: ", ceiling_arm_current_joints)

    # Get the current pose of the end-effector
    wall_arm_current_pose = wall_arm_control.get_current_pose().pose
    ceiling_arm_current_pose = ceiling_arm_control.get_current_pose().pose
    print("============ wall_arm end-effector pose: %s" % wall_arm_current_pose)

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

def apply_action(current_pose, action):
    new_pose = geometry_msgs.msg.Pose()
    euler = tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, 
                                                      current_pose.orientation.z, current_pose.orientation.w])
    new_pose.position.x = current_pose.position.x + action[0] * 0.01
    new_pose.position.y = current_pose.position.y + action[1] * 0.01
    new_pose.position.z = current_pose.position.z + action[2] * 0.01
    new_quaternion = tf.transformations.quaternion_from_euler(euler[0] + action[3], euler[1] + action[4], euler[2] + action[5])
    new_pose.orientation.x = new_quaternion[0] * 0.1
    new_pose.orientation.y = new_quaternion[1] * 0.1
    new_pose.orientation.z = new_quaternion[2] * 0.1
    new_pose.orientation.w = new_quaternion[3] * 0.1
    return new_pose

def set_tolerance_and_planning_time(move_group):
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_planning_time(10)  # Increase planning time

def set_target_pose(move_group, target_pose):
    move_group.set_pose_target(target_pose)
    plan = move_group.go(wait=True)
    return plan

def main():
    
    robot, scene = initialize_robot()
    rospy.sleep(1.0)

    tf_listener = tf.TransformListener()
    # group_names = ["ceiling_arm", "wall_arm", "ceiling_track", "wall_track"]

    # Select the arm to control
    ceiling_arm_control = initialize_move_group("ceiling_arm")
    wall_arm_control = initialize_move_group("wall_arm")
    # Set the tolerance for planning
    set_tolerance_and_planning_time(ceiling_arm_control)
    set_tolerance_and_planning_time(wall_arm_control)

    wall_arm_planning_frame = wall_arm_control.get_planning_frame()
    ceiling_arm_planning_frame = ceiling_arm_control.get_planning_frame()
    wall_arm_ee_link = wall_arm_control.get_end_effector_link()
    ceiling_arm_ee_link = ceiling_arm_control.get_end_effector_link()
    wall_arm_current_joints = wall_arm_control.get_current_joint_values()
    ceiling_arm_current_joints = ceiling_arm_control.get_current_joint_values()
    wall_arm_current_pose = wall_arm_control.get_current_pose().pose
    ceiling_arm_current_pose = ceiling_arm_control.get_current_pose().pose

    print("============ ceiling arm current pose: %s" % ceiling_arm_current_pose)

    # Transform the current pose to the "capture_frame"
    capture_frame = "capture_frame"
    wall_arm_transformed_pose = transform_pose(tf_listener, wall_arm_current_pose, wall_arm_planning_frame, capture_frame)
    ceiling_arm_transformed_pose = transform_pose(tf_listener, ceiling_arm_current_pose, ceiling_arm_planning_frame, capture_frame)
    print("============ Ceiling arm Transformed pose in capture_frame: %s" % ceiling_arm_transformed_pose)

    # action = [random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), 
    #           random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)]
    # action = [random.uniform(-1, 1),0,0,0,0,0]
    action=  [0,0,0,0,0,0]
    wall_arm_target_transformed_pose = apply_action(wall_arm_transformed_pose, action)

    wall_arm_target_pose = transform_pose(tf_listener, wall_arm_target_transformed_pose, capture_frame, wall_arm_planning_frame)
    print("============ ceiling_arm target end-effector pose: %s" % wall_arm_target_pose)

    plan = set_target_pose(wall_arm_control, wall_arm_target_pose)

    if not plan:
        print("Planning failed, no valid plan found.")
    else:
        wall_arm_control.stop()
        wall_arm_control.clear_pose_targets()

        # Checking the final pose
        wall_arm_final_pose = wall_arm_control.get_current_pose().pose
        print("============ ceiling_arm final end-effector pose: %s" % wall_arm_final_pose)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
