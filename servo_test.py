#!/usr/bin/env python

import sys
import os
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from moveit_commander.conversions import pose_to_list

def initialize_robot():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    return robot, scene

def initialize_move_group(group_name):
    return moveit_commander.MoveGroupCommander(group_name)

def calculate_new_pose(current_pose, velocity, elapsed_time):
    new_pose = geometry_msgs.msg.Pose()
    new_pose.position.x = current_pose.position.x + velocity['x'] * elapsed_time
    new_pose.position.y = current_pose.position.y + velocity['y'] * elapsed_time
    new_pose.position.z = current_pose.position.z + velocity['z'] * elapsed_time
    new_pose.orientation = current_pose.orientation
    return new_pose

def move_to_pose(move_group, pose_target):
    move_group.set_pose_target(pose_target)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    return plan

def main():
    robot, scene = initialize_robot()
    group_names = ["ceiling_arm", "wall_arm", "ceiling_track", "wall_track"]
    move_groups = {name: initialize_move_group(name) for name in group_names}

    # Select the arm to control
    arm_to_control = "wall_arm"
    move_group = move_groups[arm_to_control]

    # Get the reference frame and end-effector link
    planning_frame = move_group.get_planning_frame()
    print("============ %s Reference frame: %s" % (arm_to_control, planning_frame))
    eef_link = move_group.get_end_effector_link()
    print("============ %s End effector: %s" % arm_to_control, eef_link)

    # Print current joint values and pose
    current_joint_values = move_group.get_current_joint_values()
    print("============ %s Current Joint Values: " % arm_to_control, current_joint_values)
    current_pose = move_group.get_current_pose().pose
    print("============ %s Current Pose: " % arm_to_control, current_pose)

    # Define a constant velocity for each direction
    velocity = {'x': 0.01, 'y': 0.0, 'z': 0.0}
    time_step = 1.0

    # Initialize the publisher for the /servo_server/delta_twist_cmds topic
    velocity_publisher = rospy.Publisher('/servo_server/delta_twist_cmds', Twist, queue_size=10)

    while not rospy.is_shutdown():
        # Create a Twist message with the velocity
        twist_msg = Twist()
        twist_msg.linear.x = velocity['x']
        twist_msg.linear.y = velocity['y']
        twist_msg.linear.z = velocity['z']
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        # Publish the velocity to the /servo_server/delta_twist_cmds topic
        velocity_publisher.publish(twist_msg)

        # Sleep for the time step duration
        rospy.sleep(time_step)

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
