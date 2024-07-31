#!/usr/bin/env python3

import sys
import os
ros_master_uri = "http://172.19.0.2:11311"
import rospy
import moveit_commander
import geometry_msgs.msg
from rosgraph_msgs.msg import Clock
from moveit_commander.conversions import pose_to_list

def initialize_robot():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    return robot, scene

def initialize_move_group(group_name):
    return moveit_commander.MoveGroupCommander(group_name)

def get_current_time():
    clock_msg = rospy.wait_for_message("/clock", Clock)
    return clock_msg.clock

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
    print("============ %s End effector: %s" % (arm_to_control, eef_link))

    # Print current joint values and pose
    current_joint_values = move_group.get_current_joint_values()
    print("============ %s Current Joint Values: " % arm_to_control, current_joint_values)
    current_pose = move_group.get_current_pose().pose
    print("============ %s Current Pose: " % arm_to_control, current_pose)

    # Define a constant velocity for each direction
    velocity = {'x': 0.01, 'y': 0.01, 'z': 0.01}
    time_step = 1.0

    # Get the start time
    current_time = get_current_time()
    start_time = current_time.secs + current_time.nsecs / 1e9

    while not rospy.is_shutdown():
        # Get the current time
        current_time = get_current_time()
        elapsed_time = current_time.secs + current_time.nsecs / 1e9 - start_time

        # Calculate the new pose based on the constant velocity and elapsed time
        pose_target = calculate_new_pose(current_pose, velocity, elapsed_time)

        # Move to the new pose
        plan = move_to_pose(move_group, pose_target)

        if not plan:
            print("Planning failed, no valid plan found.")
        else:
            current_pose = move_group.get_current_pose().pose
            print("============ %s final end-effector pose: %s" % (arm_to_control, current_pose))

        # Sleep for the time step duration
        rospy.sleep(time_step)

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
