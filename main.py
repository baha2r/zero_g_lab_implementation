#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri
import time
from pose_listener import PoseListener  # Assuming the PoseListener class is saved in a file named pose_listener.py
import rospy

def main():
    # Instantiate the PoseListener
    pose_listener = PoseListener()

    # Start the listeners in separate threads
    pose_listener.start_listening()

    # Give the threads a moment to start
    time.sleep(1)

    # Loop to periodically fetch and print the latest data
    try:
        while not rospy.is_shutdown():
            # Fetch the latest data for robot c_a_tool0
            latest_c_a_tool0_info = pose_listener.get_latest_c_a_tool0_info()
            print("Latest Data for /capture_c_a_tool0/pose:")
            print("Position:", latest_c_a_tool0_info['position'])
            print("Orientation (Euler):", latest_c_a_tool0_info['orientation_euler'])
            print("Linear Velocity:", latest_c_a_tool0_info['linear_velocity'])
            print("Angular Velocity:", latest_c_a_tool0_info['angular_velocity'])
            print("")

            # Fetch the latest data for robot w_a_tool0
            latest_w_a_tool0_info = pose_listener.get_latest_w_a_tool0_info()
            print("Latest Data for /capture_w_a_tool0/pose:")
            print("Position:", latest_w_a_tool0_info['position'])
            print("Orientation (Euler):", latest_w_a_tool0_info['orientation_euler'])
            print("Linear Velocity:", latest_w_a_tool0_info['linear_velocity'])
            print("Angular Velocity:", latest_w_a_tool0_info['angular_velocity'])
            print("")

            # Wait for a short period before fetching again
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Shutting down.")

if __name__ == '__main__':
    main()
