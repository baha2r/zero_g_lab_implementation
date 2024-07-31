#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri

import rospy
import tf
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_euler

def get_transform(listener, parent_frame, child_frame):
    try:
        # Lookup the transform at the latest available time
        (trans, rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        # Convert quaternion to rotation matrix
        rotation = quaternion_matrix(rot)[:3, :3]
        return trans, rotation
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        return None, None

def create_capture_frame(listener, broadcaster, parent_frame, ceiling_frame):
    # Get ceiling arm tool0 transformation
    ceiling_trans, ceiling_rot = get_transform(listener, parent_frame, ceiling_frame)
    if ceiling_trans is None or ceiling_rot is None:
        print("Failed to get transform for", ceiling_frame)
        return
    
    # The ceiling_arm_tool0 should be at [0, 0, 1] in capture_frame, so the translation should be [ceiling_trans[0], ceiling_trans[1], ceiling_trans[2] - 1]
    trans_to_capture = [ceiling_trans[0], ceiling_trans[1], ceiling_trans[2] - 1]
    
    # Broadcast the new capture_frame
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # Debugging information
        # print("Broadcasting capture_frame with translation:", trans_to_capture)
        
        broadcaster.sendTransform(trans_to_capture,
                                  quaternion_from_euler(0, 0, -np.pi),
                                  rospy.Time.now(),
                                  "capture_frame",
                                  parent_frame)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('tf_capture_frame_creator', anonymous=True)
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    # Wait for the listener to get the first message
    rospy.sleep(2.0)

    parent_frame = 'zerog_room_frame'
    ceiling_frame = 'ceiling_arm_tool0'

    create_capture_frame(listener, broadcaster, parent_frame, ceiling_frame)
