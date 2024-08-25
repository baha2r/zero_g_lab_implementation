#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def publish_pose():
    pub_c = rospy.Publisher('/capture_c_a_tool1/pose', PoseStamped, queue_size=10)
    pub_w = rospy.Publisher('/capture_ot_offset/pose', PoseStamped, queue_size=10)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    i=1

    while not rospy.is_shutdown():
        msg_c = PoseStamped()
        msg_c.header.stamp = rospy.Time.now()
        msg_c.header.frame_id = 'world'
        msg_c.pose.position.x = i
        msg_c.pose.position.y = -i
        msg_c.pose.position.z = 1.0
        msg_c.pose.orientation.x = 0.0
        msg_c.pose.orientation.y = 0.0
        msg_c.pose.orientation.z = 0.0
        msg_c.pose.orientation.w = 1.0

        msg_w = PoseStamped()
        msg_w.header.stamp = rospy.Time.now()
        msg_w.header.frame_id = 'world'
        msg_w.pose.position.x = i
        msg_w.pose.position.y = -i
        msg_w.pose.position.z = 6.0
        msg_w.pose.orientation.x = 0.0
        msg_w.pose.orientation.y = 0.0
        msg_w.pose.orientation.z = 0.0
        msg_w.pose.orientation.w = 1.0

        pub_c.publish(msg_c)
        pub_w.publish(msg_w)
        rate.sleep()
        i = i + 1

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
