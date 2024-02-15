#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import time

def publish_pose():
    # Initialize the ROS node
    rospy.init_node('pose_publisher', anonymous=True)

    # Create a publisher for the PoseStamped message
    pose_publisher = rospy.Publisher('/vrpn_client_node/TargetRobetarme/pose_transform', PoseStamped, queue_size=10)

    # Set the publishing rate (adjust as needed)
    rate = rospy.Rate(10)  # 1 Hz

    # Create a PoseStamped message
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "base"
    pose_msg.pose.position.x = -0.061671819088715774
    pose_msg.pose.position.y = -1.2231335817673443
    pose_msg.pose.position.z = 0.6230377347931229
    pose_msg.pose.orientation.x = 0.7107592148598265
    pose_msg.pose.orientation.y = -0.010362802471094404
    pose_msg.pose.orientation.z = 0.01012737717438275
    pose_msg.pose.orientation.w = 0.7032861345486314

    # Start publishing
    while not rospy.is_shutdown():
        # Update timestamp and sequence number
        pose_msg.header.seq += 1
        pose_msg.header.stamp = rospy.Time.now()

        # Publish the PoseStamped message
        pose_publisher.publish(pose_msg)

        # Sleep to control the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
