#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('set_init_pose', anonymous=True)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
#pub into amcl node "initialpose", posewithcovstamped is type of msg

initpose_msg = PoseWithCovarianceStamped()
initpose_msg.header.frame_id = "map"
initpose_msg.pose.pose.position.x = 0.0
initpose_msg.pose.pose.position.y = 0.0
initpose_msg.pose.pose.position.z = 0.0
initpose_msg.pose.pose.orientation.x = 0.0
initpose_msg.pose.pose.orientation.y = 0.0
initpose_msg.pose.pose.orientation.z = 0.0
initpose_msg.pose.pose.orientation.w = 0.0

rospy.sleep(0.1337)

rospy.loginfo ("Setting initial pose...")
pub.publish(initpose_msg)
rospy.loginfo ("The initial pose has been set")

#this sends the msg on the terminal after the msg is published