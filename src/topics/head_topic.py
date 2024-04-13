#!/usr/bin/env python3

import rospy
from msg import head

rospy.init_node("head_topic")
my_pub = rospy.Publisher("/head_topic", head, queue_size=10)

head_msg = head()
head_msg.mode=0
head_msg.move_head="Robot"

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    #my_pub.publish(head_msg)
    rate.sleep()
