#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Bool

rospy.init_node("record_topic")
my_pub = rospy.Publisher("/record_topic", bool, queue_size=10)

bool_msg = False

rate = rospy.Rate(1)

while not rospy_is_shutdown():
    my_pub.publish(bool_msg)
    rate.sleep()