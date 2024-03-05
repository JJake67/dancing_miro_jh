#!/usr/bin/env python3

import rospy
from msgs import body

rospy.init_node("body_topic")
my_pub = rospy.Publisher("/body_topic", body, queue_size=10)

body_msg = body()
body_msg.mode=0
body_msg.move_name="Robot"

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    my_pub.publish(body_msg)
    rate.sleep()
