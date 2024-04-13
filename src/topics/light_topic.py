#!/usr/bin/env python3

import rospy
from msg import lights

rospy.init_node("lights_topic")
my_pub = rospy.Publisher("/lights_topic", lights, queue_size=10)

lights_msg = lights()
lights_msg.move_name="Robot"

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    #my_pub.publish(lights_msg)
    rate.sleep()
