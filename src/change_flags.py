#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt32
import os 
import miro2 as miro


topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        
topic = topic_base_name + "/control/flags"
pub_flags =  rospy.Publisher(topic,UInt32, queue_size=0)

msg = UInt32()
msg.data = 0 
msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX
pub_flags.publish(msg.data)
print("Ok!")
