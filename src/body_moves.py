#!/usr/bin/env python3
"""
Simple action selection mechanism inspired by the K-bandit problem

Initially, MiRo performs one of the following actions on random, namely: 
wiggle ears, wag tail, rotate, turn on LEDs and simulate a Braitenberg Vehicle.

While an action is being executed, stroking MiRo's head will reinforce it, while  
stroking MiRo's body will inhibit it, by increasing or reducing the probability 
of this action being picked in the future.

NOTE: The code was tested for Python 2 and 3
For Python 2 the shebang line is
#!/usr/bin/env python
"""

# Imports
##########################
import math
import os
import numpy as np
import time
import rospy  # ROS Python interface
from std_msgs.msg import (
    Float32MultiArray,
    UInt32MultiArray,
    UInt16,
)  # Used in callbacks
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control)

import miro2 as miro  # MiRo Developer Kit library
"""""
try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2
##########################
"""


class BodyMoves(object):

    # Script settings below
    TICK = 0.02  # Main loop frequency (in secs, default is 50Hz)
    ACTION_DURATION = rospy.Duration(16.0)  # seconds
    VERBOSE = True  # Whether to print out values of Q and N after each iteration
    ##NOTE The following option is relevant in MiRoCODE
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node

    def __init__(self):
        """
        Class initialisation
        """
        print("Initialising the controller...")

        # Get robot name
        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("body_moves", anonymous=True)

        # Define ROS publishers
        self.pub_cmd_vel = rospy.Publisher(
            topic_root + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        #self.rotate = TwistStamped()
        self.velocity = TwistStamped()

    ## MAIN FuNCTION THAT MAKES MiRo SPIN 
    #  Function spins him at a set speed of 5 / pi for 3 seconds then stops him 
    def rotate(self):
        print("MiRO Rotating")
        t0 = rospy.Time.now()
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.velocity.twist.linear.x = 0.1
            self.velocity.twist.angular.z = 5/math.pi
            self.pub_cmd_vel.publish(self.velocity)
            rospy.sleep(0.1)
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)
    # Raghads VERSION !!!!
    """"def rotate(self):
        t0 = rospy.Time.now()
        print("MiRo rotating") 
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.velocity.twist.linear.x = 0.1
            self.velocity.twist.angular.z = 5/math.pi
            self.pub_cmd_vel.publish(self.velocity)
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)
    
    def rotate_m(self):
        t0 = rospy.Time.now()
        print("MiRo rotating") 
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.velocity.twist.linear.x = 0
            self.velocity.twist.angular.z = 5/math.pi
            self.pub_cmd_vel.publish(self.velocity)
            #check if 5 seconds have passed
            if time.time() - t0.to_sec() >= 5:
                #reset the timer
                t0 = rospy.Time.now()
                #reverse the direction of rotation
                self.velocity.twist.angular.z = -5/math.pi
                self.pub_cmd_vel.publish(self.velocity)
        #stop the rotation
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)
    """
    """"
    def rotate_m(self):
        t0 = rospy.Time.now()
        print("MiRo rotating") 
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.velocity.twist.linear.x = 0
            self.velocity.twist.angular.z = 5/math.pi
            self.pub_cmd_vel.publish(self.velocity)
            #check if 5 seconds have passed
            if rospy.Time.now() > t0 + rospy.Duration(5):
                #reset the timer
                t0 = rospy.Time.now()
                #reverse the direction of rotation
                self.velocity.twist.angular.z *= -1
                self.pub_cmd_vel.publish(self.velocity)
        #stop the rotation
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)
    """
    ## THIS IS NEVER USED I DONT THINK 

    ## It is meant to rotate it in one direction for 5 secs, then rotate back and just repeat this sequence
    """
    def rotate_m2(self): 
        t0 = rospy.Time.now()
        print("MiRo rotating")
        while rospy.Time.now() < t0 + self.ACTION_DURATION:
            self.velocity.twist.linear.x = 0
            self.velocity.twist.angular.z = 2/math.pi 
            self.pub_cmd_vel.publish(self.velocity)
            print((rospy.Time.now() - t0).to_sec())
            if (rospy.Time.now() - t0).to_sec() >= 3.0:
                t0 = rospy.Time.now()
                self.velocity.twist.angular.z = -2/math.pi
                self.pub_cmd_vel.publish(self.velocity)
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)
        print("done")
    """
    ## WORKS , USUALLY SOME DELAY BETWEEN PUBLISHIGN AND NEXT COMMAND SO rospy.sleep(0.1) allows publisher to get through
    ## idk why but it does
    def rotate_m2(self): 
        t0 = rospy.Time.now()
        print("MiRo rotating")
        self.velocity.twist.angular.z = 2/math.pi 
        self.velocity.twist.linear.x = 0
        self.pub_cmd_vel.publish(self.velocity)
        rospy.sleep(0.1)
        while rospy.Time.now() < (t0 + self.ACTION_DURATION):
            print((rospy.Time.now()-t0).to_sec())
            if (rospy.Time.now() - t0).to_sec() >= 6.0:
                t0 = rospy.Time.now()
                self.velocity.twist.angular.z = self.velocity.twist.angular.z * -1
                self.pub_cmd_vel.publish(self.velocity)
            self.pub_cmd_vel.publish(self.velocity)
            rospy.sleep(0.1)
            print(self.velocity.twist.angular.z)
            
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)
        print("done")

movement = BodyMoves()
while not rospy.is_shutdown():
    ##movement.rotate_m2()
    ##print("ROTATION M2 DONE")
    ##rospy.sleep(1)
    movement.rotate()
