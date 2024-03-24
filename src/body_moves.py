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
from diss.msg import body
#import miro2 as miro  # MiRo Developer Kit library
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
    #ACTION_DURATION = rospy.Duration(16.0)  # seconds

    def __init__(self):
        self.command = ""
        self.moveLength = 0.0

        # Get robot name
        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Initialise a new ROS node to communicate with MiRo
        rospy.init_node("body_moves", anonymous=True)

        # Define Publishers and Subscribers
        self.pub_cmd_vel = rospy.Publisher(
            topic_root + "/control/cmd_vel", TwistStamped, queue_size=10
        )
        self.velocity = TwistStamped()

        self.sub = rospy.Subscriber("body_topic", body, self.cmd_callback)
        rospy.loginfo("Body Move node is active...")

    # Callback for when parameters are passed from Miro_Dance Node 
    def cmd_callback(self,topic_message):
        #print(f'Node obtained msg: {topic_message.move_name}')
        #print(f'Node also said: {topic_message.mode}')
        self.moveLength = (60 / topic_message.tempo ) * 8 
        self.command = topic_message.move_name


    def rotate(self):
        #print("MiRO Rotating")
        t0 = rospy.Time.now()
        self.velocity.twist.linear.x = 0.05
        self.velocity.twist.angular.z = 3/math.pi
        self.pub_cmd_vel.publish(self.velocity)
        rospy.sleep(0.1)

    def rotate_m(self,spin_length):
        t0 = rospy.Time.now().to_sec()
        tFinal = t0 + spin_length
        print("MiRo rotating") 
        while rospy.Time.now().to_sec() < tFinal:
            self.velocity.twist.linear.x = 0
            self.velocity.twist.angular.z = 5/math.pi
            self.pub_cmd_vel.publish(self.velocity)
            #check if 5 seconds have passed
            if rospy.Time().now().to_sec() - t0 >= 5:
                #reset the timer
                t0 = rospy.Time.now().to_sec()
                #reverse the direction of rotation
                self.velocity.twist.angular.z = -5/math.pi
                self.pub_cmd_vel.publish(self.velocity)
        #stop the rotation
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)


    # Not dead on, need to ask Alex about that 
    def full_spin(self,spin_length):
        t0 = rospy.Time.now().to_sec()
        tFinal = t0 + spin_length
        
        # Closest to accurate 
        ang_vel = (2 / spin_length) * (math.pi)

        self.velocity.twist.linear.x = 0    # m/s

        while t0 < tFinal:
            # Set linear and angular velocities for spin
            self.velocity.twist.angular.z = ang_vel
            self.pub_cmd_vel.publish(self.velocity)
            rospy.sleep(0.02)
            t0 = rospy.Time.now().to_sec()

        # Stops MiRo when spin is done 
        rospy.sleep(0.02)
        self.velocity.twist.angular.z = ang_vel
        self.pub_cmd_vel.publish(self.velocity)
        rospy.sleep(self.moveLength)

    # RIGHT NOW ACTS LIKE MORE OF A WINDING PATH
    def half_spin(self,spin_length):
        t0 = rospy.Time.now().to_sec()
        tFinal = t0 + spin_length
        
        # Closest to accurate 
        ang_vel = (2 / spin_length) * (math.pi)

        self.velocity.twist.linear.x = 0    # m/s
        tHalf = t0 + (spin_length / 2)
        while t0 < tFinal:
            # Set linear and angular velocities for spin
            self.velocity.twist.angular.z = ang_vel
            self.velocity.twist.linear.x = -0.1
    
            if t0 > tHalf:
                self.velocity.twist.angular.z = -ang_vel

            self.pub_cmd_vel.publish(self.velocity)
            rospy.sleep(0.02)
            t0 = rospy.Time.now().to_sec()

        rospy.sleep(0.02)
        self.velocity.twist.angular.z = ang_vel
        self.pub_cmd_vel.publish(self.velocity)
        rospy.sleep(5)

    def small_rotate_and_back(self,spin_length):
        t0 = rospy.Time.now().to_sec()
        tFinal = t0 + spin_length
        tHalf = t0 + (spin_length/2)
        ang_vel = (2 / spin_length) * (math.pi/4)

        while t0 < tFinal:
            self.velocity.twist.angular.z = ang_vel
            self.velocity.twist.linear.x = -0.1
                
            if t0 > tHalf:
                self.velocity.twist.angular.z = -ang_vel

            self.pub_cmd_vel.publish(self.velocity)


            rospy.sleep(0.02)
            t0 = rospy.Time.now().to_sec()

        # Move Done 
        rospy.sleep(0.02)
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)

    def wait(self):
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)
        rospy.sleep(0.5)   
                                                                                    
    def loop(self):
        if self.moveLength != 0.0:
            rospy.sleep(0.02)
            #print(self.moveLength)
            self.full_spin(self.moveLength)
            rospy.sleep(self.moveLength)
        #self.small_rotate_and_back(4)
        #rospy.sleep(1)
        #if self.command == "full_spin":
        #    movement.half_spin(10)
        #    self.command = "not_spin"
        #else:
        #    movement.wait()

movement = BodyMoves()
while not rospy.is_shutdown():
    #movement.rotate_m()
    movement.loop()
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

        ## MAIN FuNCTION THAT MAKES MiRo SPIN 
    #  Function spins him at a set speed of 5 / pi for 3 seconds then stops him 
    def rotate(self):
        #print("MiRO Rotating")
        t0 = rospy.Time.now()
        self.velocity.twist.linear.x = 0.05
        self.velocity.twist.angular.z = 3/math.pi
        self.pub_cmd_vel.publish(self.velocity)
        rospy.sleep(0.1)

    ## WORKS , USUALLY SOME DELAY BETWEEN PUBLISHIGN AND NEXT COMMAND SO rospy.sleep(0.1) allows publisher to get through
    ## idk why but it does
    def rotate_m2(self): 
        t0 = rospy.Time.now()
        print("MiRo rotating")
        self.velocity.twist.angular.z = 2/math.pi 
        self.velocity.twist.linear.x = 0
        self.pub_cmd_vel.publish(self.velocity)
        rospy.sleep(0.1)
        
        # EVEN IF CTRL C WILL KEEP PRINTING INFO UNTIL DONE 
        while rospy.Time.now() < (t0 + self.ACTION_DURATION):
            #print((rospy.Time.now()-t0).to_sec())
            if (rospy.Time.now() - t0).to_sec() >= 6.0:
                t0 = rospy.Time.now()
                self.velocity.twist.angular.z = self.velocity.twist.angular.z * -1
                self.pub_cmd_vel.publish(self.velocity)
            self.pub_cmd_vel.publish(self.velocity)
            rospy.sleep(0.1)
            #print(self.velocity.twist.angular.z)
            
        self.velocity.twist.linear.x = 0
        self.velocity.twist.angular.z = 0
        self.pub_cmd_vel.publish(self.velocity)
        print("done")
    

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