#!/usr/bin/env python
import os
import time
import rospy            # ROS Python interface
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class JointPublisher(object):
    
    """
        The following code will move the joints, cosmetic and kinematic

    """
    def __init__(self):
        rospy.init_node("joint_publisher")
        self.position = None
        self.start = time.time()
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.kinematic_pub = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        self.cosmetic_pub = rospy.Publisher(
            topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

    # movement for either tilt, lift, yaw or pitch
    def set_move_kinematic(self, tilt = 0, lift = 0, yaw = 0, pitch = 0):
        joint_cmd = JointState()
        joint_cmd.position = [tilt, lift, yaw, pitch]
        self.kinematic_pub.publish(joint_cmd)

    # movement for the tail, eye lid, ears
    def set_move_cosmetic(self, tail_pitch = 0, tail_yaw = 0, left_eye = 0, right_eye = 0, left_ear = 0, right_ear = 0):
        joint_cmd = Float32MultiArray()
        joint_cmd.data = [tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear]
        self.cosmetic_pub.publish(joint_cmd)

    # a sample on how the miro can blink
    def blink_sample(self):
        current_time = time.time()
        set_angle = np.sin(current_time - self.start)
        self.set_move_cosmetic(left_eye = set_angle, right_eye = set_angle)
        rospy.sleep(0.05)
   
    # ear movemnt sample
    def ear_sample(self):
        current_time = time.time()
        set_angle = np.sin(current_time - self.start)
        self.set_move_cosmetic(left_ear = set_angle, right_ear= set_angle)
        rospy.sleep(0.1)
   
    #tail
    def tail_sample(self):
        current_time = time.time()
        set_angle = np.sin(current_time - self.start)
        self.set_move_cosmetic(tail_pitch = set_angle, tail_yaw = set_angle)
        rospy.sleep(0.05)
    
    #head movements/ up and down
    def head_sample(self):
        current_time = time.time()
        set_angle = np.sin(current_time - self.start)
        self.set_move_kinematic(tilt = set_angle, lift=set_angle)
        rospy.sleep(0.05)

    #head movements / left and right 
    def rotate_sample(self):
        current_time = time.time()
        set_angle = np.sin(current_time - self.start)
        self.set_move_kinematic(yaw = set_angle, pitch= set_angle)
        rospy.sleep(0.05)

movement = JointPublisher()
while not rospy.is_shutdown():
    movement.blink_sample()
    #movement.ear_sample()
    #movement.tail_sample()
    #movement.head_sample()
    movement.rotate_sample()