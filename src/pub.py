#!/usr/bin/env python3
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
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.kinematic_pub = rospy.Publisher(
            self.topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        self.cosmetic_pub = rospy.Publisher(
            self.topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )
        self.time_scale = 0.3
        self.kinematic_joint_cmd = JointState()
        self.kinematic_joint_cmd.position = [0, 0, 0, 0]
        self.cosmetic_joint_cmd = Float32MultiArray()   
        self.cosmetic_joint_cmd.data = [0,0,0,0,0,0]   

    # movement for either tilt, lift, yaw or pitch
    def set_move_kinematic(self, tilt = 0, lift = 0, yaw = 0, pitch = 0):
        self.kinematic_joint_cmd.position = [tilt, lift, yaw, pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    # movement for the tail, eye lid, ears
    def set_move_cosmetic(self, tail_pitch = 0, tail_yaw = 0, left_eye = 0, right_eye = 0, left_ear = 0, right_ear = 0):
        self.cosmetic_joint_cmd.data = [tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    # a sample on how the miro can blink
    def blink_sample(self):
        current_time = time.time()
        set_angle = np.sin(self.time_scale*(current_time - self.start))
        tail_pitch = self.cosmetic_joint_cmd.data[0]
        tail_yaw = self.cosmetic_joint_cmd.data[1]
        left_eye = set_angle
        right_eye = set_angle
        left_ear = self.cosmetic_joint_cmd.data[4]
        right_ear = self.cosmetic_joint_cmd.data[5]
        self.set_move_cosmetic(tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear)
        rospy.sleep(0.05)
   
    # ear movemnt sample
    def ear_sample(self):
        current_time = time.time()
        set_angle = np.sin(self.time_scale*(current_time - self.start))
        tail_pitch = self.cosmetic_joint_cmd.data[0]
        tail_yaw = self.cosmetic_joint_cmd.data[1]
        left_eye = self.cosmetic_joint_cmd.data[2]
        right_eye = self.cosmetic_joint_cmd.data[3]
        left_ear = set_angle
        right_ear = set_angle
        self.set_move_cosmetic(tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear)
        rospy.sleep(0.05)
   
    #tail
    def tail_sample(self):
        current_time = time.time()
        set_angle = np.sin(self.time_scale*(current_time - self.start))
        left_eye = self.cosmetic_joint_cmd.data[0]
        right_eye = self.cosmetic_joint_cmd.data[1]
        tail_pitch = set_angle
        tail_yaw = set_angle
        left_ear = self.cosmetic_joint_cmd.data[2]
        right_ear = self.cosmetic_joint_cmd.data[3]
        self.set_move_cosmetic(tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear)
        rospy.sleep(0.05)
    
    #head movements/ up and down
    def head_sample(self):
        current_time = time.time()
        set_angle = np.sin(self.time_scale*(current_time - self.start))
        tilt = set_angle
        lift = set_angle
        yaw = self.kinematic_joint_cmd.position[0]
        pitch = self.kinematic_joint_cmd.position[1]
        self.set_move_kinematic( tilt, lift, yaw, pitch)
        rospy.sleep(0.05)

    #head movements / left and right 
    def rotate_sample(self):
        current_time = time.time()
        set_angle = np.sin(self.time_scale*(current_time - self.start))
        tilt = self.kinematic_joint_cmd.position[0]
        lift = self.kinematic_joint_cmd.position[1]
        yaw = set_angle
        pitch = set_angle
        self.set_move_kinematic( tilt, lift, yaw, pitch)
        rospy.sleep(0.05)

movement = JointPublisher()
while not rospy.is_shutdown():
    movement.blink_sample()
    movement.ear_sample()
    movement.tail_sample()
    movement.head_sample()
    movement.rotate_sample()
    print(np.sin(movement.time_scale*(time.time() - movement.start)))