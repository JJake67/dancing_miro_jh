#!/usr/bin/env python3
import os
import time
import rospy            # ROS Python interface
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt32MultiArray

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
        self.time_scale = 0.1#for how fast the movement is 
        self.blink_freq = 0.5 #for how fast the movement is 
        self.ear_freq = 1 #for how fast the movement is 
        self.kinematic_joint_cmd = JointState()
        self.kinematic_joint_cmd.position = [0, 0, 0, 0]
        self.cosmetic_joint_cmd = Float32MultiArray()   
        self.cosmetic_joint_cmd.data = [0,0,0,0,0,0]

    # movement for either tilt, lift, yaw or pitch
    def set_move_kinematic(self, tilt = 0, lift = 0, yaw = 0, pitch = 0):
        self.kinematic_joint_cmd.position = [tilt, lift, yaw, pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    # movement for the tail, eye lid, ears
    def set_move_cosmetic2(self):
        self.cosmetic_joint_cmd = Float32MultiArray()  
        self.data_calib = [0,0.5,0.5,0.5,0.33,0.33] 
        self.cosmetic_joint_cmd.data = self.data_calib
        #self.cosmetic_joint_cmd.data = [0,0,0,0,0,0]
            
        #self.cosmetic_joint_cmd.data = [tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    # movement for the tail, eye lid, ears
    def set_move_cosmetic(self, tail_pitch = 0, tail_yaw = 0, left_eye = 0, right_eye = 0, left_ear = 0, right_ear = 0):
        self.cosmetic_joint_cmd.data = [tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    # a sample on how the miro can blink
    def blink_sample(self):
        current_time = rospy.get_time()
        set_angle = np.sin(self.blink_freq*(current_time - self.start))
        tail_pitch = self.cosmetic_joint_cmd.data[0]
        tail_yaw = self.cosmetic_joint_cmd.data[1]
        left_eye = set_angle
        right_eye = set_angle
        left_ear = self.cosmetic_joint_cmd.data[4]
        right_ear = self.cosmetic_joint_cmd.data[5]
        self.set_move_cosmetic2(tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear)
        #rospy.sleep(60)
   
    # ear movement sample
    def ear_sample(self):
        current_time = rospy.get_time()
        set_angle = np.sin(self.ear_freq*(current_time - self.start))
        tail_pitch = self.cosmetic_joint_cmd.data[0]
        tail_yaw = self.cosmetic_joint_cmd.data[1]
        left_eye = self.cosmetic_joint_cmd.data[2]
        right_eye = self.cosmetic_joint_cmd.data[3]
        left_ear = set_angle
        right_ear = set_angle
        self.set_move_cosmetic(tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear)
        rospy.sleep(0.05)
   
    #rospy.time.noW() NOT A THING USE rospy.get_time()
    #tail
    def tail_sample(self):
        current_time = rospy.get_time()
        set_angle = np.sin(self.time_scale*(current_time - self.start))
        left_eye = self.cosmetic_joint_cmd.data[0]
        right_eye = self.cosmetic_joint_cmd.data[1]
        tail_pitch = set_angle
        tail_yaw = set_angle
        left_ear = self.cosmetic_joint_cmd.data[2]
        right_ear = self.cosmetic_joint_cmd.data[3]
        self.set_move_cosmetic(tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear)
        #rospy.sleep(0.05)
    #tilt RANGE (0 - 1)
    #head movements/ up and down
    def head_sample(self):
        current_time = rospy.get_time()
        set_angle = np.sin(self.time_scale*(current_time - self.start))
        tilt = set_angle
        lift = set_angle
        yaw = self.kinematic_joint_cmd.position[0]
        pitch = self.kinematic_joint_cmd.position[1]
        #self.set_move_kinematic(tilt, lift, yaw, pitch)
        self.set_move_kinematic(tilt,0,0,0)
        print(set_angle)
        #self.set_move_kinematic(0,0,yaw,pitch)
        rospy.sleep(0.05)

    #head movements / left and right 
    def rotate_sample(self):
        current_time = rospy.get_time()
        set_angle = np.sin(self.time_scale*(current_time - self.start))
        tilt = self.kinematic_joint_cmd.position[0]
        lift = self.kinematic_joint_cmd.position[1]
        yaw = set_angle
        pitch = set_angle
        self.set_move_kinematic( tilt, lift, yaw, pitch)
        #rospy.sleep(0.05)

    def sine_generator(self, mx=1, mn=0, offset=0, freq=1, phase=0, t=1.0, t0=0):
        return ((mx-mn) * np.sin (freq*(t-t0) + phase) / 2.0 + offset)

    def cosine_generator(self, mx=1, mn=0, offset=0, freq=1, phase=0, t=1.0, t0=0):
        return ((mx-mn) * np.cos (freq*(t-t0) + phase) / 2.0 + offset)
    # Rotates Head Side to Side 
    # Yaw Freq determines how quickly he swaps directions NOT his speed 
    def yaw_movement(self, t, t0):
        self.kinematic_joint_cmd = JointState()
        
        yaw_freq = 0.5
        yaw = self.sine_generator(55, -55, 0, yaw_freq, 0, t, t0)
        print(yaw)
        self.kinematic_joint_cmd.position = [0, 0, yaw, 0]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    # Rotates Head Up and Down 
    # Pitch Freq determines how often he swaps directions
    #def pitch_movement(self, tilt = 0, lift = 0, yaw = 0, pitch = 0):
    def pitch_movement(self, t, t0):
        self.kinematic_joint_cmd = JointState()
        pitch_freq = 3
        pitch = self.sine_generator(8, -22, 0, pitch_freq, 0, t, t0)
        print(pitch)
        self.kinematic_joint_cmd.position = [0,2,0,pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)
        #pass
    
    def yaw_and_pitch(self,t,t0):
        self.kinematic_joint_cmd = JointState()
        pitch_freq = 2 
        yaw_freq = 2
        pitch = self.sine_generator(8, -22, 0, pitch_freq, 0, t, t0)
        yaw = self.cosine_generator(55, -55, 0, yaw_freq, 0, t, t0)
        self.kinematic_joint_cmd.position = [0, 0, yaw, pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    def blink_m(self,t,t0):
        self.cosmetic_joint_cmd = Float32MultiArray()
        blink_f=1
        blink= self.sine_generator(0,0.5,0.5,blink_f,t,t0)
        print(blink)
        self.cosmetic_joint_cmd.data= [0,0,blink,blink,0,0]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)
    
    def ear_m(self,t,t0):
        self.cosmetic_joint_cmd = Float32MultiArray()
        ear_f=50
        ear= self.sine_generator(0,0.33,0.5,ear_f,t,t0)
        print(ear)
        self.cosmetic_joint_cmd.data= [0,0,0,0,ear,ear]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    def tail(self,t,t0):
        self.cosmetic_joint_cmd = Float32MultiArray()
        tail_f= 90
        tail= self.sine_generator(0,0.5,0.5,tail_f,t,t0)
        print(tail)
        self.cosmetic_joint_cmd.data= [0,tail,0,0,0,0]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    # Cosmetic_joint_cmd.data : [?, tail, eye, eye, ear, ear]
    # Kinematic_joint_cmd.data : [tilt, life, yaw, pitch]
    # Lift = Neck
    # Tilt = Head Tilt ( THOUGH SEEMS TO BE NOT REAL)
    # Yaw = Head Facing Left / Right
    # Pitch = Head Facing Up / Down

    # I think for all the kinematic stuff MAX = 1, MIN = 0, thats why they used sine generator 
    # NEED TO DO 
    # Define a lot of functions that execute SPECIFIC dance moves 
    #   Gonna Need Around ~20 Start with 5 
    #   - Rock One (Head Banging)
    #   - Pop One (Happy Dancing)
    #   - Soul One (Eyes Closed, Slow moves, Boppy?)
    #   - The Robot (Jerky but intentional )
    #   - Hip-Hop One (Unts Unts Unts Unts)
    # Define the base function that will run autonomously when a specific dance move is being performed.

    # VER 1 : Moves just look janky unintentionally 
    def the_Robot(self,t,t0):
        #print ("Doing The Robot")
        # Range of -15 to 15
        yaw_freq = 2
        self.kinematic_joint_cmd = JointState()
        yaw = self.sine_generator(8, -22, 0, yaw_freq, 0, t, t0)     
        #print(yaw)   
        if yaw < -10:
            act_yaw = -3
        elif yaw < 5:
            act_yaw = 0
        else:
            act_yaw = 3

        lift_freq = 1
        lift = self.sine_generator(0,1, 0, lift_freq, 0, t, t0)
        if lift < -0.32:
            act_lift = 0.1
        elif lift < 0:
            act_lift = 0.35
        elif lift < 0.32:
            act_lift = 0.65
        else:
            act_lift = 0.9
        print(lift)
        
        self.kinematic_joint_cmd.position = [0,act_lift,act_yaw,0]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    def soul_Head_Bounce(self,t,t0):
        self.kinematic_joint_cmd = JointState()

        bounce_f = 1
        lift = abs(self.sine_generator(0,2,0,bounce_f,0,t,t0))
        print(lift)

        yaw_f = bounce_f
        yaw = (self.cosine_generator(0,2,0,yaw_f,0,t,t0))
        self.kinematic_joint_cmd.position = [0,lift,yaw,0]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)


    def head_Banging(self,t,t0):
        #print ("Head Banging")
        self.kinematic_joint_cmd = JointState()
        self.cosmetic_joint_cmd = Float32MultiArray()

        blink_f=1
        blink= self.sine_generator(0,0.8,1,blink_f,t,t0)
        self.cosmetic_joint_cmd.data= [0,0,blink,blink,0,0]
        #print(blink)

        pitch_freq = 3.4
        pitch = self.sine_generator(8, -22, 0, pitch_freq, 0, t, t0)
        lift_freq = 3.4
        lift = self.sine_generator(8, -22, 0, lift_freq, 0, t, t0)
        #print(pitch)
        self.kinematic_joint_cmd.position = [0,lift,0,pitch]

        self.kinematic_pub.publish(self.kinematic_joint_cmd)
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    def cosmetic_pub_test(self,t,t0):
        self.cosmetic_joint_cmd = Float32MultiArray()

        blink_f=1
        blink= self.sine_generator(0,0,0.5,blink_f,t,t0)
        self.cosmetic_joint_cmd.data= [0,0,blink,blink,0,0]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

movement = JointPublisher()
t0 = rospy.Time.now().to_sec()
while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec()
    #ovement.blink_m(t,t0)
    #movement.ear_m(t,t0)
    #movement.tail(t,t0)
    #movement.yaw_movement(t, t0)
    #movement.pitch_movement(t, t0)
    #movement.head_Banging(t,t0)
    #movement.the_Robot(t,t0)
    movement.soul_Head_Bounce(t,t0)
    #movement.yaw_and_pitch(t,t0)
    #movement.cosmetic_pub_test(t,t0)
    rospy.sleep(0.05)
    #movement.blink_sample()
    # 
    
    #movement.ear_sample()
    #movement.tail_sample()
    #movement.head_sample()
    #movement.rotate_sample()
    #movement.set_move_cosmetic2()

    #print(np.sin(movement.time_scale*(time.time() - movement.start))) 



"""
LaSolitude Estimated tempo bpm =  143.5546875 = 417.958638ms = 2.38hzs frequency/angular velocity = 14.97 radian per seconds 

Sine generators for different joints:
A*sin(w*t + phi)

1. Yaw
min = -55, max = 55,
yaw_joint_pose = 55 * sin (self.yaw_freq*(current_time - self.start) + phase)

2. Pitch
min = -22, max = 8,
pitch_joint_pose = 15 * sin (self.yaw_freq*(current_time - self.start) + phase) - 7

3. Ear 
calib = 0.3333
ear_counts_0 = 1750
ear_count_1 = 1000

4. Eye 
calib = 0.5
eye_counts_0 = 1000
eye_counts_1 = 2000

5. Wagging tail 
calib = 0.5 default set 
wag_counts_0 = 1300
wag_counts_1= 1700 
mdk/share/python/miro2/constants.py
"""