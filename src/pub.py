#!/usr/bin/env python3
import os
import time
import rospy            # ROS Python interface
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt32MultiArray, String
from dancing_miro.msg import head

class JointPublisher(object):
    
    def __init__(self):
        rospy.init_node("joint_publisher")
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Define Joint parameters for easier publishing together
        self.ear = 0.0
        self.eye = 0.0 
        self.tail = 0.0

        self.head_tilt = 0.0
        self.head_yaw = 0.0
        self.head_pitch = 0.0
        self.neck_lift = 0.0

        self.tempo = 2
        self.command = ""

        self.start = rospy.Time.now().to_sec()

        # Publishers and Subscribers 
        self.kinematic_pub = rospy.Publisher(
            self.topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        self.cosmetic_pub = rospy.Publisher(
            self.topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

        topic_name = "head_topic"
        self.sub = rospy.Subscriber(topic_name, head, self.cmd_callback)
        
        
        rospy.loginfo("Head/Neck Moves node is active...")

        # Defining joint commands
        self.kinematic_joint_cmd = JointState()
        self.kinematic_joint_cmd.position = [0, 0, 0, 0]
        self.cosmetic_joint_cmd = Float32MultiArray()   
        self.cosmetic_joint_cmd.data = [0,0,0,0,0,0]

    # Callback for when parameters are passed from Miro_Dance Node 
    def cmd_callback(self,topic_message):
        print(f'Node obtained msg: {topic_message.move_name}')
        print(f'Node also said: {topic_message.mode}')
        #print(topic_message.tempo)
        self.command = topic_message.move_name 
        self.tempo = 60 / topic_message.tempo
    
    # I DONT UNDERSTAND WHY THESE EXIST WHEN YOU CAN JUST WRITE THE COMMAND JUST AS EASILY
    # movement for either tilt, lift, yaw or pitch
    def set_move_kinematic(self, tilt = 0, lift = 0, yaw = 0, pitch = 0):
        self.kinematic_joint_cmd.position = [tilt, lift, yaw, pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    # movement for the tail, eye lid, ears
    def set_move_cosmetic(self, tail_pitch = 0, tail_yaw = 0, left_eye = 0, right_eye = 0, left_ear = 0, right_ear = 0):
        self.cosmetic_joint_cmd.data = [tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    # Resets ears / eyes / tail to calibration(middle) settings
    def set_move_cosmetic2(self):
        self.cosmetic_joint_cmd = Float32MultiArray() 
        # Puts them all to their default position I believe
        self.data_calib = [0,0.5,0.5,0.5,0.33,0.33] 
        self.cosmetic_joint_cmd.data = self.data_calib
        #self.cosmetic_joint_cmd.data = [0,0,0,0,0,0]
            
        #self.cosmetic_joint_cmd.data = [tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)
    
    def sine_generator(self, mx=1, mn=0, offset=0, freq=1, phase=0, t=1.0, t0=0):
        return ((mx-mn) * np.sin (freq*(t-t0) + phase) / 2.0 + offset)

    def new_sine_generator(self,mx=1, mn=0, freq=1, phase=0 ,t=1.0 ,t0=0):
        return ((mx-mn)) * (np.sin(freq*(t-t0) + phase) / 2) + (mx - ((mx-mn)/2)) 

    def cosine_generator(self, mx=1, mn=0, offset=0, freq=1, phase=0, t=1.0, t0=0):
        return ((mx-mn) * np.cos (freq*(t-t0) + phase) / 2.0 + offset)

    # Ears limits are mx = 1, mn = 0
    # Calibration point = 0.33
    def move_ears(self,t,t0,freq):
        self.ear = self.new_sine_generator(1,0,freq,0,t,t0)
    
    # Eye limits are mx = 1, mn = 0 
    # Calibration point = 0.5
    def move_eyes(self,t,t0,freq):
        self.eye = self.new_sine_generator(1,0,freq,0,t,t0)

    def wag_tail(self,t,t0,freq):
        self.tail = self.new_sine_generator(0.5,0.2,freq,0,t,t0)

    def move_head_yaw(self,t,t0,freq):
        self.head_yaw = self.new_sine_generator(15,-15,freq,0,t,t0)

    def move_head_pitch(self,t,t0,freq):
        self.head_pitch = self.new_sine_generator(8,-22,freq,0,t,t0)

    def move_neck(self,t,t0,freq):
        self.neck_lift = self.new_sine_generator(8,-22,freq,0,t,t0)

    # Cosmetic Publisher controls the tail, eyes and ears
    def publish_cosmetics(self):
        self.cosmetic_joint_cmd = Float32MultiArray()
        self.cosmetic_joint_cmd.data= [0,self.tail,self.eye,self.eye,self.ear,self.ear]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    def publish_kinematics(self):
        self.kinematic_joint_cmd = JointState()

        self.kinematic_joint_cmd.position = [0,self.neck_lift,self.head_yaw,self.head_pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

        
    # SPECIFIC DANCE MOVES ------------------------------
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
    
    def head_bop(self,t,t0):
        self.kinematic_joint_cmd = JointState()
        freq = 2
        pitch = abs(self.sine_generator(15,-15,0,freq,0,t,t0))-7

        self.kinematic_joint_cmd.position = [0,0,0,pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    # More Square
    def full_head_spin(self,t,t0):
        self.kinematic_joint_cmd = JointState()
        freq = 1
        yaw = self.sine_generator(55,-55,0,freq,0,t,t0)
        pitch = self.cosine_generator(8,-22,0,freq,0,t,t0)

        self.kinematic_joint_cmd.position = [0,0,yaw,pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    def soul_Head_Bounce(self,t,t0):
        self.kinematic_joint_cmd = JointState()

        bounce_f = self.tempo
        lift = abs(self.sine_generator(0,2,0,bounce_f,0,t,t0))
        #print(lift)

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

        pitch_freq = 2
        pitch = self.sine_generator(8, -22, 0, pitch_freq, 0, t, t0)
        lift_freq = 2
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

    def loop(self,t,t0):
        self.wag_tail(t,t0,5)
        self.move_ears(t,t0,3)
        self.move_eyes(t,t0,2)
        self.move_head_yaw(t,t0,1)
        self.move_head_pitch(t,t0,3)
        self.move_neck(t,t0,2)
        if self.tempo != 0.0:
            self.wag_tail(t,t0,self.tempo)
            self.move_ears(t,t0,self.tempo)
            self.move_eyes(t,t0,self.tempo*4)
            self.move_head_yaw(t,t0,self.tempo*2)
            self.move_head_pitch(t,t0,self.tempo)
            self.move_neck(t,t0,self.tempo*0.5)
        self.publish_cosmetics()
        self.publish_kinematics()


movement = JointPublisher()
t0 = rospy.get_time()
while not rospy.is_shutdown():
    t = rospy.get_time() 
    movement.loop(t,t0)
    #movement.wag_tail(t,t0,5)
    #movement.move_ears(t,t0,2)
    #movement.move_eyes(t,t0,3)
    #movement.move_head_yaw(t,t0,1)
    #movement.move_head_pitch(t,t0,3)
    #movement.move_neck(t,t0,2)
    #if movement.tempo != 0.0:
    #    movement.wag_tail(t,t0,tempo)
    #    movement.move_ears(t,t0,tempo)
    #    movement.move_eyes(t,t0,tempo)
    #    movement.move_head_yaw(t,t0,tempo)
    #    movement.move_head_pitch(t,t0,tempo)
    #    movement.move_neck(t,t0,tempo)  
    #movement.publish_cosmetics()
    #movement.publish_kinematics()
    #movement.head_Banging(t,t0)
    #movement.the_Robot(t,t0)
    #movement.soul_Head_Bounce(t,t0)
    #movement.yaw_and_pitch(t,t0)
    #movement.cosmetic_pub_test(t,t0)
    #movement.head_bop(t,t0)
    rospy.sleep(0.1)


"""
    # Cosmetic_joint_cmd.data : [tail (up/down), tail (left/right), eye, eye, ear, ear]
    
    # Kinematic_joint_cmd.data : [tilt, lift, yaw, pitch]
    # Lift = Neck
    # Tilt = Head Tilt ( THOUGH SEEMS TO BE NOT REAL)
    # Yaw = Head Facing Left / Right  * LESS JERKY WHEN FASTER IDK WHY 
    # Pitch = Head Facing Up / Down

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

    # THIS WAS IN while not rospy.isSHutdown() idk what it does
    #print(np.sin(movement.time_scale*(time.time() - movement.start))) 

        self.time_scale = 0.1#for how fast the movement is 
        self.blink_freq = 0.5 #for how fast the movement is 
        self.ear_freq = 1 #for how fast the movement is 

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
"""