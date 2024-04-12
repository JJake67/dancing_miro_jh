#!/usr/bin/env python3
import os
import time
import rospy            # ROS Python interface
import numpy as np
import random
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt32MultiArray, String
from diss.msg import head

class JointPublisher(object):
    
    def __init__(self):
        rospy.init_node("joint_publisher")
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Define Joint parameters for easier publishing together
        # Cosmetic Joints 
        self.ear = 0.0
        self.eye = 0.0 
        self.tail = 0.0

        # Kinematic Joints
        self.head_yaw = 0.0
        self.head_pitch = 0.0
        self.neck_lift = 0.0

        # Kinematic Variables
        self.pitch_max = 0.14
        self.pitch_min = -0.38
        self.pitch_phase = 0

        self.yaw_max = 0 
        self.yaw_min = 0 
        self.yaw_phase = 0
        self.yaw_modifier = 1
        
        self.lift_max = 0
        self.lift_min = 0 
        self.lift_phase = 0
        self.lift_modifier = 1
        
        self.joint_chooser = 0 
        self.t_2bars = 0.0
        self.tempo = 0.5
        self.command = ""

        # Publishers
        self.kinematic_pub = rospy.Publisher(
            self.topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        self.cosmetic_pub = rospy.Publisher(
            self.topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

        # Subscriber
        topic_name = "head_topic"
        self.sub = rospy.Subscriber(topic_name, head, self.cmd_callback)
    
        # Defining joint commands
        self.kinematic_joint_cmd = JointState()
        self.kinematic_joint_cmd.position = [0, 0, 0, 0]
        self.cosmetic_joint_cmd = Float32MultiArray()   
        self.cosmetic_joint_cmd.data = [0,0,0,0,0,0]

        rospy.loginfo("Head/Neck Moves node is active...")

    # Callback for when subscriber to head_topic
    # reads in parameters published from Miro_Dance Node
    def cmd_callback(self,topic_message):  
        self.command = topic_message.move_name 
        if topic_message.tempo != 0:
            # Tempo is used in terms of beats per second so it gets converted when it is passed
            self.tempo = 60 / topic_message.tempo
        else:
            self.tempo = 0
    
    # V1: Used by the pre-programmed moves but is wrong :)
    def sine_generator(self, mx=1, mn=0, offset=0, freq=1, phase=0, t=1.0, t0=0):
        return ((mx-mn) * np.sin (freq*(t-t0) + phase) / 2.0 + offset)

    def new_sine_generator(self,mx=1, mn=0, freq=1, phase=0 ,t=1.0 ,t0=0):
        return ((mx-mn)) * (freq/2* math.pi) * (np.sin(freq*(t-t0)*2*math.pi+phase) / 2) + (mx - ((mx-mn)/2)) 

    # Would be the same as using new_sine_generator with a phase of pi/2
    def cosine_generator(self, mx=1, mn=0, offset=0, freq=1, phase=0, t=1.0, t0=0):
        return ((mx-mn) * np.cos (freq*(t-t0) + phase) / 2.0 + offset)

    # General cosmetic functions --------------------------------------------
    def move_ears(self,t,t0,freq):
        self.ear = self.new_sine_generator(1,0,freq,0,t,t0)
        
    def move_eyes(self,t,t0,freq):
        self.eye = self.new_sine_generator(1,0,freq,0,t,t0)

    def wag_tail(self,t,t0,freq):
        self.tail = self.new_sine_generator(0.5,0.2,freq,0,t,t0)

    # Sets his eyes openness based on bpm 
    def set_eyes(self,tempo):
        if tempo < 0.40:
            self.eye = 0
        elif tempo > 0.9:
            self.eye = 0.5
        else:
            self.eye = tempo - 0.4
    
    # General kinematic joints ---------------------------------------------
    # YAW MAX = 0.95, MIN = -0.95 (RAD) MAX = 55, MIN = -55 (DEG) 
    def move_head_yaw(self,t,t0,freq, y_max, y_min,y_phase):
        self.head_yaw = self.new_sine_generator(y_max,y_min,freq,0,t,t0)
        #print(self.head_yaw)

    # PITCH MAX = 0.14, MIN = -0.38 (RAD) MAX = 8, MIN = 22 (DEG)
    def move_head_pitch(self,t,t0,freq,p_max,p_min,p_phase):
        self.head_pitch = self.new_sine_generator(p_max,p_min,freq,0,t,t0)

    # LIFT MAX = 1.04, MIN = 0.14 (RAD) MAX = 60, MIN = 8 (DEG)
    def move_neck_lift(self,t,t0,freq,l_max,l_min,l_phase):
        self.neck_lift = self.new_sine_generator(l_max,l_min,freq,l_phase,t,t0)
        #print(self.neck_lift)

    # Publishes joint info, done every iteration --------------------------
    def publish_cosmetics(self):
        self.cosmetic_joint_cmd = Float32MultiArray()
        self.cosmetic_joint_cmd.data= [0,self.tail,self.eye,self.eye,self.ear,self.ear]
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    def publish_kinematics(self):
        self.kinematic_joint_cmd = JointState()

        self.kinematic_joint_cmd.position = [0,self.neck_lift,self.head_yaw,self.head_pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)
      
    # Pre-programmed Moveset (One will be picked randomly every other iteration) ------------------------------
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
        #freq = 2
        #pitch = abs(self.sine_generator(15,-15,0,freq,0,t,t0))-7
        #print(pitch)
        pitch = abs(self.new_sine_generator(0.26,-0.26,self.tempo,0,t,t0))-0.2

        self.kinematic_joint_cmd.position = [0,0,0,pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    def full_head_spin(self,t,t0):
        self.kinematic_joint_cmd = JointState()
        freq = self.tempo
        #yaw = self.sine_generator(55,-55,0,freq,0,t,t0)
        #pitch = self.cosine_generator(8,-22,0,freq,0,t,t0)
        yaw = self.new_sine_generator(0.95,-0.95,self.tempo,0,t,t0)
        pitch = self.new_sine_generator(0.14,-0.38,self.tempo,math.pi/2,t,t0)

        self.kinematic_joint_cmd.position = [0,0,yaw,pitch]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    def soul_Head_Bounce(self,t,t0):
        self.kinematic_joint_cmd = JointState()
        # range = 0.9 / 2 
        # upper = 0.59
        # 1.04 / 0.14
        bounce_f = self.tempo 
        #lift = abs(self.sine_generator(0,2,0,bounce_f,0,t,t0))
        lift = abs(self.new_sine_generator(0.3,-0.3,bounce_f,0,t,t0)) + 0.45

        yaw_f = self.tempo 
        #yaw = (self.cosine_generator(0,2,0,yaw_f,0,t,t0))
        yaw = self.new_sine_generator(0.95,-0.95,yaw_f,math.pi/2,t,t0)
        self.kinematic_joint_cmd.position = [0,lift,yaw,0]
        self.kinematic_pub.publish(self.kinematic_joint_cmd)

    def head_Banging(self,t,t0):
        #print ("Head Banging")
        self.kinematic_joint_cmd = JointState()
        self.cosmetic_joint_cmd = Float32MultiArray()

        #blink_f= self.tempo 
        #blink= self.sine_generator(0,0.8,1,blink_f,t,t0)
        #tempo * 2 coz otherwise even the blinks will be in sync with the head banging
        # maybe that'll look good though idk
        blink = self.new_sine_generator(0.8,0,self.tempo/2,0,t,t0)
        self.cosmetic_joint_cmd.data= [0,0,blink,blink,0,0]
        #print(blink)

        #pitch_freq = self.tempo
        #pitch = self.sine_generator(8, -22, 0, pitch_freq, 0, t, t0)
        pitch = self.new_sine_generator(0.14,-0.38,self.tempo,0,t,t0)
        #lift_freq = self.tempo
        #lift = self.sine_generator(8, -22, 0, lift_freq, 0, t, t0)
        lift = self.new_sine_generator(1.04,0.14,self.tempo,0,t,t0)
        #print(pitch)
        self.kinematic_joint_cmd.position = [0,lift,0,pitch]

        self.kinematic_pub.publish(self.kinematic_joint_cmd)
        self.cosmetic_pub.publish(self.cosmetic_joint_cmd)

    def change_lift_values(self):
        # MAX = 1.04 and MIN = 0.14 but random can only get a random integer
        joint_min = 14
        joint_max = 104
        joint_range = abs(joint_max)+abs(joint_min)
        print(joint_range)
        new_joint_min = 0 
        new_joint_max = 0 
        if self.tempo <= 0.45:
            # FAST SONG, so uses a 1/3 of the range of motion in wave
            new_joint_range = joint_range - int(joint_range /3)
            new_joint_min = random.randint(0,new_joint_range) + joint_min 
            new_joint_max = new_joint_min + (joint_range / 3)

            self.lift_min = float(new_joint_min/100)
            self.lift_max = float(new_joint_max/100)
            self.lift_modifier = 2
        elif self.tempo < 0.75:
            # MED SONG, uses 1/2 of the range 
            new_joint_range = joint_range - int(joint_range / 2)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 2)

            self.lift_min = float(new_joint_min/100)
            self.lift_max = float(new_joint_max/100)
            self.lift_modifier = 1
        else:
            # SLOW SONG, uses full range
            # DONT EVEN KNOW IF I NEED THESE LINES
            self.lift_min = float(joint_min/100)
            self.lift_max = float(joint_max/100)
            self.lift_modifier = 0.5
        self.lift_phase = round(random.uniform(0,math.pi))

        if (random.randint(1,5) == 1):
            print("neck turning off")
            self.lift_min = self.neck_lift
            #self.lift_min = round(random.uniform(0.14,1.04),2)
            self.lift_max = self.lift_min

    def change_pitch_values(self):
        # MAX = 0.14 and MIN = -0.38 but random can only get a random integer
        joint_min = -38
        joint_max = 14
        joint_range = abs(joint_max)+abs(joint_min)
        new_joint_min = 0 
        new_joint_max = 0 
        if self.tempo <= 0.45:
            # FAST SONG, so uses a 1/3 of the range of motion in wave
            new_joint_range = joint_range - int(joint_range / 3)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 3)

            self.pitch_min = float(new_joint_min/100)
            self.pitch_max = float(new_joint_max/100)
            self.pitch_modifier = 2
        elif self.tempo < 0.75:
            # MED SONG, uses 1/2 of the range 
            new_joint_range = joint_range - int(joint_range / 2)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 2)

            self.pitch_min = float(new_joint_min/100)
            self.pitch_max = float(new_joint_max/100)
            self.pitch_modifier = 0.5
            # THESE WILL PROBABLY NEED TO BE SELF. VALUES
        else:
            # SLOW SONG, uses full range
            # DONT EVEN KNOW IF I NEED THESE LINES
            self.pitch_min = float(joint_max/100)
            self.pitch_max = float(joint_max/100)
            self.pitch_modifier = 0.5
        self.pitch_phase = round(random.uniform(0,math.pi/2))

        if (random.randint(1,3) == 1):
            print("head tilt turning off")
            self.pitch_min = round(random.uniform(-0.38,0.14),2)
            self.pitch_max = self.pitch_min
    
    def change_yaw_values(self):
        # MAX = 0.95 and MIN = -0.95 but random can only get a random integer
        joint_min = -95
        joint_max = 95
        joint_range = abs(joint_max)+abs(joint_min)
        new_joint_min = 0 
        new_joint_max = 0 
        if self.tempo <= 0.45:
            # FAST SONG, so uses a 1/3 of the range of motion in wave
            new_joint_range = joint_range - int(joint_range / 3)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 3)

            self.yaw_min = float(new_joint_min/100)
            self.yaw_max = float(new_joint_max/100)
            self.yaw_modifier = 2
        elif self.tempo < 0.75:
            # MED SONG, uses 1/2 of the range 
            new_joint_range = joint_range - int(joint_range / 2)
            new_joint_min = random.randint(0,new_joint_range) + joint_min
            new_joint_max = new_joint_min + int(joint_range / 2)

            self.yaw_min = float(new_joint_min/100)
            self.yaw_max = float(new_joint_max/100)
            self.yaw_modifier = 1
            # THESE WILL PROBABLY NEED TO BE SELF. VALUES
        else:
            # SLOW SONG, uses full range
            # DONT EVEN KNOW IF I NEED THESE LINES
            self.yaw_min = float(joint_min/100)
            self.yaw_max = float(joint_max/100)
            self.yaw_modifier = 0.5
        self.yaw_phase = round(random.uniform(0,math.pi))
        if (random.randint(1,3) == 1):
            print("head side to side turning off")
            self.yaw_min = round(random.uniform(-0.95,0.95),2)
            self.yaw_min = self.head_yaw
            self.yaw_max = self.yaw_min

    # Puts all joints to a "normal" position, after song and when not dancing
    # just for user experience sakes
    def reset_all_joints(self):
        self.eye = self.eye
        self.neck_lift = 0.29
        self.head_pitch = -0.15
        self.head_yaw = 0.0

        self.ear = 0.5

        self.publish_cosmetics()
        self.publish_kinematics()  

    def loop(self,t,t0):
        # Specific Dance Move
        if self.command != "" and self.command != "done":
            
            if self.command == "reset":
                self.reset_all_joints()
            elif self.command == "head_bounce":
                self.soul_Head_Bounce(t,t0)
            elif self.command == "head_bang":
                self.head_Banging(t,t0)
            elif self.command == "full_head_spin":
                self.full_head_spin(t,t0)
            elif self.command == "head_bop":
                self.head_bop(t,t0)
            else:
                self.head_bop(t,t0)
                #print("head bop but not the real one")

        # General Dancing 
        # LIFT = NECK, YAW = SIDE TO SIDE, PITCH = UP AND DOWN
        elif self.tempo != 0.0:

            # Publish Eye / Ear Commands, these dont change once set  
            self.set_eyes(self.tempo)
            self.move_ears(t,t0,self.tempo)

            # Publish Yaw / Pitch / Lift Commands
            self.move_head_pitch(t,t0,self.tempo,self.pitch_max,self.pitch_min,self.pitch_phase)
            self.move_neck_lift(t,t0,self.tempo*self.lift_modifier,self.lift_max,self.lift_min,self.lift_phase)
            self.move_head_yaw(t,t0,self.tempo*self.yaw_modifier,self.yaw_max,self.yaw_min,self.yaw_phase)
            rospy.sleep(0.02)

            # Every 2 bars, switches the max / min of a joint
            # random chance as well that that joint stops moving, adds dynamicity
            if self.t_2bars <= t:
                self.t_2bars = t + (8*self.tempo)

                if self.joint_chooser == 0:
                    self.change_yaw_values()
                    print("new yaws")
                    print(self.yaw_max)
                    print(self.yaw_min)
                    self.joint_chooser =+1

                elif self.joint_chooser == 1:
                    self.change_lift_values()
                    self.joint_chooser = 0
                    print("new lifts")
                    print(self.lift_max)
                    print(self.lift_min)


            self.publish_cosmetics()
            self.publish_kinematics()

movement = JointPublisher()
t0 = rospy.get_time()

#movement.tempo = 60 / 60 
#movement.tempo = 0.5
while not rospy.is_shutdown():
    t = rospy.get_time() 
    movement.head_bop(t,t0)
    #movement.loop(t,t0)
    rospy.sleep(0.02)





"""
if self.num_of_joints == 2:
                self.move_ears(t,t0,self.tempo*self.ear_modifier)
                self.move_head_yaw(t,t0,self.tempo*self.yaw_modifier)
            if self.num_of_joints == 4:
                self.move_ears(t,t0,self.tempo*self.ear_modifier)
                self.move_head_yaw(t,t0,self.tempo*self.yaw_modifier)
                self.wag_tail(t,t0,self.tempo*self.tail_modifier)
                self.move_eyes(t,t0,self.tempo*self.eye_modifier) 
            if self.num_of_joints == 6:  
                self.move_ears(t,t0,self.tempo*self.ear_modifier)
                self.move_head_yaw(t,t0,self.tempo*self.yaw_modifier)
                self.wag_tail(t,t0,self.tempo*self.tail_modifier)
                self.move_eyes(t,t0,self.tempo*self.eye_modifier) 
                self.move_head_pitch(t,t0,self.tempo*self.pitch_modifier)
                self.move_neck(t,t0,self.tempo*self.lift_modifier)
                # Switches up the tempos of each joint every 16 beats / 4 bars to keep it fresh 
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
"""