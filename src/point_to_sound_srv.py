#!/usr/bin/env python3

"""
MiRo orienting towards a sound
"""

import os
import numpy as np
import rospy
import miro2 as miro
from node_detect_audio_engine import DetectAudioEngine
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist, TwistStamped
from std_srvs.srv import SetBool, SetBoolResponse
import time

from miro2.lib import wheel_speed2cmd_vel  # Python 3

# Dont think I need
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PointToSoundService():

    def __init__(self):

        # Number of points to display
        self.x_len = 40000

        self.no_of_mics = 4

        self.input_mics = np.zeros((self.x_len, self.no_of_mics))

        # Identifies the MiRo
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # save previous head data
        self.tmp = []

        # Thresholding for volume?
        self.thresh = 0
        self.thresh_min = 0.03

        # Subscribers
        self.sub_mics = rospy.Subscriber(topic_base_name + "/sensors/mics",
            Int16MultiArray, self.callback_mics, queue_size=1, tcp_nodelay=True)

        # Publishers
        self.pub_push = rospy.Publisher(topic_base_name + "/core/mpg/push", miro.msg.push, queue_size=0)
        self.pub_wheels = rospy.Publisher(topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0)

        # Prepares push message
        self.msg_push = miro.msg.push()
        self.msg_push.link = miro.constants.LINK_HEAD
        self.msg_push.flags = (miro.constants.PUSH_FLAG_NO_TRANSLATION + miro.constants.PUSH_FLAG_VELOCITY)


        # status flags
        self.audio_event = None
        self.orienting = False
        self.action_time = 1 #secs
        self.thresh = 0.05

        # time
        self.frame_p = None
        self.msg_wheels = TwistStamped()
        self.controller = miro.lib.PoseController()
        self.cmd_vel = miro.lib.DeltaPose()

        # Service Setup
        self.localise = False
        self.start_time = 0 
        self.music_start_time = 0

        service_name = "point_to_sound"
        rospy.init_node(f"{service_name}_server")
        self.service = rospy.Service(service_name, SetBool, self.srv_callback)
        rospy.loginfo(f"{service_name} Server is ready to be called")

    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        
        #Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        self.msg_wheels.twist.linear.x = dr
        self.msg_wheels.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.pub_wheels.publish(self.msg_wheels)

    def callback_mics(self, data):
        # data for angular calculation
        self.audio_event = AudioEng.process_data(data.data)

        # data for dynamic thresholding
        data_t = np.asarray(data.data, 'float32') * (1.0 / 32768.0) #normalize the high amplitude 
        data_t = data_t.reshape((4, 500))    
        self.head_data = data_t[2][:]

        if self.tmp is None:
            self.tmp = np.hstack((self.tmp, np.abs(self.head_data)))
        elif (len(self.tmp)<10500):
            self.tmp = np.hstack((self.tmp, np.abs(self.head_data)))
        else:
            # when the buffer is full
            self.tmp = np.hstack((self.tmp[-10000:], np.abs(self.head_data)))
            # dynamic threshold is calculated and updated when new signal come
            self.thresh = self.thresh_min + AudioEng.non_silence_thresh(self.tmp)

        # data for display
        data = np.asarray(data.data)
        # 500 samples from each mics
        data = np.transpose(data.reshape((self.no_of_mics, 500)))
        data = np.flipud(data)
        self.input_mics = np.vstack((data, self.input_mics[:self.x_len-500,:]))

    def srv_callback(self,request_from_client):
        
        response_from_server = SetBoolResponse()
        msg_wheels = TwistStamped()
        
        self.localise = request_from_client.data 
        # This switch loops through MiRo behaviours:
        # Listen to sound, turn to the sound source
        self.status_code = 0
        rospy.sleep(0.5)
        self.start_time = rospy.get_time()
        while self.localise:
            time_stood_still = rospy.get_time() - self.start_time
            # Step 1. sound event detection
            if self.status_code == 1:
                # Every once in a while, look for ball
               self.voice_accident()

            # Step 2. Orient towards it
            elif self.status_code == 2:
                if self.music_start_time == 0:
                    self.music_start_time = rospy.get_time()
                self.lock_onto_sound(self.frame)
                #clear the data collected when miro is turning
                self.audio_event=[]

            # Fall back
            else:
                self.status_code = 1
            #print(time_stood_still)
            if time_stood_still > 5:
                response_from_server.success = True
                response_from_server.message = str(self.music_start_time)
                return response_from_server

    def voice_accident(self):
        m = 0.00
        if self.audio_event != []:
            if self.audio_event != None:
                if self.audio_event[0] != None:
                    ae = self.audio_event[0]
                    #print(self.audio_event[2])
                    #print("Azimuth: {:.2f}; Elevation: {:.2f}; Level : {:.2f}".format(ae.azim, ae.elev, ae.level))
                    self.frame = self.audio_event[1]
                    m = (self.audio_event[2][0]+self.audio_event[2][1])/2
                    if m >= self.thresh:
                        self.status_code = 2
                    else:
                        self.status_code = 0
                else:
                    self.status_code = 0 
            else:
                self.status_code = 0 
        else:
            self.status_code = 0 

    def lock_onto_sound(self,ae_head):
        # detect if it is the frame within the same event
        # error may occur when the sound source is nearby the 90 degree of each side of MiRo
        # this is because the sample rate is not high enough for calculating in a higher accuracy
        # thus MiRo may considerred the angel inside (85,95) or (-95,-85) as the same angel
        if ae_head.x == self.frame_p:
            self.status_code = 0 

        else:
            # the frame is different: not from the same event
            self.frame_p = ae_head.x
            self.turn_to_sound()
            #print("MiRo is moving......")
            self.status_code = 0 

    def turn_to_sound(self): 
        if self.audio_event[0] is None:
            return
        #print("angular in degrees:{:.2f}".format(self.audio_event[0].ang))
        
        # Stops him moving when he's within 20 degrees of accuracy to the sound
        if not (self.audio_event[0].ang > -10 and self.audio_event[0].ang < 10):
            self.start_time = rospy.get_time()
            v = self.audio_event[0].azim
            #MiRo finish its rotation in 0.5s
            Tf = 0.5
            T1=0
            while(T1 <= Tf):

                self.drive(v,v)
                self.msg_wheels.twist.linear.x = 0.0
                self.msg_wheels.twist.angular.z = v*2

                self.pub_wheels.publish(self.msg_wheels)
                time.sleep(0.02)
                T1+=0.02

    def loop(self):
        rospy.spin()

if __name__ == "__main__":
    AudioEng = DetectAudioEngine()
    server = PointToSoundService()
    server.loop()
