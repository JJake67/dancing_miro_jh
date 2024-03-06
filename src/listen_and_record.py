#!/usr/bin/env python3

"""
MiRo orienting towards a sound
"""
# Detecting
import os
import numpy as np
import rospy
import miro2 as miro
from node_detect_audio_engine import DetectAudioEngine
from std_msgs.msg import Int16MultiArray,UInt16MultiArray
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Recording
import time
import sys
import wave, struct
import pydub

# Recording Setup 

BUFFER_STUFF_SAMPLES = 4000
MAX_STREAM_MSG_SIZE = (4096 - 48)
BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

RECORD_TIME = 5
MIC_SAMPLE_RATE = 20000
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE


class listen_and_record():
   
    def __init__(self):       
        # IDENTIFYING AUDIO INITIALISATIONS >>>>>>>>>
        #Microphone Parameters
        # Number of points to display
        self.x_len = 40000
        # number of microphones coming through on topic
        self.no_of_mics = 4

        # which miro
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
       
        # subscribers
        # save previous head data
        self.tmp = []

        #Subscriber Node
        self.sub_mics = rospy.Subscriber(topic_base_name + "/sensors/mics",
            Int16MultiArray, self.callback_identify_mics, queue_size=1, tcp_nodelay=True)
        
        # publishers
        self.pub_push = rospy.Publisher(topic_base_name + "/core/mpg/push", miro.msg.push, queue_size=0)

        # prepare push message
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
        self.controller = miro.lib.PoseController()
        self.cmd_vel = miro.lib.DeltaPose()

        # save previous head data
        #self.tmp = []
        # dynamic threshold
        self.thresh = 0
        self.thresh_min = 0.03

        # RECORDING INITIALISATIONS >>>>>>>

        # Create robot interface
        self.interface = miro.lib.RobotInterface()

        self.micbuf = np.zeros((0, 4), 'uint16')
        self.outbuf = None
        self.buffer_stuff = 0
        self.playchan = 0
        self.playsamp = 0

        #subscribers
        topic = topic_base_name + "/sensors/stream"
        print ("subscribe", topic)
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)
       
        self.interface.register_callback("microphones", self.callback_record_mics)

        #publishers
        topic = topic_base_name + "/control/stream"
        print ("publish", topic)
        self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)
        
    ## IDENTIFYING LOUD SIGNAL FUNCTIONS ----------------------------------------------------------

    def callback_identify_mics(self, data):
        import time

        # data for angular calculation
        self.audio_event = AudioEng.process_data(data.data)

        now = rospy.Time.now()
        zero_time = rospy.Time()

        #print ('Fields are', now.secs, now.nsecs)

        # Time arithmetic
        five_secs_ago = now - rospy.Duration(5) # Time minus Duration is a Time

        # NOTE: in general, you will want to avoid using time.time() in ROS code
        py_time = rospy.Time.from_sec(time.time())
        # 1.2 1.7 2.2  -> predict -> start in sync

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
    
    ## RECORDING THE AUDIO FUNCTIONS ---------------------------------------------------------------

    def callback_record_mics(self, msg):

            # if recording
            if not self.micbuf is None:

                # append mic data to store
                self.micbuf = np.concatenate((self.micbuf, msg.data))

                # report
                sys.stdout.write(".")
                sys.stdout.flush()

                # finished recording?
                if self.micbuf.shape[0] >= SAMPLE_COUNT:

                    # end recording
                    self.outbuf = self.micbuf
                    self.micbuf = None
                    print (" OK!")

    def callback_stream(self, msg):

        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]
        self.buffer_stuff = self.buffer_total - self.buffer_space

    def record_audio(self):
        # Take client audio.py
        while not rospy.core.is_shutdown():

			# if recording finished
            if not self.outbuf is None:
                break

			# state
            time.sleep(0.02)
        
        # define output file 
        directory = os.getcwd()
        outfilename = directory + '/data/miro_audio.wav'
        file = wave.open(outfilename, 'w')    

        file.setsampwidth(2)
        file.setframerate(MIC_SAMPLE_RATE)

        print ("writing two channels to file (LEFT and RIGHT)...")
        file.setnchannels(2)
        x = np.reshape(self.outbuf[:, [0, 1]], (-1))
        for s in x:
            file.writeframes(struct.pack('<h', s))
        print(file.tell())
        file.close()

        #Converts to mp3
        print ("wrote output file at", outfilename)
        sound = pydub.AudioSegment.from_wav(outfilename)
        mp3_output = directory + '/data/miro_audio.mp3'
        sound.export(mp3_output,format="mp3")

    def loop(self):

        # This switch loops through MiRo behaviours:
        # Listen to sound, turn to the sound source
        self.status_code = 0
        rospy.sleep(0.5)
        while not rospy.core.is_shutdown():
            
            
            # Step 1. sound event detection
            if self.status_code == 1:
                # Every once in a while, look for ball
               self.voice_accident()

            # Step 2. Orient towards it
            elif self.status_code == 2:
                ## Print that a sound has been detected
                print("Loud Signal Detected")
                print("Recording...")
                self.record_audio()
                
                #clear the data collected when miro is turning
                self.audio_event=[]

            # Fall back
            else:
                self.status_code = 1

if __name__ == "__main__":

    rospy.init_node("listen_and_record", anonymous=True)
    AudioEng = DetectAudioEngine()
    main = listen_and_record()
    #plt.show() # to stop signal display next run: comment this line and line 89(self.ani...)
    main.loop()