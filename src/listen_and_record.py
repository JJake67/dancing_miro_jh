#!/usr/bin/env python3

# Pathing
import rospkg

# Detecting
import os
import numpy as np
import rospy
from node_detect_audio_engine import DetectAudioEngine
from std_msgs.msg import Int16MultiArray,UInt16MultiArray, Bool
from std_srvs.srv import SetBool, SetBoolResponse

# Recording
import time
import sys
import wave, struct
import pydub
from RobotInterface import RobotInterface
from diss.msg import lights
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
        # Nothing to do with the recording :) 
        self.x_len = 20000
        # number of microphones coming through on topic
        self.no_of_mics = 4

        # which miro
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Service Node Setup 
        service_name = "listen_and_record_music"
        self.service = rospy.Service(service_name, SetBool, self.srv_callback)
        # save previous head data
        self.tmp = []
        #self.directory = os.getcwd()
        self.start_listening = False
        #Subscriber Node
        self.sub_mics = rospy.Subscriber(topic_base_name + "/sensors/mics",
            Int16MultiArray, self.callback_identify_mics, queue_size=1, tcp_nodelay=True)
        
        # status flags
        self.audio_event = None
        self.orienting = False
        self.action_time = 1 #secs
        self.thresh = 0.05
        # time
        self.frame_p = None

        # save previous head data
        #self.tmp = []
        # dynamic threshold
        self.thresh = 0
        self.thresh_min = 0.03
        self.listen = False
        self.input_mics = np.zeros((self.x_len, self.no_of_mics))
        # RECORDING INITIALISATIONS >>>>>>>

        # Create robot interface

        #self.micbuf = [np.zeros((0, 4), 'uint16')]
        self.micbuf = np.zeros((0, 4), 'uint16')
        self.outbuf = None
        self.buffer_stuff = 0
        self.playchan = 0
        self.playsamp = 0

        rospack = rospkg.RosPack()
        path = rospack.get_path('diss')
        self.directory = path
        #subscribers
        topic = topic_base_name + "/sensors/stream"
        #print ("subscribe", topic)
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=10, tcp_nodelay=True)
        
        # Does the same as below, but without cmd_vel
        #topic = topic_base_name + "/sensors/mics"
        #self.sub_mics = rospy.Subscriber(topic,UInt16MultiArray,self.callback_record_mics,queue_size=20)

        # THIS ONE IS PUBLISHING CMD_VEL NOT GOOD
        self.interface = RobotInterface(node_name=None)
        self.interface.register_callback("microphones", self.callback_record_mics)

        #publishers
        topic = topic_base_name + "/control/stream"
        #print ("publish", topic)
        
        self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

        topic_name = "light_topic"
        self.lightsPub = rospy.Publisher(topic_name, lights, queue_size=10)

        self.music_start_time = 0.0
        print("Listen and Record Service Node now active ...")
           
    def callback_identify_mics(self, data):
        # data for angular calculation
        # TRY WITHOUT THIS IF IT DOESNT WORK
        
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
            # when the buffer is 
            self.tmp = np.hstack((self.tmp[-10000:], np.abs(self.head_data)))
            # dynamic threshold is calculated and updated when new signal come
            self.thresh = self.thresh_min + AudioEng.non_silence_thresh(self.tmp)

        # data for display
        data = np.asarray(data.data)
        # 500 samples from each mics
        data = np.transpose(data.reshape((self.no_of_mics, 500)))
        data = np.flipud(data)
        #self.record_mics(data)  
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

    def callback_stream(self, msg):
        #print("this one")
        if self.start_listening : 
            self.buffer_space = msg.data[0]
            self.buffer_total = msg.data[1]
            self.buffer_stuff = self.buffer_total - self.buffer_space

    def callback_record_mics(self, msg):
        #print("here?")
        if self.start_listening:
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

    def record_audio(self):
        # Take client audio.py
        while not rospy.core.is_shutdown():

			# if recording finished
            if not self.outbuf is None:
                break

			# state
            rospy.sleep(0.02)
        
        # define output file 
        outfilename = self.directory + '/data/miro_audio.wav'
        file = wave.open(outfilename, 'w')    

        file.setsampwidth(2)
        file.setframerate(MIC_SAMPLE_RATE)

        print ("writing two channels to file (LEFT and RIGHT)...")
        file.setnchannels(2)
        x = np.reshape(self.outbuf[:, [0, 1]], (-1))
        print(len(x))
        for s in x:
            file.writeframes(struct.pack('<h', s))
        print(file.tell())
        file.close()

        #Converts to mp3
        print ("wrote output file at", outfilename)
        sound = pydub.AudioSegment.from_wav(outfilename)
        ogg_output = self.directory + '/data/miro_audio.ogg'
        sound.export(ogg_output,format="ogg")

    def srv_callback(self,request_from_client):
        # HOW TO GET PATH WHEN os.getcwd DOESNT WORK
        response_from_server = SetBoolResponse()
        print("yes")
        # This switch loops through MiRo behaviours:
        self.status_code = 0
        while request_from_client.data == True:
            # Step 1. sound event detection
            if self.status_code == 1:
                # Every once in a while, look for ball
                self.voice_accident()

            # Step 2. Orient towards it
            elif self.status_code == 2:
                ## Print that a sound has been detected
                print("Loud Signal Detected")
                print("Recording...")

    
                message = lights()
                message.move_name = "recording"
                self.lightsPub.publish(message)

                self.start_listening = True
                self.music_start_time = rospy.get_time()
                #print(self.music_start_time)
                self.record_audio()
                #rospy.sleep(5)
                print("here?")
                #clear the data collected when miro is turning
                self.audio_event=[]
                self.status_code = 0
                self.start_listening = False
                response_from_server.success = True
                response_from_server.message = str(self.music_start_time)
                # Resets these two so that the service can be called again
                self.micbuf = np.zeros((0, 4), 'uint16')
                self.outbuf = None
                return response_from_server

            # Fall back
            else:
                self.status_code = 1
    
    def loop(self):
        rospy.spin()
    

if __name__ == "__main__":
    rospy.init_node("listen_and_record", anonymous=True)
    AudioEng = DetectAudioEngine()
    main = listen_and_record()
    #plt.show() # to stop signal display next run: comment this line and line 89(self.ani...)
    main.loop()







"""
         # HOW TO GET PATH WHEN os.getcwd DOESNT WORK
        full_path = os.path.realpath(__file__)
        path, filename = os.path.split(full_path)

def loop(self):

        # This switch loops through MiRo behaviours:
        self.status_code = 0
        rospy.sleep(0.5)
        while not rospy.core.is_shutdown():
            if self.listen == True:
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
                    #rospy.sleep(5)
                    
                    #clear the data collected when miro is turning
                    self.audio_event=[]
                    self.status_code = 1

                # Fall back
                else:
                    self.status_code = 1

    def record_mics(self, msg):
        # if recording
        #l = []
        if not self.micbuf is None:

            # Data needs to be reshaped as it comes in from the mics 
            msg = np.reshape(msg,(500,4))

            #print(msg)
            # append mic data to store
            self.micbuf = np.concatenate((self.micbuf, msg))
            # report
            sys.stdout.write(".")
            sys.stdout.flush()
            rospy.sleep(0.02)
            # finished recording?
            if self.micbuf.shape[0] >= SAMPLE_COUNT:
                print(len(self.micbuf))
                    # end recording
                self.outbuf = self.micbuf
                self.micbuf = None
                print ("Recording Ended")

    
    def record_mics(self, msg):
        # if recording
        #l = []
        if self.record_now:

            # Data needs to be reshaped as it comes in from the mics 
            msg = np.reshape(msg,(500,4))
            if self.micbuf is None: 
                self.micbuf = np.zeros((0, 4), 'uint16') 
            #print(msg)
            # append mic data to store
            self.micbuf = np.concatenate((self.micbuf, msg))
            # report
            sys.stdout.write(".")
            sys.stdout.flush()
            rospy.sleep(0.02)
            # finished recording?
            if self.micbuf.shape[0] >= SAMPLE_COUNT:
                print(len(self.micbuf))
                    # end recording
                self.outbuf = self.micbuf
                self.micbuf = None
                print ("Recording Ended")

"""