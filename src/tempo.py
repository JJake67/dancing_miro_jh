#!/usr/bin/env python3

import rospy
import librosa
import librosa.display
import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, SetBoolResponse

class estTempoAndBeats():

    def __init__(self):
        service_name = "estimate_tempo_and_beats"
        rospy.init_node(f"{service_name}_server")
        self.service = rospy.Service(service_name,SetBool, self.srv_callback)
        
        rospy.loginfo(f"{service_name} Server is ready to be called")

    def srv_callback(self, request_from_client):
        print("service called")
        response_from_server = SetBoolResponse()
        if request_from_client.data == True:
            y, sr = librosa.load('data/smooth (2).wav')
            hop_length = 512 

            # Compute local onset autocorrelation
            oenv = librosa.onset.onset_strength(y=y, sr=sr, hop_length=hop_length)
            times = librosa.times_like(oenv, sr=sr, hop_length=hop_length)

            # Estimate the global tempo for display purposes
            #tempo = librosa.beat.tempo(onset_envelope=oenv, sr=sr,
            #                        hop_length=hop_length)[0]
            tempo = librosa.feature.tempo(onset_envelope=oenv, sr=sr,
                                    hop_length=hop_length)[0]

            # Returns all the beats 
            tempo, beats = librosa.beat.beat_track(y=y,sr=sr)
            # Returns the time stamp of each beat
            time_stamps = librosa.frames_to_time(beats,sr=sr)
            #print(time_stamps)
            #print ("Estimated tempo bpm = " ,tempo)
            response_from_server.success = True
            response_from_server.message = str(tempo) + " " + str(time_stamps[-1])
        else : 
            response_from_server.success = False
            response_from_server.message = "No request"
        return response_from_server

    def main(self):
        rospy.spin()
if __name__ == '__main__':
    server = estTempoAndBeats()
    server.main()