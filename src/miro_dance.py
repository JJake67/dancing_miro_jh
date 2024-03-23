#!/usr/bin/env python3
import rospy
import os
import base64
import json
import random
import numpy as np
from requests import post, get
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest
from diss.msg import body, lights, head
# from msg_place import SetString, SetStringResponse
class MiroDance(object):

    # Main Node for Creating Dance Movements

    def __init__(self,dance_mode):
        self.dance_mode = dance_mode
        self.ctrl_c = False
        self.genre = ""
        self.duration = 0.0 
        #self.tempo = 0.0
        self.end_of_fade_in = 0.0
        self.track_start = 0.0
        
        self.bars = []
        self.sections = [] 
        
        # FOR TESTING
        #self.sections = [0,6,12,18,24,30,36,42,48]
        #self.tempo = 120
        self.song_name = ""
        self.last_beat = 0

        # For Dancing
        self.head_dance_move = ""
        self.head_move_names = ["head_bounce","head_bang","full_head_spin","head_bop"]
        self.body_dance_move = ""
        self.lights_move = ""
        #self.lights_colours = ""

        # Audio Features 
        self.danceability = 0.0
        self.valence = 0.0 

        # Required for device to access Spotify API app 
        self.client_id = "acbd6c4e089e4c9cb071ce9d3e4a9583"
        self.client_secret = "a35830533528441f9ae304893a279b38"
        
        # SERVICES
        
        ##UNCOMMENT FOR LISTEN_AND_RECORD
        service_name = "listen_and_record_music"

        rospy.wait_for_service(service_name)
        self.service_record = rospy.ServiceProxy(service_name,SetBool)
        
        self.request_to_record = SetBoolRequest()
        self.request_to_record.data = True
        """
        service_name = "estimate_tempo_and_beats"
        rospy.wait_for_service(service_name)
        self.service_tempo = rospy.ServiceProxy(service_name, SetBool)

        self.request_for_tempo = SetBoolRequest()
        self.request_for_tempo.data = True 
        """
        if self.dance_mode == "Spotify":
            service_name = "identify_song"

            rospy.wait_for_service(service_name) 
            self.service_identify = rospy.ServiceProxy(service_name, SetBool)

            self.request_to_identify = SetBoolRequest()
            self.request_to_identify.data = True 



        # SUBSCRIBERS

        # PUBLISHERS
        # body_msg : body movements
        topic_name = "body_topic"
        self.bodyPub = rospy.Publisher(topic_name, body, queue_size=10)

        topic_name = "head_topic"
        self.headPub = rospy.Publisher(topic_name, head, queue_size=10)

        topic_name = "light_topic"
        self.lightsPub = rospy.Publisher(topic_name, lights, queue_size=10)

        # localise_now : point_to_responsesound msg
        
        rospy.loginfo("Miro_Dance Node is Active...")

    # SPOTIFY 
    def set_track_data(self):

        # Finds Song on Spotify 
        url = "https://api.spotify.com/v1/search"
        token = self.get_token()
        headers = self.get_auth_headers(token)

        query = f"?q={self.song_name}&type=track&limit=1"
        query_url = url + query
        result = get(query_url, headers=headers)
        json_result = json.loads(result.content)["tracks"]["items"]

        # If no results were found searching song name, can't continue
        if len(json_result) == 0:
            print("noor sorryyy")
            return None
        
        # Song ID
        track = json_result[0]
        track_id = track["id"]
        artist_id = track["artists"][0]["id"]
        
        # Song Genre 
        url = f"https://api.spotify.com/v1/artists/{artist_id}"
        result = get(url, headers=headers)
        json_result = json.loads(result.content)
        # Returns multiple, need to find way to differentiate as they're all kinda random
        self.genre = json_result["genres"]

        # Audio Analysis
        url = f"https://api.spotify.com/v1/audio-analysis/{track_id}"
        result = get(url, headers=headers)
        audio_analysis_dict = json.loads(result.content)
        self.duration = audio_analysis_dict["track"]["duration"]
        self.tempo = int(audio_analysis_dict["track"]["tempo"])
        self.end_of_fade_in = audio_analysis_dict["track"]["end_of_fade_in"]

        # Finds the time the first beat starts so each one after that will be in time
        self.track_start = audio_analysis_dict["beats"][0]["start"]

        for x in range(0,len(audio_analysis_dict["bars"])):
            self.bars.append(audio_analysis_dict["bars"][x]["start"])

        for x in range(0,len(audio_analysis_dict["sections"])):
            self.sections.append(audio_analysis_dict["sections"][x]["start"])

        # Audio Features
        url = f"https://api.spotify.com/v1/audio-features/{track_id}"
        result = get(url, headers=headers)
        audio_feat_dict = json.loads(result.content)
        self.danceability = audio_feat_dict["danceability"]
        self.valence = audio_feat_dict["valence"]

    # Needed to access Spotify App
    def get_token(self):
        auth_string = self.client_id + ":" + self.client_secret
        auth_bytes = auth_string.encode("utf-8")
        auth_base64 = str(base64.b64encode(auth_bytes), "utf-8")

        url = "https://accounts.spotify.com/api/token"
        headers = { 
            "Authorization": "Basic " + auth_base64,
            "Content-Type": "application/x-www-form-urlencoded"
        }
        data = {"grant_type": "client_credentials"}
        result = post(url, headers=headers, data=data)
        json_result = json.loads(result.content)
        token = json_result["access_token"]
        return token
    
    # Needed to Access Spotify App
    def get_auth_headers(self,token):
        return {"Authorization": "Bearer " + token}
    
    def publish_body_cmds(self, value):
        message = body()
        message.tempo = self.tempo
        # Using Spotify Data
        if value == True:
            message.move_name = self.lights_move
        # Auto Mode
        else:
            message.move_name = ""
        self.bodyPub.publish(message)
        rospy.sleep(0.05)

    def publish_lights_cmd(self, value):
        message = lights()
        message.tempo = self.tempo
        # Using Spotify Data
        if value == True:
            message.move_name = self.body_dance_move
        # Auto Mode
        else:
            message.move_name = ""

        self.lightsPub.publish(message)
        rospy.sleep(0.05)

    def publish_head_cmd(self, value,num_of_joints):
        message = head()
        message.tempo = self.tempo
        message.num_of_joints = num_of_joints

        # Using Spotify Data
        if value == True:
            message.move_name = self.head_dance_move
        # Auto Mode
        else:
            message.move_name = ""


        self.headPub.publish(message)
        rospy.sleep(0.05)

    def change_moves_around(self):
        # Base on genres: pop, rock, blues, metal, 
        # For Head Dance Move
        if self.genre == "pop":
            dances_for_genre = [0,2,3]
            index = random.randint(0,len(dances_for_genre))
            self.head_dance_move = self.head_move_names[index]

        if self.genre == "soul":
            dances_for_genre = [0,3]
            index = random.randint(0,len(dances_for_genre))
            self.head_dance_move = self.head_move_names[index]

        if self.genre == "metal":
            dances_for_genre = [1]
            index = random.randint(0,len(dances_for_genre))
            self.head_dance_move = self.head_move_names[index]
        else:
            #Any move
            index = random.randint(0,3)
            self.head_dance_move = self.head_move_names[index]

    # MAIN PROGRAM LOOP 
    # ALL NEEDS REWORKING FOR DANCE_MODE 
    def loop(self):
        # Identify Song
        print("Play Music Now")
        
        # Service stuff, get tempo and last beat (BOTH NEED)#
        # Synchronisation stuff mostly, tempo needed if using auto mode
        
        # UNCOMMENT FOR LISTEN AND RECORD
        rospy.sleep(1)
        tSongStart = rospy.get_time()
        response_listen_and_record = self.service_record(self.request_to_record)
        #response_est_tempo = self.service_tempo(self.request_for_tempo)
        #tempo_and_last_beat = response_est_tempo.message.split()
        #self.tempo = round(float(tempo_and_last_beat[0]),2)
        #self.last_beat = round(float(tempo_and_last_beat[1]),2)
        # Identify song name 
        # ONLY FOR SPOTIFY 
        if self.dance_mode == "Spotify":
            while self.song_name == "":
                print("Plug in phone now, you have 10 seconds")
                #rospy.sleep(10)
                response_song_identification = self.service_identify(self.request_to_identify) 
                print(response_song_identification.message)
                if response_song_identification.message != "":
                    self.song_name = response_song_identification.message
                else:
                    print("unplug phone, cont playing music")
                    rospy.sleep(5)
                    response_listen_and_record = self.service_record(self.request_to_record)
            self.set_track_data()
        print("song found")

        rospy.sleep(5)
        # Test stuff -----
        #self.song_name == "ARound the world - Daft Punk"
        #self.set_track_data()
        #self.tempo = 120
        # ENd of test stuff ---

        beat_len = 60 / self.tempo
        # avg song length in beats
        avg_song_len  = 180/beat_len 
        beats_array = [beat_len*16,beat_len*32,beat_len*48,beat_len*64,avg_song_len]
        print("WE MADE IT!!!!!!")
        print(self.song_name)
        print(self.tempo)
        print(self.sections)
        start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            # Two Scenarios 
            if self.dance_mode == "Auto":
                self.publish_lights_cmd(False)
                self.publish_body_cmds(False)
                self.publish_head_cmd(False,6)
                rospy.sleep(0.02)


                # Gives the impression that the dancing
                # is building as song progresses
                # Might be too much
                #for x in range(0,len(beats_array)):
                #    current_time = rospy.get_time()-start_time
                #    while current_time < beats_array[x]:

                        #if x == 0:
                        #    self.publish_head_cmd(False,2)
                        #elif x == 1:
                        #    self.publish_head_cmd(False,4)
                        #    self.publish_lights_cmd(False)
                        #elif x == 2:
                        #    self.publish_head_cmd(False,6)
                        #    self.publish_lights_cmd(False)
                        #elif x == 3:
                        #    self.publish_head_cmd(False,6)
                        #    self.publish_lights_cmd(False)
                        #    self.publish_body_cmds(False)

                        #current_time = rospy.get_time()-start_time
                
                #print("Song Over")

            autoMode = False
            tMovesStart = rospy.get_time()
            print(tMovesStart)
            print(tSongStart)
            if self.dance_mode == "Spotify":
                for x in range(0,len(self.sections)):
                    current_time = rospy.get_time()-start_time + (tMovesStart -tSongStart)
                    while current_time < self.sections[x]: 
                        #print(autoMode)
                        self.publish_body_cmds(False)
                        self.publish_head_cmd(False,6)
                        self.publish_lights_cmd(False)
                        rospy.sleep(0.1)
                        
                    # Alternates between the auto kind of dancing
                    # and specific dance moves 
                    autoMode = not autoMode
                    # only changes the dance moves if they will be used 
                    # next iteration
                    if autoMode == False:
                        print("swapping moves")
                        self.change_moves_around()
                        print(self.head_dance_move)
        
if __name__ == "__main__":
    rospy.init_node("dance_MiRo",anonymous=True)
    args = rospy.myargv()
    dance_mode = str(args[1])
    #print(f"#{dance_mode}#")
    if dance_mode == "Spotify":
        print("Spotify Mode")
    elif dance_mode == "Auto":
        print("auto mode")
    else: 
        print("No valid input, going into auto mode")
        dance_mode = "Auto"
    main = MiroDance(dance_mode)
    main.loop()




# CODE FOR MAKING this node into SERVICE_CLIENT for identify_song
#        service_name = "identify_song"
#        rospy.init_node(f"{service_name}_client") 

#        rospy.wait_for_service(service_name) 
#        service = rospy.ServiceProxy(service_name, SetString)

#        request_to_server = SetStringResponse() 
#        request_to_server.request_signal = True 

#        response_from_server = service(request_to_server) 
# SHOULD RETURN THE SONG NAME SO FOR NOW I'LL JUST INSERT ONE
#print(response_from_server)


# OLD LOOP 
            # Calls identify_song service
            #response_from_server = self.service(self.request_to_server) 
"""
if self.song_name != "No":
    #print("okay")
    for x in range(0,len(self.sections)):
        current_time = rospy.get_time()-start_time
        while current_time < self.sections[x]: 

            #  HERE uses self.genre to choose a dancemove to be performed and or a light setting.
            self.publish_body_cmds(autoMode)
            self.publish_head_cmd(autoMode)
            self.publish_lights_cmd(autoMode)
            rospy.sleep(0.5)
            current_time = rospy.get_time()-start_time
        autoMode = not autoMode
rospy.sleep(0.5)
"""