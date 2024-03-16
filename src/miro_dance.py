#!/usr/bin/env python3
import rospy
import os
import base64
import json
import numpy as np
from dotenv import load_dotenv
from requests import post, get
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest
from dancing_miro.msg import body, lights, head
# from msg_place import SetString, SetStringResponse
class MiroDance(object):

    # Main Node for Creating Dance Movements

    def __init__(self):
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

        # Audio Features 
        self.danceability = 0.0
        self.valence = 0.0 

        # Required for device to access Spotify API app 
        self.client_id = "acbd6c4e089e4c9cb071ce9d3e4a9583"
        self.client_secret = "a35830533528441f9ae304893a279b38"
        
        #self.set_track_data()
        # SERVICES
        service_name = "identify_song"

        rospy.wait_for_service(service_name) 
        self.service_identify = rospy.ServiceProxy(service_name, SetBool)

        self.request_to_identify = SetBoolRequest()
        self.request_to_identify.data = True 

        service_name = "listen_and_record_music"

        rospy.wait_for_service(service_name)
        self.service_record = rospy.ServiceProxy(service_name,SetBool)
        
        self.request_to_record = SetBoolRequest()
        self.request_to_record.data = True


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
        #rospy.sleep(5)
        #self.set_track_data()

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
        if value == True:
            message.move_name = "Spin Big"
            message.mode = True
        else:
            message.move_name = "Wait"
            message.mode = True
        self.bodyPub.publish(message)
        rospy.sleep(0.05)

    def publish_lights_cmd(self, value):
        message = lights()
        message.tempo = self.tempo
        if value == True:
            message.move_name = "blue"
        else : 
            message.move_name = "rainbow"

        self.lightsPub.publish(message)
        rospy.sleep(0.05)

    def publish_head_cmd(self, value):
        message = head()
        message.tempo = self.tempo
        if value == True:
            message.move_name = "head bang"
            message.mode = True
        else:
            message.move_name = "soul nod"
            message.mode = True
        self.headPub.publish(message)
        rospy.sleep(0.05)

    # MAIN PROGRAM LOOP 
    def loop(self):
        # Identify Song
        print("Play Music Now")
        

        while self.song_name == "":
            response_listen_and_record = self.service_record(self.request_to_record)
            print("Plug in phone now")
            rospy.sleep(10)
            response_song_identification = self.service_identify(self.request_to_identify) 
            print(response_song_identification.message)
            self.song_name = response_song_identification.message
            print("song_name" + self.song_name)
        # Retrieve Spotify Data, set object values to that data
        self.set_track_data()
        print(self.song_name)
        print("WE MADE IT!!!!!!")
        print(self.sections)
        print(len(self.sections))
        start_time = rospy.get_time()
        autoMode = False
        while not rospy.is_shutdown():
            # Calls identify_song service
            #response_from_server = self.service(self.request_to_server) 
            if self.song_name != "No":
                print("okay")
                for x in range(0,len(self.sections)):
                    current_time = rospy.get_time()-start_time
                    while current_time < self.sections[x]: 

                        #  HERE uses self.genre to choose a dancemove to be performed and or a light setting.
                        #self.publish_body_cmds(autoMode)
                        #self.publish_head_cmd(autoMode)
                        #self.publish_lights_cmd(autoMode)
                        rospy.sleep(0.5)
                        current_time = rospy.get_time()-start_time
                    autoMode = not autoMode
            rospy.sleep(0.5)
        

        
#song_name = "Smooth Santana"
#load_dotenv()
if __name__ == "__main__":
    rospy.init_node("dance_MiRo",anonymous=True)
    main = MiroDance()
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