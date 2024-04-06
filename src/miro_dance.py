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
        
        #self.genre_list = []   Dont think I'll need
        self.genre = ""
        self.duration = 0.0 
        self.tempo = 0.0
        self.beat_len = 0.0
        self.end_of_fade_in = 0.0
        self.track_start = 0.0
        self.music_start_time = 0
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

        # Audio Features 
        self.danceability = 0.0
        self.valence = 0.0 

        # Required for device to access Spotify API app 
        self.client_id = "acbd6c4e089e4c9cb071ce9d3e4a9583"
        self.client_secret = "a35830533528441f9ae304893a279b38"
        
        # SERVICES
        ## LOCALISE SOUND SERVICE
        service_name = "point_to_sound"

        rospy.wait_for_service(service_name)
        self.service_localise_MiRo = rospy.ServiceProxy(service_name, SetBool)
        self.request_to_localise = True

        print("yellow")
        # RECORD MIRO AUDIO SERVICE
        service_name = "listen_and_record_music"
        rospy.wait_for_service(service_name)
        self.service_record = rospy.ServiceProxy(service_name,SetBool)
        
        self.request_to_record = SetBoolRequest()
        self.request_to_record.data = True
        #print("Yellow")
        
        # ESTIMATE TEMPO SERVICE
        service_name = "estimate_tempo_and_beats"
        rospy.wait_for_service(service_name)
        self.service_tempo = rospy.ServiceProxy(service_name, SetBool)
        
        self.request_for_tempo = SetBoolRequest()
        self.request_for_tempo.data = True 
        #print("yellow")
        # IDENTIFY SONG SHAZAM SERVICE
        if self.dance_mode == "Spotify":
            service_name = "identify_song"

            rospy.wait_for_service(service_name) 
            self.service_identify = rospy.ServiceProxy(service_name, SetBool)

            self.request_to_identify = SetBoolRequest()
            self.request_to_identify.data = True 
        
        # PUBLISHERS
        # body_msg : body movements
        topic_name = "body_topic"
        self.bodyPub = rospy.Publisher(topic_name, body, queue_size=10)

        topic_name = "head_topic"
        self.headPub = rospy.Publisher(topic_name, head, queue_size=10)

        topic_name = "light_topic"
        self.lightsPub = rospy.Publisher(topic_name, lights, queue_size=10)

        # localise_now : point_to_responsesound msg
        rospy.sleep(1)
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
        genre_list = json_result["genres"]
        self.genre = self.decide_genre(genre_list)

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
            message.move_name = self.head_dance_move
        # Auto Mode
        else:
            message.move_name = ""
        self.bodyPub.publish(message)
        rospy.sleep(0.05)

    def publish_lights_cmd(self, value):
        message = lights()
        message.tempo = self.tempo
        # Using Spotify Data
        message.move_name = self.genre

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

    def stop_all_joints_cmd(self):
        head_message = head()
        lights_message = lights()
        body_message = body()
        head_message.move_name = "done"
        body_message.move_name = "done"
        lights_message.move_name = "done"
        
        self.lightsPub.publish(lights_message)
        self.bodyPub.publish(body_message)
        self.headPub.publish(head_message)

    def change_moves_around(self):
        # Base on genres: pop, rock, blues, metal, 
        # For Head Dance Move
        if self.genre == "pop":
            print("pop picked")
            dances_for_genre = [0,1,2,3]
            index = random.randint(0,len(dances_for_genre)-1)
            self.head_dance_move = self.head_move_names[index]

        if self.genre == "soul":
            print("soul picked")
            dances_for_genre = [0,3]
            index = random.randint(0,len(dances_for_genre)-1)
            self.head_dance_move = self.head_move_names[index]

        if self.genre == "electronic":
            print("metal picked")
            dances_for_genre = [1]
            index = random.randint(0,len(dances_for_genre)-1)
            self.head_dance_move = self.head_move_names[index]
        else:
            print("random picked")
            #Any move
            index = random.randint(0,3)
            self.head_dance_move = self.head_move_names[index]

    # Calculates how long to wait before publishing in order for commands to be in sync with the song beat
    def length_to_wait(self, length_into_song):
        beat_to_join = self.last_beat + ( 8 * self.beat_len)

        while beat_to_join < length_into_song + 0.1:
            beat_to_join = beat_to_join + ( 8 * self.beat_len)
        
        # Takes about 0.15 seconds to publish commands so send publish 0.1 seconds before actual beat
        length_to_sleep = beat_to_join - length_into_song - 0.1
        return length_to_sleep 

    def decide_genre(self, list_of_genres):
        genre = ""
        for genre_name in list_of_genres:
            if "soul" in genre_name:
                genre = "soul"
            if "pop" in genre_name:
                genre = "pop"
            if "rock" in genre_name:
                genre = "rock"
            if "metal" in genre_name:
                genre = "metal"
            if "blues" in genre_name:
                genre = "blue"
            if "classical" in genre_name:
                genre = "classical"
            if "electr" in genre_name:
                genre = "electronic"
            # If a genre has been found, doesn't need to iterate through the other genres
            if genre != "":
                break
        return genre

    def pre_dance_processes(self):
        # Identify Song
        print("Play Music Now")
        message = lights()
        message.move_name = "localising"
        self.lightsPub.publish(message)
        # SEND LIGHT COMMAND THAT SHOWS HE IS LOCALISING
        # Calls service that gets MiRo to face the source of the sound 
        response_point_to_sound = self.service_localise_MiRo(self.request_to_localise)
        self.music_start_time = float(response_point_to_sound.message)
        message = lights()
        message.move_name = "listening"
        self.lightsPub.publish(message)
        # Calls service that records the MiRo's microphones when music is playing and
        # saves it to data/miro_audio.mp3
        response_listen_and_record = self.service_record(self.request_to_record)
        if self.music_start_time == 0.0:
            self.music_start_time = float(response_listen_and_record.message)

        message.move_name = "processing"
        self.lightsPub.publish(message)
        # Calls service that estimates tempo (needed for auto mode) and the time 
        # of the last beat in the recording (needed for synchronisation)
        response_est_tempo = self.service_tempo(self.request_for_tempo)
        tempo_and_last_beat = response_est_tempo.message.split()
        #tempo_and_last_beat = 0 

        # if dance_mode is Spotify, it will retrieve the tempo that way and 
        # likely be more accurate
        if self.dance_mode == "Auto":
            self.tempo = int(float(tempo_and_last_beat[0]))
            
            self.beat_len = 60 / self.tempo

            # Assumes the avg song is 3 minutes long
            avg_song_len  = 180/self.beat_len 
            self.sections = [self.beat_len*16,self.beat_len*32,self.beat_len*48,self.beat_len*64,avg_song_len]
            print("here??")

        #self.last_beat = round(float(tempo_and_last_beat[1]),2)
        
        # Calls service that uses shazam to identify the song name
        # and then calls methods that access Spotify API to retrieve
        # audio analysis (tempo, beats, sections etc)
        if self.dance_mode == "Spotify":

            # Loops until shazam finds the song, may require multiple recordings for shazam to find it 
            while self.song_name == "":
                print("Plug in phone now, you have 10 seconds")
                response_song_identification = self.service_identify(self.request_to_identify) 
                print(response_song_identification.message)
                if response_song_identification.message != "":
                    self.song_name = response_song_identification.message
                    print("song found")
                else:
                    print("unplug phone, cont playing music")
                    rospy.sleep(5)
                    response_listen_and_record = self.service_record(self.request_to_record)

            # Calls Spotify only once the song name has definitely been found
            self.set_track_data()
            self.beat_len = 60 / self.tempo
        
        # PUBLISH STOP LIGHTS COMMAND
        message.move_name = "done"
        self.lightsPub.publish(message)
        rospy.sleep(2)

    # MAIN PROGRAM LOOP 
    def loop(self):
        # Enter Dancing State
        while not rospy.is_shutdown():
            head_message = head()
            head_message.move_name = "reset"
            self.headPub.publish(head_message)

            self.pre_dance_processes()
            # Test stuff -----
            #self.song_name = "Deja Vu Beyonce"
            #print("Setting Track Data PLUGGG!!!!")
            #rospy.sleep(5)
            #self.set_track_data()
            self.music_start_time = rospy.get_time()
            #self.sections = [0,10,20,30,40,50,60,70,80]
            #self.beat_len = 60 / self.tempo
            self.tempo = 76
            # End of test stuff ---

            #dance_start_time = float(rospy.get_time())
            #how_far_into_song = dance_start_time  + 5 - self.music_start_time
            #length_to_wait = self.length_to_wait(how_far_into_song)
            #print(length_to_wait)
            #rospy.sleep(length_to_wait)
            #print(f"the dance moves started {how_far_into_song} seconds after the song started")
            
            # Two Scenarios   
            # s        
            if self.dance_mode == "Auto":
                current_time = rospy.get_time() - self.music_start_time
                print(current_time)
                # Assumes the song is 2 minutes long so the dancing will stop 
                while current_time < 15:
                    self.publish_lights_cmd(False)
                    #self.publish_body_cmds(False)
                    self.publish_head_cmd(False,6) 
                    current_time = rospy.get_time() - self.music_start_time
                    rospy.sleep(0.02)

            if self.dance_mode == "Spotify":    
                autoMode = True
                for x in range(0,len(self.sections)):
                    current_time = rospy.get_time()-self.music_start_time
                    while current_time < self.sections[x]: 
                        self.publish_body_cmds(False)
                        self.publish_head_cmd(autoMode,6)
                        self.publish_lights_cmd(autoMode)
                        current_time = rospy.get_time()-self.music_start_time
                        rospy.sleep(0.02)
                        
                    # Alternates between the auto kind of dancing
                    # and specific dance moves 
                    autoMode = not autoMode
                    # only changes the dance moves if they will be used 
                    # next iteration
                    print(self.sections[x])
                    if autoMode == False:
                        print("swapping moves")
                        self.change_moves_around()
                        print(self.head_dance_move)
                
            # Song ended
            self.stop_all_joints_cmd()
            print("The current song has ended")
            print("--------------------------")
            print("Restarting for next song...")
            rospy.sleep(10)
            # Reset any values that need to be reset before doing another song
            self.music_start_time = 0 

        
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
    rospy.sleep(1)
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