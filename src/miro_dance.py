#!/usr/bin/env python3
import rospy
import os
import base64
import json
from dotenv import load_dotenv
from requests import post, get
# from msg_place import SetString, SetStringResponse
class MiroDance(object):

    # Main Node for Creating Dance Movements

    def __init__(self):
        # Func Initialisations
        # Empty 2D arrays for song data
        self.genre = ""
        self.duration = 0.0 
        self.tempo = 0.0
        self.end_of_fade_in = 0.0
        self.track_start = 0.0
        
        self.bars_array = []
        self.sections = [] 

        # Audio Features 
        #   Valence = Estimate of how positive the song is 
        self.danceability = 0.0
        self.valence = 0.0 

        # Required for device to access Spotify API app 
        self.client_id = os.getenv("CLIENT_ID")
        self.client_secret = os.getenv("CLIENT_SECRET")
        
        self.set_track_data()
        # SUBSCRIBERS

        # PUBLISHERS
        # localise_now : point_to_sound msg
        # body_msg : body movements
        # light_msg : lights commands
        # head_msg : head and neck commands
        
        #NEEDS ALL THE SPOTIFY STUFF TO START

    # SPOTIFY 
    def set_track_data(self):

        # Finds Song on Spotify 
        url = "https://api.spotify.com/v1/search"
        token = self.get_token()
        headers = self.get_auth_headers(token)

        query = f"?q={song_name}&type=track&limit=1"
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
        self.tempo = audio_analysis_dict["track"]["tempo"]
        self.end_of_fade_in = audio_analysis_dict["track"]["end_of_fade_in"]

        # Finds the time the first beat starts so each one after that will be in time
        self.track_start = audio_analysis_dict["beats"][0]["start"]

        self.bars = audio_analysis_dict["bars"]
        self.sections = audio_analysis_dict["sections"]

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
    def get_auth_headers(token):
        return {"Authorization": "Bearer " + token}
    
    # MAIN PROGRAM LOOP 
    def loop(self):
        #Get Spotify Data For Song 
        self.set_track_data()
        
song_name = "Smooth Santana"
load_dotenv()
if __name__ == "__main__":
    rospy.init_node("dance_MiRo",anonymous=True)
    main = MiroDance()
    main.loop()


# CODE FOR MAKING this node into SERVICE_CLIENT for identify_song
#service_name = "identify_song"
#rospy.init_node(f"{service_name}_client") 

#rospy.wait_for_service(service_name) 
#service = rospy.ServiceProxy(service_name, SetString)

#request_to_server = SetStringResponse() 
#request_to_server.request_signal = True 

#response_from_server = service(request_to_server) 
# SHOULD RETURN THE SONG NAME SO FOR NOW I'LL JUST INSERT ONE
#print(response_from_server)