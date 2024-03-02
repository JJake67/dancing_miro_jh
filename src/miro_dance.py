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

        self.client_id = os.getenv("CLIENT_ID")
        self.client_secret = os.getenv("CLIENT_SECRET")
        # SUBSCRIBERS

        # PUBLISHERS
        # localise_now : point_to_sound msg
        # body_msg : body movements
        # light_msg : lights commands
        # head_msg : head and neck commands
        print("nothin")
        #NEEDS ALL THE SPOTIFY STUFF TO START

    # SPOTIFY 
    def set_song_data(self):
        print("data,data,data")
        url = "https://api.spotify.com/v1/search"
        token = self.get_token()
        headers = self.get_auth_headers(token)

        query = f"?q={song_name}&type=track&limit=1"

        query_url = url + query
        result = get(query_url, headers=headers)
        json_result = json.loads(result.content)["tracks"]["items"]
        if len(json_result) == 0:
            print("noor sorryyy")
            return None
        
        # Song Code
        print(json_result[0])
        song_id = json_result[0]

        url = f"https://api.spotify.com/v1/audio-analysis/{song_id}"
        result = get(url, headers=headers)

        #ta daaaa
        data_dict = json.loads(result.content)

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

    def get_auth_headers(token):
        return {"Authorization": "Bearer " + token}
    
    # MAIN PROGRAM LOOP 
    def loop(self):
        #Get Spotify Data For Song 
        self.set_song_data()

        #





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
song_name = "Smooth Santana"
load_dotenv()
if __name__ == "__main__":
    rospy.init_node("dance_MiRo",anonymous=True)
    main = MiroDance()
    main.loop()
