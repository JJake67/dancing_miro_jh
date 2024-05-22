#!/usr/bin/env python3

import rospy
import os
import asyncio
import time
import rospkg
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from shazamio import Shazam

class identifySongService():

  def __init__(self):
    service_name = "identify_song"
    rospy.init_node(f"{service_name}_server")
    self.song_title = ""
    self.service = rospy.Service(service_name, SetBool, self.srv_callback)
    
    rospy.loginfo(f"{service_name} Server is ready to be called")

  def srv_callback(self, request_from_client):
    #directory = os.getcwd()
    rospack = rospkg.RosPack()
    path = rospack.get_path('dancing_miro_jh')
    #print(path)
    #print("plz")
    response_from_server = SetBoolResponse()
    print("")
    print("Connect to your hotspot now, you have TEN seconds!")
    #self.listenForSongPub.publish(True)
    #rospy.sleep(10)
    if request_from_client.data == True:
      #print("Server received True, will attempt song identification")
      async def findSong():
        shazam = Shazam()
        # Waits until shazam identifies song
        out = await shazam.recognize(path+"/data/miro_audio.ogg")
        #print(out)
        if len(out["matches"]) == 0 :
          self.song_title = ""
          print("No Matches Found")
        else:
          self.song_title = out["track"]["title"]
      #print("ye")
      loop = asyncio.new_event_loop()
      #print("yeee")
      loop.run_until_complete(findSong())
      #print("yeeeeee")
      response_from_server.success = True
      response_from_server.message = self.song_title
    else:
      response_from_server.success = False
      response_from_server.message = "No Request"
    #self.listenForSongPub.publish(False)
    return response_from_server

  def main(self):
    rospy.spin()

if __name__ == '__main__':
  server = identifySongService()
  server.main()