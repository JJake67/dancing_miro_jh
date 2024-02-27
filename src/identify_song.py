import os
import asyncio
from shazamio import Shazam

print('Get current working directory : ', os.getcwd())
directory = os.getcwd()
def main():
  shazam = Shazam()
  # Waits until shazam identifies song
  out = shazam.recognize_song(directory+"\data\white_noise_5secs.mp3")
  if len(out["matches"]) == 0 :
    print("soz")
  else:
    print(out["track"]["title"])

#loop = asyncio.get_event_loop()
#loop.run_until_complete(main())
main = main()
main.loop()
