import random
import numpy as np



def sine_generator(self, mx=1, mn=0, offset=0, freq=1, phase=0, t=1.0, t0=0):
    return ((mx-mn) * np.sin (freq*(t-t0) + phase) / 2.0 + offset)

def new_sine_generator(self,mx=1, mn=0, freq=1, phase=0 ,t=1.0 ,t0=0):
    return ((mx-mn)) * (freq/math.pi)* (np.sin(freq*(t-t0) + phase) / 2) + (mx - ((mx-mn)/2)) 

freq = 
head_yaw = sine_generator(20,10,freq,0,5,2)
new_head_yaw = new_sine_generator(20,10,freq,0,5,2)

print(head_yaw)
print(new_head_yaw)
"""
list_of_genres = ["electaronic","pop"]
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
        genre = "blues"
    if "classical" in genre_name:
        genre = "classical"
    if "electr" in genre_name:
        genre = "electr"
    # If a genre has been found, doesn't need to iterate through the other genres
    if genre != "":
        break

print(genre)
print(len(list_of_genres))


head_move_names = ["head_bounce","head_bang","full_head_spin","head_bop"]
print("pop picked")
dances_for_genre = [0,1,2,3]
index = random.randint(0,len(dances_for_genre))
head_dance_move = head_move_names[index]
for x in range(0,10):
    index = random.randint(0,len(dances_for_genre)-1)
    head_dance_move = head_move_names[index]
    print(index)
    print(head_dance_move)
"""