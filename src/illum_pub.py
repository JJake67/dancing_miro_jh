#!/usr/bin/env python3
import os
import rospy            # ROS Python interface
import random
from std_msgs.msg import UInt32MultiArray
from dancing_miro_jh.msg import lights

class IllumPublisher(object):
    
    def __init__(self):
        rospy.init_node("illumination_publisher")
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Parameters to be passed by Miro_Dance
        # Used to sync to the music 
        self.tempo = 0.0
        self.t_4bars = 0.0
        # Used to denote what type of lights to do
        self.genre = ""
        self.flash = True
        self.colours = []
        self.color1 = "red"
        self.color2 = "blue"
        self.colours = ["red","blue","orange","yellow","purple","green"]
        self.preset = False
        self.next_lights = 0

        # Subscribers and Publishers
        self.illumination = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )
        self.sub = rospy.Subscriber("light_topic", lights,self.cmd_callback)

        rospy.loginfo("Lights Node is active...")

    # Callback for when parameters are passed from Miro_Dance Node 
    def cmd_callback(self,topic_msg):
        #print(f'Node obtained msg: {topic_msg.move_name}')
        
        # Check if the genre has changed, if so needs to update the 
        # colours that will be used
        if topic_msg.move_name == "localising" or topic_msg.move_name == "recording" or topic_msg.move_name == "processing" or topic_msg.move_name == "listening" or topic_msg.move_name == "done":
            self.genre = topic_msg.move_name

        elif self.genre != topic_msg.move_name:

            self.genre = topic_msg.move_name
            self.set_colours_by_genre()
            self.tempo = 60/topic_msg.tempo

    def set_colours_by_genre(self):
        if self.genre == "pop":
            self.colours = ["red","pink","purple","yellow","orange","white"]
        elif self.genre == "soul":
            self.colours = ["purple","blue","green"]
        elif self.genre == "electronic":
            self.colours = ["blue","red","pink"]
        elif self.genre == "rock":
            self.colours = ["black","blue",]
        elif self.genre == "blues":
            self.colours = ["blue","blue"]
        elif self.genre == "metal":
            self.colours = ["black","white"]
        elif self.genre == "classical":
            self.colours = ["yellow","orange","white"]
        else:
            self.colours =["red","blue","orange","yellow","purple","green"]

    def get_rgbs(self, colour_name):
        if colour_name == "red":
                return [255,0,0]
        elif colour_name == "orange":
                return [255,165,0]
        elif colour_name == "yellow":
                return [255,255,0]
        elif colour_name == "green":
                return [0,255,64]
        elif colour_name == "blue":
                return [0,153,255]
        elif colour_name == "purple":
                return [255,102,255]
        elif colour_name == "pink":
                return [255,105,180]
        elif colour_name == "black":
                return [0,0,0]
        elif colour_name == "white":
                return [255,255,255]
        elif colour_name == "black":
                return [0,0,0]
        else :
             # Check if this is no light or black lol 
             return [0,0,0]

    def set_all_lights(self, colour):
        colour_rgb = self.get_rgbs(colour)
        color_change = UInt32MultiArray()

        color = self.cvt_to_unsigned_int(colour_rgb) 

        color_change.data = [
            color,
            color,
            color,
            color,
            color,
            color
        ]
        self.illumination.publish(color_change)
        rospy.sleep(0.02)

    def cvt_to_unsigned_int(self,colour_rgb):
        color_detail = (int(colour_rgb[0]), int(colour_rgb[1]), int(colour_rgb[2]))
        color = '0xFF%02x%02x%02x'%color_detail
        color = int(color, 16)
        return color 

    def transition_lights(self, colour1, colour2, transition_length):
        # MAX TRANSITION LENGTH = 50 
        colour1_rgb = self.get_rgbs(colour1)
        colour2_rgb = self.get_rgbs(colour2)

        color_change = UInt32MultiArray()
        
        step_time = 0.1    #secs
        no_of_steps = int(transition_length / step_time)

        r_step_amt = (colour2_rgb[0]-colour1_rgb[0]) / no_of_steps
        g_step_amt = (colour2_rgb[1]-colour1_rgb[1]) / no_of_steps
        b_step_amt = (colour2_rgb[2]-colour1_rgb[2]) / no_of_steps 

        for x in range(no_of_steps+1):
            new_r = int(colour1_rgb[0] + (r_step_amt*x))
            new_g = int(colour1_rgb[1] + (g_step_amt*x))
            new_b = int(colour1_rgb[2] + (b_step_amt*x))
            #print(f" at step {x} the new rgb is R:{new_r}, G:{new_g}, B:{new_b}")

            new_colour = (new_r,new_g,new_b)
            color = self.cvt_to_unsigned_int(new_colour)

            color_change.data = [color, color, color, color, color, color]
            self.illumination.publish(color_change)
            rospy.sleep(step_time)
        
        #print("done")

    def flashing_lights(self, colour1, colour2, flash_length):
        colour1_rgb = self.get_rgbs(colour1)
        colour2_rgb = self.get_rgbs(colour2)
        color_change = UInt32MultiArray()
        cur_command = self.genre

        if self.flash:
            color = self.cvt_to_unsigned_int(colour1_rgb)
        else:
            color = self.cvt_to_unsigned_int(colour2_rgb)

        color_change.data = [color, color, color, color, color, color]    
        self.illumination.publish(color_change)
        rospy.sleep(flash_length)
        self.flash = not self.flash
    
    # FUNCTIONS NEED TO MAKE
    # Searching for Song Lights 
    # Genre Specific Lights
    def set_new_light_mods(self):
        self.next_lights = random.randint(0,1)
        len_of_colours = len(self.colours) - 1
        color1 = random.randint(0,len_of_colours)
        color2 = random.randint(0,len_of_colours)
        self.color1 = self.colours[color1]
        self.color2 = self.colours[color2]
        
    def loop(self,t,t0):
        if self.genre == "done":
             self.flashing_lights("white","white",0.5)
        if self.genre == "localising":
                self.flashing_lights("green","white",1)
        elif self.genre == "listening":
                self.flashing_lights("red","red",1)
        elif self.genre == "recording":
                self.flashing_lights("orange","pink",0.2)
        elif self.genre == "processing":
                self.transition_lights("blue","yellow",1)
                self.transition_lights("yellow","red",1)
                self.transition_lights("red","blue",1)

        elif self.tempo != 0.0 and self.genre != "done":
            if self.next_lights == 0:
                self.flashing_lights(self.color1,self.color2,self.tempo)
            if self.next_lights == 1:
                self.transition_lights(self.color1,self.color2,self.tempo)
                self.transition_lights(self.color2,self.color1,self.tempo)

            if self.t_4bars <= t:
                self.t_4bars = t + (32*self.tempo)
                self.set_new_light_mods()
        #self.command="all_lights"
        #if self.command == "all_lights":
        #      self.set_all_lights("red")
        #if self.command == "flashing lights":
        #      self.flashing_lights("red","blue",2)
        #if self.command == "transition":
        #      self.transition_lights("red","blue", 3)
        #      self.transition_lights("blue","red", 3)



illum = IllumPublisher()
t0 = rospy.get_time()
while not rospy.is_shutdown():
    t = rospy.get_time()
    illum.loop(t,t0)
