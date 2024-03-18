#!/usr/bin/env python3
import os
#import rospy            # ROS Python interface
#from std_msgs.msg import UInt32MultiArray
#from dancing_miro.msg import lights

class IllumPublisher(object):
    
    def __init__(self):
        rospy.init_node("illumination_publisher")
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Parameters to be passed by Miro_Dance
        # Used to sync to the music 
        self.tempo = 0
        # Used to denote what type of lights to do
        self.command = ""

        # Subscribers and Publishers
        self.illumination = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )
        self.sub = rospy.Subscriber("light_topic", lights,self.cmd_callback)

    # Callback for when parameters are passed from Miro_Dance Node 
    def cmd_callback(self,topic_msg):
        print(f'Node obtained msg: {topic_msg.move_name}')
        self.command = topic_msg.move_name
        self.tempo = 30/topic_msg.tempo

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
                return [255,0,0]
        elif colour_name == "black":
                return [255,0,255]
        elif colour_name == "white":
                return [255,255,255]
        else :
             # Check if this is no light or black lol 
             return [0,0,0]

    def set_all_lights(self, colour):
        colour_rgb = self.get_rgbs(colour)
        color_change = UInt32MultiArray()
        color_change.data = [
            colour_rgb,
            colour_rgb,
            colour_rgb,
            colour_rgb,
            colour_rgb,
            colour_rgb
        ]
        self.illumination.publish(color_change)
        rospy.sleep(0.02)

    def transition_lights(self, colour1, colour2, transition_length):
        # MAX TRANSITION LENGTH = 50 
        colour1_rgb = self.get_rgbs(colour1)
        colour2_rgb = self.get_rgbs(colour2)

        color_change = UInt32MultiArray()
        
        step_time = 0.2    #secs
        no_of_steps = int(transition_length / step_time)

        r_step_amt = (colour2_rgb[0]-colour1_rgb[0]) / no_of_steps
        g_step_amt = (colour2_rgb[1]-colour1_rgb[1]) / no_of_steps
        b_step_amt = (colour2_rgb[2]-colour1_rgb[2]) / no_of_steps 

        for x in range(no_of_steps+1):
            new_r = int(colour1_rgb[0] + (r_step_amt*x))
            new_g = int(colour1_rgb[1] + (g_step_amt*x))
            new_b = int(colour1_rgb[2] + (b_step_amt*x))
            #print(f" at step {x} the new rgb is R:{new_r}, G:{new_g}, B:{new_b}")

            color_change.data = [(new_r,new_g,new_b)] * 6
            self.illumination.publish(color_change)
            rospy.sleep(0.02)

    def flashing_lights(self, colour1, colour2, flash_length):
        colour1_rgb = self.get_rgbs(colour1)
        colour2_rgb = self.get_rgbs(colour1)
        color_change = UInt32MultiArray()
        cur_command = self.command
        flash = True
        while self.command == cur_command:
            if flash:
                color_change.data = [colour1_rgb]
            else:
                color_change.data = [colour2_rgb]
            self.illumination.publish(color_change)
            rospy.sleep(flash_length)
            flash = not flash
    
    # FUNCTIONS NEED TO MAKE
    # Searching for Song Lights 
    # Genre Specific Lights
    
    def loop(self):
        if self.command == "all_lights":
              self.set_all_lights("red")
        if self.command == "flashing lights":
              self.flashing_lights("red","blue",2)
        if self.command == "transition":
              self.transition_lights("red","blue", 3)
              self.transition_lights("blue","red", 3)


        """ OLD LOOP, KEEP COZ I HAVE ATTACHMENT ISSUES
        if self.command == "blue":
            blue = [51,204,70]
            illum.blue(blue)
            #illum.blue(blue)
        if self.command == "rainbow":
            colours = [purple,blue,green,red,orange,yellow]
            #illum.rainbow(colours)
            illum.rainbowLoop()
        """

illum = IllumPublisher()
while not rospy.is_shutdown(): #light up 3 different colors 
    illum.loop()

"""
def set_illumination(self, red = 0, green = 0, blue = 0):
        # changing the rgb format into android format to be published in MiRo message
        color_change = UInt32MultiArray()
        color_detail = (int(red), int(green), int(blue))
        color = '0xFF%02x%02x%02x'%color_detail
        color = int(color, 16)
        # six seperate leds in the miro
        color_change.data = [
            color,
            color,
            color,
            color,
            color,
            color
        ]
        self.illumination.publish(color_change)

def blue(self, blue):
        # changing the rgb format into android format to be published in MiRo message
    color_change = UInt32MultiArray()
    #length = len(colours)
    #color = [0]*6
    #print(colours[0][1])
    count = 0
    blue_detail = (int(blue[0]),int(blue[1]),int(blue[2]))
    blue = '0xFF%02x%02x%02x'%blue_detail
    blue = int(blue, 16)
    #for x in colours:

        #color_detail = (int(x[0]),int(x[1]),int(x[2]))
        #color[count] = '0xFF%02x%02x%02x'%color_detail
        #color[count] = int(color[count], 16)
        #count = count+1
    # six seperate leds in the miro
    color_change.data = [
        blue,
        blue,
        blue,
        blue,
        blue,
        blue
    ]
    self.illumination.publish(color_change)
    rospy.sleep(0.1)

def rainbow(self,colours):
        # changing the rgb format into android format to be published in MiRo message
    color_change = UInt32MultiArray()
    length = len(colours)
    color = [0]*6
    #print(colours[0][1])
    count = 0
    for x in colours:

        color_detail = (int(x[0]),int(x[1]),int(x[2]))
        color[count] = '0xFF%02x%02x%02x'%color_detail
        color[count] = int(color[count], 16)
        count = count+1
    # six seperate leds in the miro
    color_change.data = [
        color[0],
        color[1],
        color[2],
        color[3],
        color[4],
        color[5]
    ]
    self.illumination.publish(color_change)
    rospy.sleep(0.1)

def rainbowLoop(self):
    self.command = "rainbow"
    purple = [255,102,255]
    blue = [0,153,255]
    green = [132,255,152]
    yellow = [255,255,0]
    orange = [255,165,0]
    red = [255,0,0]
    colours = [purple,blue,green,red,orange,yellow]
    increment = 0 
    while self.command == "rainbow":
        if increment == 0:
            colours = [purple,blue,green,red,orange,yellow]
        elif increment == 1:
            colours = [blue,green,red,orange,yellow, purple]
        elif increment == 2:
            colours = [green,red,orange,yellow,purple,blue]
        elif increment == 3:
            colours = [red,orange,yellow,purple,blue,green]
        elif increment == 4:
            colours = [orange,yellow,purple,blue,green,red]
        elif increment == 5:
            colours = [yellow,purple,blue,green,red,orange]
            increment = 0 
        illum.rainbow(colours)
        increment = increment + 1
        rospy.sleep(self.tempo)
        
def OppRainbow(self):
        # changing the rgb format into android format to be published in MiRo message
    purple = [255,102,255]
    blue = [0,153,255]
    green = [132,255,152]
    yellow = [255,255,0]
    orange = [255,165,0]
    red = [255,0,0]
    colours = [red,orange,yellow,purple,blue,green]
    color_change = UInt32MultiArray()
    length = len(colours)
    color = [0]*6
    #print(colours[0][1])
    count = 0
    for x in colours:

        color_detail = (int(x[0]),int(x[1]),int(x[2]))
        color[count] = '0xFF%02x%02x%02x'%color_detail
        color[count] = int(color[count], 16)
        count = count+1
    # six seperate leds in the miro
    color_change.data = [
        color[0],
        color[1],
        color[2],
        color[3],
        color[4],
        color[5]
    ]
    self.illumination.publish(color_change)

"""