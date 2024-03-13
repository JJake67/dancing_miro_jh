#!/usr/bin/env python3
import os
import rospy            # ROS Python interface
from std_msgs.msg import UInt32MultiArray
from dancing_miro.msg import lights
class IllumPublisher(object):
    
    """
        The following code will change color
    """
    def __init__(self):
        self.tempo = 0
        self.command = ""
        rospy.init_node("illumination_publisher")
        self.position = None
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.illumination = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )
        
        self.sub = rospy.Subscriber("light_topic", lights,self.cmd_callback)

    def cmd_callback(self,topic_msg):
        print(f'Node obtained msg: {topic_msg.move_name}')
        self.command = topic_msg.move_name
        self.tempo = 30/topic_msg.tempo

    # set color
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

    # FUNCTIONS NEED TO MAKE
    # Searching for Song Lights 
    # Genre Specific Lights
    
    def loop(self):
        purple = [255,102,255]
        blue = [0,153,255]
        green = [31,204,50]
        yellow = [255,255,0]
        orange = [255,165,0]
        red = [255,0,0]
        if self.command == "blue":
            blue = [51,204,70]
            illum.blue(blue)
            #illum.blue(blue)
        if self.command == "rainbow":
            colours = [purple,blue,green,red,orange,yellow]
            #illum.rainbow(colours)
            illum.rainbowLoop()

illum = IllumPublisher()
while not rospy.is_shutdown(): #light up 3 different colors 
    #illum.set_illumination(red = 0, green = 200, blue = 200)
    #illum.rainbow()
    illum.loop()
    #illum.OppRainbow()
    
    #illum.rainbowLoop()