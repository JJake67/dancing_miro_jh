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
        self.ctrl_c = False
        rospy.init_node("illumination_publisher")
        self.position = None
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.illumination = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )
        
        self.sub = rospy.Subscriber("light_topic", lights,self.cmd_callback)

    def cmd_callback(self,topic_msg):
        print(f'Node obtained msg: {topic_msg.move_name}')

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
    def rainbow(self):
         # changing the rgb format into android format to be published in MiRo message
        purple = [255,102,255]
        blue = [0,153,255]
        green = [132,255,152]
        yellow = [255,255,0]
        orange = [255,165,0]
        red = [255,0,0]
        colours = [purple,blue,green,red,orange,yellow]
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
    
illum = IllumPublisher()
while not rospy.is_shutdown(): #light up 3 different colors 
    #illum.set_illumination(red = 0, green = 200, blue = 200)
    #illum.rainbow()
    illum.OppRainbow()
    #illum.