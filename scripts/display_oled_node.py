#!/usr/bin/env python

"""

Particularly, this code initializes the OLED SSD1306 and updates the screen 
based on received ROS messages.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

"""

import rospy
import modules.oled_display as display

# Import ROS msgs
from std_msgs.msg import UInt8
from ema_common_msgs.msg import Stimulator

# Import utilities
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

# Other imports
import time

# Global variables
global oledi2c
global current

# Configure display OLED 
oledi2c = display.SSD1306_128_64(rst=None)

# initialize lcd display
oledi2c.begin()
oledi2c.clear()
oledi2c.display()

# initial width and height display
width = oledi2c.width
height = oledi2c.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Position definition
padding = 4
top = padding
bottom = height-padding
x = 0
# Load font.
fontsize = 26 
font = ImageFont.truetype('./resources/FreeMono.ttf', fontsize)

# set initial value
current =-1

def callback(data):
    global oledi2c
    global current

    draw.text((x, top),"Corrente", font=font, fill=255)
    draw.text((x+22, top+28),"%2d mA" %data.data, font=font, fill=255)

    # check current value and update display
    if data.data != current:
        # Draw a black filled box to clear the image.
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, top),"Corrente", font=font, fill=255)
        draw.text((x+22, top+28),"%2d mA" %data.data, font=font, fill=255)
        oledi2c.image(image)
        oledi2c.display()
        time.sleep(.1)
        current = data.data
        
def main():
    # init display node
    rospy.init_node('display', anonymous=True)
    
    # subscribe to main current topic from control node
    sub = rospy.Subscriber("display/update", UInt8, callback = callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
