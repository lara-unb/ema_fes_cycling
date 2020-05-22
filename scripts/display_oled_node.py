#!/usr/bin/env python

import rospy
import time
import ema.modules.OLED_SSD1306 as disp_module

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

# import ema common msgs
from ema_common_msgs.msg import Stimulator

# import ros msgs
from std_msgs.msg import UInt8

# define global values
global oledi2c
global current

# Configure display OLED 
oledi2c = disp_module.SSD1306_128_64(rst=None)

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
font = ImageFont.truetype('displayfont/FreeMono.ttf', fontsize)

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
