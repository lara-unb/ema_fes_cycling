#!/usr/bin/env python

import rospy
import ema.modules.display as display

# import ema common msgs
from ema_common_msgs.msg import Stimulator

# import ros msgs
from std_msgs.msg import UInt8

# define global values
global lcdi2c
global current

# initialize lcd display
lcdi2c = display.Display()

# set initial value
current =-1

def callback(data):
    global lcdi2c
    global current

    if data.data != current:
        lcdi2c.lcd_clear()
        lcdi2c.lcd_display_string("Corrente", 1,4)
        lcdi2c.lcd_display_string("%2d mA" %data.data, 2,6)
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
