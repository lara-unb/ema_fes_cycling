#!/usr/bin/env python

"""

Particularly, this code initializes the LCD display and updates the screen 
based on received ROS messages.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

"""

import rospy
import modules.lcd_display as display

# Import ROS msgs
from std_msgs.msg import UInt8
from ema_common_msgs.msg import Stimulator

# Global variables
global lcdi2c
global current

# initialize lcd display
lcdi2c = display.LCD()

# set initial value
current =-1

def callback(data):
    global lcdi2c
    global current

    # check current value and update display
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
