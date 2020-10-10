#!/usr/bin/env python

"""

Particularly, this code initializes the LCD display and updates the screen
based on ROS service requests.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

"""

import rospy
import modules.display as display

# import ema common msgs
from ema_common_msgs.msg import Stimulator

# import ros msgs
from std_msgs.msg import UInt8

from ema_common_msgs.srv import Display as disp 
from ema_common_msgs.srv import DisplayResponse

# define global values
global lcdi2c
# global current

# initialize lcd display
lcdi2c = display.Display()
lcdi2c.lcd_display_string("Ligando...", 1,3)
lcdi2c.lcd_display_string("Aguarde", 2,4)
# set initial value
#current =-1
def display_request(req):
    global lcdi2c
    if req.clear:
        lcdi2c.lcd_clear()
    lcdi2c.lcd_display_string(req.message, req.line+1,req.position)
    return DisplayResponse(success=True)
# def callback(data):
#     global lcdi2c
#     global current

#     # check current value and update display
#     if data.data != current:
#         lcdi2c.lcd_clear()
#         lcdi2c.lcd_display_string("Corrente", 1,4)
#         lcdi2c.lcd_display_string("%2d mA" %data.data, 2,6)
#         current = data.data
        
def main():
    # init display node
    rospy.init_node('display', anonymous=False)
    display_service =rospy.Service('display/write', disp, display_request)
    # subscribe to main current topic from control node
    # sub = rospy.Subscriber("display/update", UInt8, callback = callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
