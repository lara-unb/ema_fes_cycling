#!/usr/bin/env python

import rospy
from ema_common_msgs.srv import Display, DisplayRequest
from std_msgs.msg import UInt8

global display_service

def button_callback(data):
    resp = display_service(message="Teste",line=0,position=3,clear=1)
    resp = display_service(message="%d"%data.data,line=1,position=6,clear=0)

def main():
    global display_service
    rospy.init_node('display_test', anonymous=False)
    rospy.wait_for_service('display/write')
    display_service = rospy.ServiceProxy('display/write', Display)

    buttons_sub=rospy.Subscriber('button/action', UInt8, callback = button_callback)
    
    resp = display_service(message="Teste",line=1,position=3,clear=0)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass