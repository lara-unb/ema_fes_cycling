#!/usr/bin/env python

import rospy
from ema_common_msgs.srv import display, displayRequest

def main():
    rospy.init_node('display_test', anonymous=False)
    rospy.wait_for_service('display/write')
    display_service = rospy.ServiceProxy('display/write', display)
    
    resp = display_service(message="Teste",line=1,position=3,clear=0)
    if resp.success:
        print("Sucesso")
        rospy.signal_shutdown("Success write to display")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass