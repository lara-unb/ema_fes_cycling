#!/usr/bin/env python

import rospy
import os
from std_srvs.srv import Trigger, TriggerResponse

def reboot_request(data):
    print("Reiniciando...")
    os.system("sudo reboot")
    return TriggerResponse(
        success=True,
        message="Rebooting"
    )

def main():
    rospy.init_node('services') 

    # Create service reboot with response type Trigger and response function reboot_resquest
    reboot_service = rospy.Service('reboot', Trigger, reboot_request)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass