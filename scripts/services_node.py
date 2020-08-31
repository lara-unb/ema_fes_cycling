#!/usr/bin/env python

import rospy
import os
import yaml
import rospkg
from std_srvs.srv import Trigger, TriggerResponse
from ema_common_msgs.srv import SetUInt16, SetUInt16Response

def reboot_request(data):
    print("Reiniciando...")
    os.system("sudo reboot")
    return TriggerResponse(
        success=True,
        message="Rebooting"
    )

def imu_change_request(data):
    rospack = rospkg.RosPack()
    imu_cfg_path=rospack.get_path("yostlabs_3space_imu")+"/config/imu.yaml"
    with open(imu_cfg_path, 'r') as f:
        imu_file = yaml.safe_load(f)
    if data.imu_number > 0 and data.imu_number <= 10:
        imu_file['wireless_id']['pedal'] = data.imu_number
        with open(imu_cfg_path, 'w') as f:
            yaml.safe_dump(imu_file, f)
    return SetUInt16Response(imu_file['wireless_id']['pedal'])

def main():
    rospy.init_node('services') 

    # Create service reboot with response type Trigger and response function reboot_resquest
    reboot_service = rospy.Service('reboot', Trigger, reboot_request)
    imu_change_service = rospy.Service('changeImu',SetUInt16,imu_change_request)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass