#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ema_fes_cycling.cfg import DynamicParamsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {current_left}, {current_right}, {angle_left_min}, \
    	{angle_left_max}, {angle_right_min}, {angle_right_max}, {shift},""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()
