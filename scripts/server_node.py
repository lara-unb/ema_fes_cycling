#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ema_fes_cycling.cfg import DynamicParamsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request:\n\
Left current: 		{current_left}    \n\
Right current: 		{current_right}	  \n\
Left angle (min): 	{angle_left_min}  \n\
Left angle (max): 	{angle_left_max}  \n\
Right angle (min): 	{angle_right_min} \n\
Right angle (max): 	{angle_right_max} \n\
Shift:			{shift}\n""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()
