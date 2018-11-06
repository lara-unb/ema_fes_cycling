#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ema_fes_cycling.cfg import DynamicParamsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {angle_left_min}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()
