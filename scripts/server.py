#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ema_fes_cycling.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {angle_left_min}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("ema_fes_cycling")

    srv = Server(TutorialsConfig, callback)
    rospy.spin()
