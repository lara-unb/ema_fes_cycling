#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ema_fes_cycling.cfg import DynamicParamsConfig

global prev_config

# used to compare past values
prev_config = {
	# 'Link': True,
	'Current_Left': 2,
	'Current_Right': 2,
	# 'Angle_Left_Min': 300,
	# 'Angle_Left_Max': 40,
	#'Angle_Right_Min': 120,
	# 'Angle_Right_Max': 220,
	# 'Shift': 35
}

def callback(config, level):
    global prev_config

    # check for changes in current
    diffL = config['Current_Left'] - prev_config['Current_Left']
    diffR = config['Current_Right'] - prev_config['Current_Right']

    # prevents the user from abruptly increasing the current
    if diffL:
        if diffL > 2:
            config['Current_Left'] = prev_config['Current_Left'] + 2
            prev_config['Current_Left'] = config['Current_Left']
        else:
            prev_config['Current_Left'] = config['Current_Left']
        # modify left and right at the same time    
        if config['Link']:
            config['Current_Right'] = config['Current_Left']
            prev_config['Current_Right'] = config['Current_Right']
    		
    elif diffR:
        if diffR > 2:
            config['Current_Right'] = prev_config['Current_Right'] + 2
            prev_config['Current_Right'] = config['Current_Right']
        else:
            prev_config['Current_Right'] = config['Current_Right']
        # modify left and right at the same time    
        if config['Link']:
            config['Current_Left'] = config['Current_Right']
            prev_config['Current_Left'] = config['Current_Left']

    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()
