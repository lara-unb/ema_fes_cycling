#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ema_fes_cycling.cfg import DynamicParamsConfig

global prev_config

# used to compare past values
prev_config = {
    'Quadriceps_CH1/2': {
    	'Current_Left': 2,
    	'Current_Right': 2,
    	'Angle_Left_Min': 300,
    	'Angle_Left_Max': 40,
    	'Angle_Right_Min': 120,
    	'Angle_Right_Max': 220
    },
    'Hamstrings_CH3/4': {
        'Current_Left': 2,
        'Current_Right': 2,
        'Angle_Left_Min': 30,
        'Angle_Left_Max': 105,
        'Angle_Right_Min': 210,
        'Angle_Right_Max': 285
    },
    'Gluteal_CH5/6': {
        'Current_Left': 2,
        'Current_Right': 2,
        'Angle_Left_Min': 70,
        'Angle_Left_Max': 170,
        'Angle_Right_Min': 250,
        'Angle_Right_Max': 350
    }
}

def callback(config, level):
    global prev_config
    prefix = {'Quadriceps_CH1/2':'Q_', 'Hamstrings_CH3/4':'H_', 'Gluteal_CH5/6':'G_'}

    for m, p in prefix.items():

        ########### CHANNEL CHECK ###########
        #####################################
        if config[p+'Enable']:
            config['groups']['groups'][m]['state'] = True
        else:
            config['groups']['groups'][m]['state'] = False
            config[p+'Current_Left'] = 2
            config[p+'Current_Right'] = 2

        ########### CURRENT CHECK ###########
        #####################################
        # check for changes in current
        diffL = config[p+'Current_Left'] - prev_config[m]['Current_Left']
        diffR = config[p+'Current_Right'] - prev_config[m]['Current_Right']

        # prevents the user from abruptly increasing the current
        if diffL:            
            if diffL > 2:
                config[p+'Current_Left'] = prev_config[m]['Current_Left'] + 2
                prev_config[m]['Current_Left'] = config[p+'Current_Left']
            else:
                prev_config[m]['Current_Left'] = config[p+'Current_Left']
            # modify left and right at the same time    
            if config[p+'Link_Current']:
                config[p+'Current_Right'] = config[p+'Current_Left']
                prev_config[m]['Current_Right'] = config[p+'Current_Right']
        		
        elif diffR:
            if diffR > 2:
                config[p+'Current_Right'] = prev_config[m]['Current_Right'] + 2
                prev_config[m]['Current_Right'] = config[p+'Current_Right']
            else:
                prev_config[m]['Current_Right'] = config[p+'Current_Right']
            # modify left and right at the same time    
            if config[p+'Link_Current']:
                config[p+'Current_Left'] = config[p+'Current_Right']
                prev_config[m]['Current_Left'] = config[p+'Current_Left']

        ############ ANGLE CHECK ############
        #####################################
        if config[p+'Link_Angle']:
            # check for changes in angle
            diffLmin = config[p+'Angle_Left_Min'] - prev_config[m]['Angle_Left_Min']
            diffLmax = config[p+'Angle_Left_Max'] - prev_config[m]['Angle_Left_Max']
            diffRmin = config[p+'Angle_Right_Min'] - prev_config[m]['Angle_Right_Min']
            diffRmax = config[p+'Angle_Right_Max'] - prev_config[m]['Angle_Right_Max']

    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()
