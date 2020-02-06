#!/usr/bin/env python
#
# Dont change group names! they're used by other pieces of code (eg. "Ch12" should remain the same)
#
# Ch12: commonly used with quadriceps, refers to stimulator channels 1 and 2
# Ch34: commonly used with hamstrings, refers to stimulator channels 3 and 4
# Ch56: commonly used with gluteal, refers to stimulator channels 5 and 6
# Ch78: commonly not used, refers to stimulator channels 7 and 8
# Odd:  commonly used with left side, refers to the odd channel number (1,3,5,7)
# Even: commonly used with right side, refers to the even channel number (2,4,6,8)

import rospy
from dynamic_reconfigure.server import Server
from ema_fes_cycling.cfg import DynamicParamsConfig

global prev_config

# used to compare past values
prev_config = {
    'Ch12': {
    	'Current_Odd':0,
    	'Current_Even':0,
        'Pulse_Width_Odd': 500,
        'Pulse_Width_Even': 500,
    	'Angle_Odd_Min': 280,
    	'Angle_Odd_Max': 0,
    	'Angle_Even_Min': 100,
    	'Angle_Even_Max': 180
    },
    'Ch34': {
        'Current_Odd':0,
        'Current_Even':0,
        'Pulse_Width_Odd': 500,
        'Pulse_Width_Even': 500,
        'Angle_Odd_Min': 30,
        'Angle_Odd_Max': 105,
        'Angle_Even_Min': 210,
        'Angle_Even_Max': 285
    },
    'Ch56': {
        'Current_Odd':0,
        'Current_Even':0,
        'Pulse_Width_Odd': 500,
        'Pulse_Width_Even': 500,
        'Angle_Odd_Min': 70,
        'Angle_Odd_Max': 170,
        'Angle_Even_Min': 250,
        'Angle_Even_Max': 350
    },
    'Ch78': {
        'Current_Odd':0,
        'Current_Even':0,
        'Pulse_Width_Odd': 500,
        'Pulse_Width_Even': 500,
        'Angle_Odd_Min': 0,
        'Angle_Odd_Max': 0,
        'Angle_Even_Min': 0,
        'Angle_Even_Max': 0
    }
}

# stim channel mapping
channel_list = ['Ch12','Ch34','Ch56','Ch78']

def callback(config, level):
    for p in channel_list:

        ########### CHANNEL CHECK ###########
        #####################################
        # config['groups']['groups']['MyGroup']['state'] = False # for types 'hide' and 'collapse'
        if config[p+'_'+'Enable']:
            config['groups']['groups'][p]['state'] = True
        else:
            config['groups']['groups'][p]['state'] = False
            config[p+'_'+'Current_Odd'] = 0
            config[p+'_'+'Current_Even'] = 0

            continue # muscle group deactivated

        ########### CURRENT CHECK ###########
        #####################################
        # check for changes in current
        diffL = config[p+'_'+'Current_Odd'] - prev_config[p]['Current_Odd']
        # prevents the user from abruptly increasing the current
        if diffL:            
            if diffL > 2:
                config[p+'_'+'Current_Odd'] = prev_config[p]['Current_Odd'] + 2
                prev_config[p]['Current_Odd'] = config[p+'_'+'Current_Odd']
            else:
                prev_config[p]['Current_Odd'] = config[p+'_'+'Current_Odd']
            # modify left and right at the same time    
            if config[p+'_'+'Link_Current']:
                config[p+'_'+'Current_Even'] = config[p+'_'+'Current_Odd']
                prev_config[p]['Current_Even']  = config[p+'_'+'Current_Even']
            
            return config # assumes only one change per callback

        # check for changes in current
        diffR = config[p+'_'+'Current_Even'] - prev_config[p]['Current_Even']
        # prevents the user from abruptly increasing the current
        if diffR:
            if diffR > 2:
                config[p+'_'+'Current_Even'] = prev_config[p]['Current_Even'] + 2
                prev_config[p]['Current_Even'] = config[p+'_'+'Current_Even']
            else:
                prev_config[p]['Current_Even'] = config[p+'_'+'Current_Even']
            # modify left and right at the same time    
            if config[p+'_'+'Link_Current']:
                config[p+'_'+'Current_Odd'] = config[p+'_'+'Current_Even']
                prev_config[p]['Current_Odd']  = config[p+'_'+'Current_Odd']

            return config # assumes only one change per callback

        ######### PULSE WIDTH CHECK #########
        #####################################
        # check for changes in pulse width
        diffL = config[p+'_'+'Pulse_Width_Odd'] - prev_config[p]['Pulse_Width_Odd']
        # prevents the user from abruptly increasing the current
        if diffL:
            prev_config[p]['Pulse_Width_Odd'] = config[p+'_'+'Pulse_Width_Odd']
            # modify left and right at the same time    
            if config[p+'_'+'Link_Current']:
                config[p+'_'+'Pulse_Width_Even'] = config[p+'_'+'Pulse_Width_Odd']
                prev_config[p]['Pulse_Width_Even']  = config[p+'_'+'Pulse_Width_Even']
            
            return config # assumes only one change per callback

        # check for changes in pulse width
        diffR = config[p+'_'+'Pulse_Width_Even'] - prev_config[p]['Pulse_Width_Even']
        # prevents the user from abruptly increasing the current
        if diffR:
            prev_config[p]['Pulse_Width_Even'] = config[p+'_'+'Pulse_Width_Even']
            # modify left and right at the same time    
            if config[p+'_'+'Link_Current']:
                config[p+'_'+'Pulse_Width_Odd'] = config[p+'_'+'Pulse_Width_Even']
                prev_config[p]['Pulse_Width_Odd']  = config[p+'_'+'Pulse_Width_Odd']
            
            return config # assumes only one change per callback

        ############ ANGLE CHECK ############
        #####################################
        if config[p+'_'+'Link_Angle']:
            # check for changes in angle
            diffLmin = config[p+'_'+'Angle_Odd_Min'] - prev_config[p]['Angle_Odd_Min']
            if diffLmin:
                if 0<=(config[p+'_'+'Angle_Even_Min'] + diffLmin)<=360:
                    config[p+'_'+'Angle_Even_Min'] += diffLmin # left n right legs linked
                    prev_config[p]['Angle_Even_Min'] = config[p+'_'+'Angle_Even_Min']
                    prev_config[p]['Angle_Odd_Min']  = config[p+'_'+'Angle_Odd_Min']
                else: # one change makes the other exceed its range
                    config[p+'_'+'Angle_Odd_Min'] = prev_config[p]['Angle_Odd_Min']

                return config # assumes only one change per callback

            # check for changes in angle
            diffLmax = config[p+'_'+'Angle_Odd_Max'] - prev_config[p]['Angle_Odd_Max']
            if diffLmax:
                if 0<=(config[p+'_'+'Angle_Even_Max'] + diffLmax)<=360:
                    config[p+'_'+'Angle_Even_Max'] += diffLmax # left n right legs linked
                    prev_config[p]['Angle_Even_Max'] = config[p+'_'+'Angle_Even_Max']
                    prev_config[p]['Angle_Odd_Max']  = config[p+'_'+'Angle_Odd_Max']
                else: # one change makes the other exceed its range
                    config[p+'_'+'Angle_Odd_Max'] = prev_config[p]['Angle_Odd_Max']

                return config # assumes only one change per callback

            # check for changes in angle
            diffRmin = config[p+'_'+'Angle_Even_Min'] - prev_config[p]['Angle_Even_Min']
            if diffRmin:
                if 0<=(config[p+'_'+'Angle_Odd_Min'] + diffRmin)<=360:
                    config[p+'_'+'Angle_Odd_Min'] += diffRmin # left n right legs linked
                    prev_config[p]['Angle_Odd_Min'] = config[p+'_'+'Angle_Odd_Min']
                    prev_config[p]['Angle_Even_Min']  = config[p+'_'+'Angle_Even_Min']
                else: # one change makes the other exceed its range
                    config[p+'_'+'Angle_Even_Min'] = prev_config[p]['Angle_Even_Min']

                return config # assumes only one change per callback

            # check for changes in angle
            diffRmax = config[p+'_'+'Angle_Even_Max'] - prev_config[p]['Angle_Even_Max']
            if diffRmax:
                if 0<=(config[p+'_'+'Angle_Odd_Max'] + diffRmax)<=360:
                    config[p+'_'+'Angle_Odd_Max'] += diffRmax # left n right legs linked
                    prev_config[p]['Angle_Odd_Max'] = config[p+'_'+'Angle_Odd_Max']
                    prev_config[p]['Angle_Even_Max']  = config[p+'_'+'Angle_Even_Max']
                else: # one change makes the other exceed its range
                    config[p+'_'+'Angle_Even_Max'] = prev_config[p]['Angle_Even_Max']

                return config # assumes only one change per callback
                
    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()
