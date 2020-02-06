#!/usr/bin/env python
#
# Dont change group names! they're used by other pieces of code (eg. "Ch12" should remain the same)
#
# Ch12: commonly used with quadriceps, refers to stimulator channels 1 and 2
# Ch34: commonly used with hamstrings, refers to stimulator channels 3 and 4
# Ch56: commonly used with gluteal, refers to stimulator channels 5 and 6
# Ch78: commonly not used, refers to stimulator channels 7 and 8

import rospy
from dynamic_reconfigure.server import Server
from ema_fes_cycling.cfg import DynamicParamsConfig

global prev_config

# used to compare past values
prev_config = {
    'Ch12': {
    	'Ch1_Current':0,
    	'Ch2_Current':0,
        'Ch1_Pulse_Width': 500,
        'Ch2_Pulse_Width': 500,
    	'Ch1_Angle_Min': 280,
    	'Ch1_Angle_Max': 0,
    	'Ch2_Angle_Min': 100,
    	'Ch2_Angle_Max': 180
    },
    'Ch34': {
        'Ch3_Current':0,
        'Ch4_Current':0,
        'Ch3_Pulse_Width': 500,
        'Ch4_Pulse_Width': 500,
        'Ch3_Angle_Min': 30,
        'Ch3_Angle_Max': 105,
        'Ch4_Angle_Min': 210,
        'Ch4_Angle_Max': 285
    },
    'Ch56': {
        'Ch5_Current':0,
        'Ch6_Current':0,
        'Ch5_Pulse_Width': 500,
        'Ch6_Pulse_Width': 500,
        'Ch5_Angle_Min': 70,
        'Ch5_Angle_Max': 170,
        'Ch6_Angle_Min': 250,
        'Ch6_Angle_Max': 350
    },
    'Ch78': {
        'Ch7_Current':0,
        'Ch8_Current':0,
        'Ch7_Pulse_Width': 500,
        'Ch8_Pulse_Width': 500,
        'Ch7_Angle_Min': 0,
        'Ch7_Angle_Max': 0,
        'Ch8_Angle_Min': 0,
        'Ch8_Angle_Max': 0
    }
}

# stim channel mapping
channel_groups = ['Ch12','Ch34','Ch56','Ch78']

def callback(config, level):
    for p in channel_groups:
        odd = p[2]
        even = p[3]

        ########### CHANNEL CHECK ###########
        #####################################
        # config['groups']['groups']['MyGroup']['state'] = False # for types 'hide' and 'collapse'
        if not config[p+'_'+'Enable']:
            config['groups']['groups'][p]['state'] = False
            config['Ch'+odd+'_Current'] = 0
            config['Ch'+even+'_Current'] = 0

            continue # channel group deactivated

        else:
            config['groups']['groups'][p]['state'] = True

            for n in (odd,even):
                m = str((int(n)-1)+(2*(int(n)%2))) # if n=odd, m=even; if n=even, m=odd

                ########### CURRENT CHECK ###########
                #####################################
                # check for changes in current
                diff = config['Ch'+n+'_Current'] - prev_config[p]['Ch'+n+'_Current']
                # prevents the user from abruptly increasing the current
                if diff:            
                    if diff > 2:
                        config['Ch'+n+'_Current'] = prev_config[p]['Ch'+n+'_Current'] + 2
                        prev_config[p]['Ch'+n+'_Current'] = config['Ch'+n+'_Current']
                    else:
                        prev_config[p]['Ch'+n+'_Current'] = config['Ch'+n+'_Current']
                    # modify left and right at the same time    
                    if config[p+'_Link_Current']: 
                        config['Ch'+m+'_Current'] = config['Ch'+n+'_Current']
                        prev_config[p]['Ch'+m+'_Current']  = config['Ch'+m+'_Current']
                    
                    return config # assumes only one change per callback

                ######### PULSE WIDTH CHECK #########
                #####################################
                # check for changes in pulse width
                diff = config['Ch'+n+'_Pulse_Width'] - prev_config[p]['Ch'+n+'_Pulse_Width']
                # prevents the user from abruptly increasing the current
                if diff:
                    prev_config[p]['Ch'+n+'_Pulse_Width'] = config['Ch'+n+'_Pulse_Width']
                    # modify left and right at the same time    
                    if config[p+'_'+'Link_Current']:
                        config['Ch'+m+'_Pulse_Width'] = config['Ch'+n+'_Pulse_Width']
                        prev_config[p]['Ch'+m+'_Pulse_Width'] = config['Ch'+m+'_Pulse_Width']
                    
                    return config # assumes only one change per callback

                ############ ANGLE CHECK ############
                #####################################
                if config[p+'_'+'Link_Angle']:
                    # check for changes in angle
                    diffmin = config['Ch'+n+'_Angle_Min'] - prev_config[p]['Ch'+n+'_Angle_Min']
                    if diffmin:
                        if 0<=(config['Ch'+m+'_Angle_Min'] + diffmin)<=360:
                            config['Ch'+m+'_Angle_Min'] += diffmin # left n right legs linked
                            prev_config[p]['Ch'+m+'_Angle_Min'] = config['Ch'+m+'_Angle_Min']
                            prev_config[p]['Ch'+n+'_Angle_Min']  = config['Ch'+n+'_Angle_Min']
                        else: # one change makes the other exceed its range
                            config['Ch'+n+'_Angle_Min'] = prev_config[p]['Ch'+n+'_Angle_Min']

                        return config # assumes only one change per callback

                    # check for changes in angle
                    diffmax = config['Ch'+n+'_Angle_Max'] - prev_config[p]['Ch'+n+'_Angle_Max']
                    if diffmax:
                        if 0<=(config['Ch'+m+'_Angle_Max'] + diffmax)<=360:
                            config['Ch'+m+'_Angle_Max'] += diffmax # left n right legs linked
                            prev_config[p]['Ch'+m+'_Angle_Max'] = config['Ch'+m+'_Angle_Max']
                            prev_config[p]['Ch'+n+'_Angle_Max']  = config['Ch'+n+'_Angle_Max']
                        else: # one change makes the other exceed its range
                            config['Ch'+n+'_Angle_Max'] = prev_config[p]['Ch'+n+'_Angle_Max']

                        return config # assumes only one change per callback
                    
    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()
