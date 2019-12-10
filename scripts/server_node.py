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
        'Pulse_Width_Left': 0,
        'Pulse_Width_Right': 0,
    	'Angle_Left_Min': 280,
    	'Angle_Left_Max': 20,
    	'Angle_Right_Min': 100,
    	'Angle_Right_Max': 200
    },
    'Hamstrings_CH3/4': {
        'Current_Left': 2,
        'Current_Right': 2,
        'Pulse_Width_Left': 0,
        'Pulse_Width_Right': 0,
        'Angle_Left_Min': 30,
        'Angle_Left_Max': 105,
        'Angle_Right_Min': 210,
        'Angle_Right_Max': 285
    },
    'Gluteal_CH5/6': {
        'Current_Left': 2,
        'Current_Right': 2,
        'Pulse_Width_Left': 0,
        'Pulse_Width_Right': 0,
        'Angle_Left_Min': 70,
        'Angle_Left_Max': 170,
        'Angle_Right_Min': 250,
        'Angle_Right_Max': 350
    }
}

# muscle and group mapping
muscle_dict = {
    'Quad': 'Quadriceps_CH1/2',
    'Hams': 'Hamstrings_CH3/4',
    'Glut': 'Gluteal_CH5/6'
}

def callback(config, level):
    for m, p in muscle_dict.items():

        ########### CHANNEL CHECK ###########
        #####################################
        # config['groups']['groups']['MyGroup']['state'] = False # for types 'hide' and 'collapse'
        if config[p[0]+'_'+'Enable']:
            config['groups']['groups'][p]['state'] = True
        else:
            config['groups']['groups'][p]['state'] = False
            config[p[0]+'_'+'Current_Left'] = 0
            config[p[0]+'_'+'Current_Right'] = 0

            continue # muscle group deactivated

        ########### CURRENT CHECK ###########
        #####################################
        # check for changes in current
        diffL = config[p[0]+'_'+'Current_Left'] - prev_config[p]['Current_Left']
        # prevents the user from abruptly increasing the current
        if diffL:            
            if diffL > 2:
                config[p[0]+'_'+'Current_Left'] = prev_config[p]['Current_Left'] + 2
                prev_config[p]['Current_Left'] = config[p[0]+'_'+'Current_Left']
            else:
                prev_config[p]['Current_Left'] = config[p[0]+'_'+'Current_Left']
            # modify left and right at the same time    
            if config[p[0]+'_'+'Link_Current']:
                config[p[0]+'_'+'Current_Right'] = config[p[0]+'_'+'Current_Left']
                prev_config[p]['Current_Right']  = config[p[0]+'_'+'Current_Right']
            
            return config # assumes only one change per callback

        # check for changes in current
        diffR = config[p[0]+'_'+'Current_Right'] - prev_config[p]['Current_Right']
        # prevents the user from abruptly increasing the current
        if diffR:
            if diffR > 2:
                config[p[0]+'_'+'Current_Right'] = prev_config[p]['Current_Right'] + 2
                prev_config[p]['Current_Right'] = config[p[0]+'_'+'Current_Right']
            else:
                prev_config[p]['Current_Right'] = config[p[0]+'_'+'Current_Right']
            # modify left and right at the same time    
            if config[p[0]+'_'+'Link_Current']:
                config[p[0]+'_'+'Current_Left'] = config[p[0]+'_'+'Current_Right']
                prev_config[p]['Current_Left']  = config[p[0]+'_'+'Current_Left']

            return config # assumes only one change per callback

        ######### PULSE WIDTH CHECK #########
        #####################################
        # check for changes in pulse width
        diffL = config[p[0]+'_'+'Pulse_Width_Left'] - prev_config[p]['Pulse_Width_Left']
        # prevents the user from abruptly increasing the current
        if diffL:
            prev_config[p]['Pulse_Width_Left'] = config[p[0]+'_'+'Pulse_Width_Left']
            # modify left and right at the same time    
            if config[p[0]+'_'+'Link_Current']:
                config[p[0]+'_'+'Pulse_Width_Right'] = config[p[0]+'_'+'Pulse_Width_Left']
                prev_config[p]['Pulse_Width_Right']  = config[p[0]+'_'+'Pulse_Width_Right']
            
            return config # assumes only one change per callback

        # check for changes in pulse width
        diffR = config[p[0]+'_'+'Pulse_Width_Right'] - prev_config[p]['Pulse_Width_Right']
        # prevents the user from abruptly increasing the current
        if diffR:
            prev_config[p]['Pulse_Width_Right'] = config[p[0]+'_'+'Pulse_Width_Right']
            # modify left and right at the same time    
            if config[p[0]+'_'+'Link_Current']:
                config[p[0]+'_'+'Pulse_Width_Left'] = config[p[0]+'_'+'Pulse_Width_Right']
                prev_config[p]['Pulse_Width_Left']  = config[p[0]+'_'+'Pulse_Width_Left']
            
            return config # assumes only one change per callback

        ############ ANGLE CHECK ############
        #####################################
        if config[p[0]+'_'+'Link_Angle']:
            # check for changes in angle
            diffLmin = config[p[0]+'_'+'Angle_Left_Min'] - prev_config[p]['Angle_Left_Min']
            if diffLmin:
                if 0<=(config[p[0]+'_'+'Angle_Right_Min'] + diffLmin)<=360:
                    config[p[0]+'_'+'Angle_Right_Min'] += diffLmin # left n right legs linked
                    prev_config[p]['Angle_Right_Min'] = config[p[0]+'_'+'Angle_Right_Min']
                    prev_config[p]['Angle_Left_Min']  = config[p[0]+'_'+'Angle_Left_Min']
                else: # one change makes the other exceed its range
                    config[p[0]+'_'+'Angle_Left_Min'] = prev_config[p]['Angle_Left_Min']

                return config # assumes only one change per callback

            # check for changes in angle
            diffLmax = config[p[0]+'_'+'Angle_Left_Max'] - prev_config[p]['Angle_Left_Max']
            if diffLmax:
                if 0<=(config[p[0]+'_'+'Angle_Right_Max'] + diffLmax)<=360:
                    config[p[0]+'_'+'Angle_Right_Max'] += diffLmax # left n right legs linked
                    prev_config[p]['Angle_Right_Max'] = config[p[0]+'_'+'Angle_Right_Max']
                    prev_config[p]['Angle_Left_Max']  = config[p[0]+'_'+'Angle_Left_Max']
                else: # one change makes the other exceed its range
                    config[p[0]+'_'+'Angle_Left_Max'] = prev_config[p]['Angle_Left_Max']

                return config # assumes only one change per callback

            # check for changes in angle
            diffRmin = config[p[0]+'_'+'Angle_Right_Min'] - prev_config[p]['Angle_Right_Min']
            if diffRmin:
                if 0<=(config[p[0]+'_'+'Angle_Left_Min'] + diffRmin)<=360:
                    config[p[0]+'_'+'Angle_Left_Min'] += diffRmin # left n right legs linked
                    prev_config[p]['Angle_Left_Min'] = config[p[0]+'_'+'Angle_Left_Min']
                    prev_config[p]['Angle_Right_Min']  = config[p[0]+'_'+'Angle_Right_Min']
                else: # one change makes the other exceed its range
                    config[p[0]+'_'+'Angle_Right_Min'] = prev_config[p]['Angle_Right_Min']

                return config # assumes only one change per callback

            # check for changes in angle
            diffRmax = config[p[0]+'_'+'Angle_Right_Max'] - prev_config[p]['Angle_Right_Max']
            if diffRmax:
                if 0<=(config[p[0]+'_'+'Angle_Left_Max'] + diffRmax)<=360:
                    config[p[0]+'_'+'Angle_Left_Max'] += diffRmax # left n right legs linked
                    prev_config[p]['Angle_Left_Max'] = config[p[0]+'_'+'Angle_Left_Max']
                    prev_config[p]['Angle_Right_Max']  = config[p[0]+'_'+'Angle_Right_Max']
                else: # one change makes the other exceed its range
                    config[p[0]+'_'+'Angle_Right_Max'] = prev_config[p]['Angle_Right_Max']

                return config # assumes only one change per callback
                
    return config

if __name__ == "__main__":
    rospy.init_node('server', anonymous=False)

    srv = Server(DynamicParamsConfig, callback)
    rospy.spin()
