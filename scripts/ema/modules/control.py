#!/usr/bin/env python

import rospy

muscle_dict = {
    'quad': 'Quadriceps_CH1/2',
    'hams': 'Hamstrings_CH3/4',
    'glut': 'Gluteal_CH5/6'
}

class Control:

    def __init__(self, config_dict):
    	self.config_dict = config_dict
    
    # to stimulate or not based on sensor's angle 
    def fx(self, m, side, angle, speed, speed_ref):
        
        muscle = muscle_dict[m][0]
    	param = rospy.get_param('/ema/server/')
        dth = (speed/speed_ref)*param['Shift']

        begin = param[muscle+"Angle_"+side+"_Min"] - dth
        end = param[muscle+"Angle_"+side+"_Max"] - dth

        # check if angle in range (begin, end)
        if begin <= angle and angle <= end:
            return 1
        elif param[muscle+"Angle_"+side+"_Min"] > param[muscle+"Angle_"+side+"_Max"]:
            if angle <= begin and angle <= end:
                if begin <= angle + 360 and angle <= end:
                    return 1
            elif angle >= begin and angle >= end:
                if begin <= angle and angle <= end + 360:
                    return 1

        # return 0 otherwise
        return 0

    # control coefficient routine
    def g(self, error):

        Kp = 1/float(5000)
        Ki = 1/float(100000)
        
        # If there is a change of signal, reset
        if ((error[-2] >= 0) and (error[-1] < 0)) or ((error[-2] < 0) and (error[-1] >= 0)):
            errorTemp = [0 for x in range(len(error))]
            errorTemp[-1] = error[-1]
            error = errorTemp
        
        signal = 0.5 + Kp*error[-1]+Ki*sum(error)
        
        # saturation
        if signal > 1:
            signal = 1
            error[-1] = 0
        elif signal < 0:
            signal = 0
            error[-1] = 0
        
        return signal
    
    # control applied to stimulation signal
    def calculate(self, muscle, angle, speed, speed_ref, speed_err):
        
        fx_left = self.fx(muscle, 'Left', angle, speed, speed_ref)
        fx_right = self.fx(muscle, 'Right', angle, speed, speed_ref)
        
        # g = self.g(speed_err)
        g = 1
        
        bool_left = fx_left*g
        bool_right = fx_right*g

        return bool_left, bool_right
