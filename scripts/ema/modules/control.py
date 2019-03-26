#!/usr/bin/env python

import rospy

class Control:

    def __init__(self, config_dict):
    	self.config_dict = config_dict
        
    def fx(self, id, angle, speed, speed_ref):
        
    	theta = rospy.get_param('/ema/server/')
        dth = (speed/speed_ref)*theta['Shift']

        theta_min = theta["Angle_"+id+"_Min"] - dth
        theta_max = theta["Angle_"+id+"_Max"] - dth

        # check if angle in range (theta_min, theta_max)
        if theta_min <= angle and angle <= theta_max:
            return 1
        elif theta["Angle_"+id+"_Min"] > theta["Angle_"+id+"_Max"]:
            if angle <= theta_min and angle <= theta_max:
                if theta_min <= angle + 360 and angle <= theta_max:
                    return 1
            elif angle >= theta_min and angle >= theta_max:
                if theta_min <= angle and angle <= theta_max + 360:
                    return 1

        # return 0 otherwise
        return 0

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
            
    def calculate(self, angle, speed, speed_ref, speed_err):
        
        fx_left = self.fx('Left', angle, speed, speed_ref)
        fx_right = self.fx('Right', angle, speed, speed_ref)
        
        # g = self.g(speed_err)
        g = 1
        
        bool_left = fx_left*g
        bool_right = fx_right*g

        return bool_left, bool_right
