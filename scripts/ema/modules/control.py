#!/usr/bin/env python

import rospy

# muscle and stim channel mapping
stim_order = ['Quad_Left','Quad_Right', # CH1 & CH2
              'Hams_Left','Hams_Right', # CH3 & CH4
              'Glut_Left','Glut_Right'] # CH5 & CH6

class Control:

    def __init__(self, config_dict):
    	self.config_dict = config_dict

###############################################
# To stimulate or not based on sensor's angle 
###############################################

    def fx(self, muscle, angle, speed, speed_ref):
        m = muscle[0] # Q, H or G
        side = muscle[5:] # Left/Right
        ramp_degrees = 10.0
        param_dict = rospy.get_param('/ema/server/')
        dth = (speed/speed_ref)*param_dict['Shift']
        # param_dict = self.config_dict[muscle[0:4]]
        # dth = (speed/speed_ref)*self.config_dict['Shift']

        theta_min = param_dict[m+"_Angle_"+side+"_Min"] - dth
        theta_max = param_dict[m+"_Angle_"+side+"_Max"] - dth

        # check if angle in range (theta_min, theta_max) 
        if theta_min <= angle and angle <= theta_max:
            if (angle-theta_min) <= ramp_degrees:
                return (angle-theta_min)/ramp_degrees
            elif (theta_max-angle) <= ramp_degrees:
                return (theta_max-angle)/ramp_degrees
            else:
                return 1
        elif param_dict[m+"_Angle_"+side+"_Min"] > param_dict[m+"_Angle_"+side+"_Max"]:
            if angle <= theta_min and angle <= theta_max:
                if theta_min <= angle + 360 and angle <= theta_max:
                    if (angle+360-theta_min) <= ramp_degrees:
                        return (angle+360-theta_min)/ramp_degrees
                    elif (theta_max-angle) <= ramp_degrees:
                        return (theta_max-angle)/ramp_degrees
                    else:
                        return 1
            elif angle >= theta_min and angle >= theta_max:
                if theta_min <= angle and angle <= theta_max + 360:
                    if (theta_max+360-angle) <= ramp_degrees:
                        return (theta_max+360-angle)/ramp_degrees
                    elif (angle-theta_min) <= ramp_degrees:
                        return (angle-theta_min)/ramp_degrees
                    else:
                        return 1

        return 0

###############################################
# Control coefficient routine
###############################################

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
    
###############################################
# Control applied to stimulation signal
###############################################
    
    def calculate(self, angle, speed, speed_ref, speed_err):
        factor = 6*[0]

        for i, x in enumerate(stim_order):
            factor[i] = self.fx(x, angle, speed, speed_ref)

        return factor
    
###############################################
# Control applied to current amplitude
###############################################
    
    def automatic(self, stim_dict, increment, cadence, min_cadence, limit):
        if (cadence < min_cadence) and (increment<limit):
            step = 2

            if increment+step > limit:
                step = abs(limit-increment+step)

            increment = min(increment+step, limit)

            for x in stim_order:
                muscle = x[:4] # Quad, Hams or Glut
                side = x[5:] # Left or Right

                stim_dict[muscle][side] = stim_dict[muscle][side] + step

        return stim_dict, increment