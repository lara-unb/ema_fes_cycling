#!/usr/bin/env python

import rospy

# stim channel mapping
stim_order = ['Ch12_Odd','Ch12_Even', # CH1 & CH2
              'Ch34_Odd','Ch34_Even', # CH3 & CH4
              'Ch56_Odd','Ch56_Even', # CH5 & CH6
              'Ch78_Odd','Ch78_Even'] # CH5 & CH6

class Control:

    def __init__(self, config_dict):
    	self.config_dict = config_dict

###############################################
# To stimulate or not based on sensor's angle 
###############################################

    def fx(self, channel, angle, speed, speed_ref):
        m = channel[:4] # Ch12, Ch34, Ch56, Ch78
        side = channel[5:] # Odd or Even
        ramp_degrees = 10.0
        param_dict = rospy.get_param('/ema/server/')
        # param_dict = self.config_dict[channel[0:4]]
        # dth = (speed/speed_ref)*param_dict['Shift']
        # dth = (speed/speed_ref)*self.config_dict['Shift']
        dth = 0

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
        factor = 8*[0]

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
                channel = x[:4] # Ch12, Ch34, Ch56, Ch78
                side = x[5:] # Odd or Even

                stim_dict[channel][side] = stim_dict[channel][side] + step

        return stim_dict, increment