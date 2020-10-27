#!/usr/bin/env python

"""

Particularly, this code is an auxiliary module for the FES cycling
application. It consists of classes and methods that define the stimulation
controller and give support in a deeper level.

The ROS node uses this code. It gives support in a deeper level, dealing
with minor details and is supposed to be independent of ROS, meaning it
shouldn't have to interact with ROS in any way. For example, it would
establish serial comm and treat raw measurements instead of publishing a
filtered sensor measurement as a ROS message to other ROS nodes.

"""

import rospy

# stim channel mapping
stim_order = [
    'Ch1','Ch2',
    'Ch3','Ch4',
    'Ch5','Ch6',
    'Ch7','Ch8'
]

class Control:

    def __init__(self, config_dict):
    	self.config_dict = config_dict

###############################################
# To stimulate or not based on sensor's angle 
###############################################

    def fx(self, ch, angle, speed, speed_ref):
        n = int(ch[2])
        m = (n-1)+(2*(n%2)) # if n=odd, m=even; if n=even, m=odd

        # param_dict = rospy.get_param('/ema/server/')
        # dth = (speed/speed_ref)*param_dict['Shift']
        param_dict = self.config_dict['Ch'+str(min(n,m))+str(max(n,m))]
        dth = (speed/speed_ref)*self.config_dict['shift']
        # dth = 0

        ramp_start = self.config_dict['ramp_start']
        ramp_end = self.config_dict['ramp_end']

        theta_min = param_dict[ch+"_Angle_Min"] - dth
        theta_max = param_dict[ch+"_Angle_Max"] - dth

        # check if angle in range (theta_min, theta_max) 
        if theta_min <= angle and angle <= theta_max:
            if (angle-theta_min) <= ramp_start:
                return (angle-theta_min)/ramp_start
            elif (theta_max-angle) <= ramp_end:
                return (theta_max-angle)/ramp_end
            else:
                return 1
        elif param_dict[ch+"_Angle_Min"] > param_dict[ch+"_Angle_Max"]:
            if angle <= theta_min and angle <= theta_max:
                if theta_min <= angle + 360 and angle <= theta_max:
                    if (angle+360-theta_min) <= ramp_start:
                        return (angle+360-theta_min)/ramp_start
                    elif (theta_max-angle) <= ramp_end:
                        return (theta_max-angle)/ramp_end
                    else:
                        return 1
            elif angle >= theta_min and angle >= theta_max:
                if theta_min <= angle and angle <= theta_max + 360:
                    if (theta_max+360-angle) <= ramp_end:
                        return (theta_max+360-angle)/ramp_end
                    elif (angle-theta_min) <= ramp_start:
                        return (angle-theta_min)/ramp_start
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

        for i, ch in enumerate(stim_order):
            factor[i] = self.fx(ch, angle, speed, speed_ref)

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

            for ch in stim_order:
                stim_dict[ch] = stim_dict[ch] + step

        return stim_dict, increment

###############################################
# Initializes the current amplitude and pw
###############################################

    def initialize(self, current_dict, pw_dict):

        ini = self.config_dict['training_current']
        pw = self.config_dict['pulse_width']
        proportion = self.config_dict['stim_proportion']

        for ch in stim_order:
            current_dict[ch] = round(ini*proportion[ch])
            pw_dict[ch] = pw

        return ini, current_dict, pw_dict

###############################################
# Returns the proportion dictionary
###############################################
    
    def multipliers(self):
        return self.config_dict['stim_proportion']

###############################################
# Returns the max current among all channels
###############################################
    
    def currentLimit(self):
        return self.config_dict['stim_limit']