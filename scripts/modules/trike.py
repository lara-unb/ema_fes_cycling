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

# Stim channel mapping:
stim_order = [
    'ch1', 'ch2',
    'ch3', 'ch4',
    'ch5', 'ch6',
    'ch7', 'ch8'
]


class Control(object):
    """A class used to control the stimulation.

    Attributes:
        config_dict (dict): stores the static config parameters
    """
    def __init__(self, config_dict):
        self.config_dict = config_dict

    def fx(self, ch, angle, speed, speed_ref):
        """Decide to stimulate or not based on sensor.

        Attributes:
            angle (double): pedal angle
            speed (double): pedal angular speed
            speed_ref (double): predefined reference speed
        """
        # Get the parameters from dict
        ramp_start = self.config_dict['ramp_start']
        ramp_end = self.config_dict['ramp_end']
        dth = (speed/speed_ref)*self.config_dict['shift']
        theta_min = self.config_dict[ch+"_angle_min"]-dth
        theta_max = self.config_dict[ch+"_angle_max"]-dth
        # Check if angle in range (theta_min, theta_max)
        if theta_min <= angle and angle <= theta_max:
            if (angle-theta_min) <= ramp_start:
                return (angle-theta_min)/ramp_start
            elif (theta_max-angle) <= ramp_end:
                return (theta_max-angle)/ramp_end
            else:
                return 1
        elif theta_min > theta_max:
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

    def g(self, error):
        """PI speed controller logic.

        Attributes:
            error (double): speed error input
        """
        Kp = 1/float(5000)
        Ki = 1/float(100000)
        # If there is a change of signal, reset.
        if ((error[-2] >= 0) and (error[-1] < 0)) or ((error[-2] < 0) and (error[-1] >= 0)):
            errorTemp = [0 for x in range(len(error))]
            errorTemp[-1] = error[-1]
            error = errorTemp
        signal = 0.5 + Kp*error[-1]+Ki*sum(error)
        # Saturation:
        if signal > 1:
            signal = 1
            error[-1] = 0
        elif signal < 0:
            signal = 0
            error[-1] = 0
        return signal

    def calculate(self, angle, speed, speed_ref, speed_err):
        """Tell if stimulation should be activated or not.

        Attributes:
            angle (double): pedal angle
            speed (double): pedal angular speed
            speed_ref (double): predefined reference speed
            speed_err (double): calculated speed error
        """
        factor = 8*[0]
        for i, ch in enumerate(stim_order):
            factor[i] = self.fx(ch, angle, speed, speed_ref)
        return factor

    def automatic(self, stim_dict, increment, cadence, min_cadence, limit):
        """Cadence control applied to stimulation current amplitude.

        Attributes:
            stim_dict (dict): stores the current in each stimulation channel
            increment (int): amount to add
            cadence (int): speed in rpm
            min_cadence (int): control threshold
            limit (int): auto-adjust control limit
        """
        if (cadence < min_cadence) and (increment < limit):
            step = 2
            if increment+step > limit:
                step = abs(limit-increment+step)
            increment = min(increment+step, limit)
            for ch in stim_order:
                stim_dict[ch] = stim_dict[ch] + step
        return stim_dict, increment

    def initialize(self, current_dict, pw_dict):
        """Initialize the current amplitude.

        Attributes:
            current_dict (dict): stores the current values
            pw_dict (dict): stores the pulse width values
        """
        try:
            ini = self.config_dict['training_current']
            pw = self.config_dict['pulse_width']
            proportion = self.config_dict['stim_proportion']
            for ch in stim_order:
                current_dict[ch] = round(ini*proportion[ch])
                pw_dict[ch] = pw
        except KeyError as e:
            ini = 0
            for ch in stim_order:
                current_dict[ch], pw_dict[ch] = (0,0)
        return ini, current_dict, pw_dict

    def multipliers(self):
        """Return the proportion dictionary."""
        return self.config_dict['stim_proportion']

    def current_limit(self):
        """Return the max current among all channels."""
        return self.config_dict['stim_limit']

    def update_param(self, new, value=None):
        if isinstance(new, dict):
            self.config_dict = new
        else:
            self.config_dict[new] = value
        return
