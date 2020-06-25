#!/usr/bin/env python

"""

Particularly, this code is an auxiliary module for the FES cycling
application. It consists of classes and methods that define the stimulation
controller and give support in a deeper level.

The ROS node uses this code. It gives support in a deeper level, dealing
with minor details and is supposed to be independent of ROS, meaning it
shouldn't have to interact with ROS in any way. For example, it would
establish serial comm and treat raw measurements instead ofpublishing a
filtered sensor measurement as a ROS message to other ROS nodes.

"""

# Stim channel mapping:
stim_order = [
    'Ch1', 'Ch2',
    'Ch3', 'Ch4',
    'Ch5', 'Ch6',
    'Ch7', 'Ch8'
]


class Control:
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
        ramp_degrees = 10.0

        # dth = (speed/speed_ref)*param_dict['Shift']
        # dth = (speed/speed_ref)*self.config_dict['Shift']

        # Shift disabled:
        dth = 0

        theta_min = self.config_dict[ch+"AngleMin"] - dth
        theta_max = self.config_dict[ch+"AngleMax"] - dth

        # Check if angle in range (theta_min, theta_max):
        if theta_min <= angle and angle <= theta_max:
            if (angle-theta_min) <= ramp_degrees:
                return (angle-theta_min)/ramp_degrees
            elif (theta_max-angle) <= ramp_degrees:
                return (theta_max-angle)/ramp_degrees
            else:
                return 1
        elif self.config_dict[ch+"AngleMin"] > self.config_dict[ch+"AngleMax"]:
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

    def initialize(self, current_dict):
        """Initialize the current amplitude.

        Attributes:
            config_dict (dict): stores the static config parameters
        """
        ini = self.config_dict['initial_current']
        proportion = self.config_dict['stim_proportion']

        for ch in stim_order:
            current_dict[ch] = round(ini*proportion[ch])

        return ini, current_dict

    def multipliers(self):
        """Return the proportion dictionary."""
        return self.config_dict['stim_proportion']

    def currentLimit(self):
        """Return the max current among all channels."""
        return self.config_dict['stim_limit']

    def updateParam(self, new, value=None):
        if isinstance(new, dict):
            self.config_dict = new
        else:
            self.config_dict[new] = value
        return