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

# Stimulation channel mapping
stim_order = [
    'ch1', 'ch2',
    'ch3', 'ch4',
    'ch5', 'ch6',
    'ch7', 'ch8'
]


class Trike(object):
    """A class used to control the stimulation.

    Attributes:
        config_dict (dict): stores the static config parameters
    """
    def __init__(self, config_dict):
        self.config_dict = config_dict
        self.stim_pw = dict.fromkeys(stim_order,0)  # Pulse width peak for each channel
        self.stim_pw_now = dict.fromkeys(stim_order,0)  # Instant pulse width for each channel
        self.stim_pw_max = 0  # Max from all values of stim_pw
        self.stim_current = dict.fromkeys(stim_order,0)  # Current peak for each channel
        self.stim_current_now = dict.fromkeys(stim_order,0)  # Instant current for each channel
        self.stim_current_max = 0  # Max from all values of stim_current

        # Other components
        self.status = 'off'  # 'off', 'training' or 'racing'
        self.angle = 5*[0]  # List of pedal angles
        self.speed = 5*[0]  # List of pedal angular speeds
        self.speed_err = 5*[0]  # List of speed error
        self.time = 5*[0]  # List of sensor timestamps
        self.cycles = 0  # Number of pedal turns
        self.cadence = 0  # Mean km/h speed of last cycle
        self.distance = 0  # Distance travelled in km

        # Support components
        self.speed_ref = 300  # Reference speed
        self.passed_half_turn = False  # Flag 180 deg has passed
        self.cycle_speed = [0]  # List of current cycle speeds

        # Additional statements
        if 'stim_limit' not in self.config_dict:
            self.config_dict['stim_limit'] = 110  # Current limit in mA

    def get_latest_measurements(self):
        """Return latest trike data."""
        return (self.time[-1], self.angle[-1], self.speed[-1], self.speed_err[-1])

    def get_stim_list(self):
        """Return stimulation current and pulse width for all channels
        as lists.
        """
        pw_list = 8*[0]
        current_list = 8*[0]
        for i, channel in enumerate(stim_order):
            pw_list[i] = self.stim_pw_now[channel]
            current_list[i] = self.stim_current_now[channel]
        return pw_list, current_list

    def set_latest_measurements(self, time, angle, speed):
        """Update pedal data based on received measurements.

        Attributes:
            time (double): sensor timestamp
            angle (double): pedal angle
            speed (double): pedal angular speed
        """
        self.time.append(time)
        self.angle.append(angle)
        self.speed.append(speed)
        self.speed_err.append(self.speed_ref-speed)
        self.cycle_speed.append(speed)

    def set_stim_pw(self, value, ch=None):
        """Change the stimulation pulse width. When value is an int, ch
        is used to update a specific channel, when None all channels get
        the same value.

        Attributes:
            value (int/dict): pulse width ampitude/s.
            ch (int): respective stimulation channel.
        """
        # Check safe limit
        limit = 500
        # Every pulse width amplitude specified
        if isinstance(value, dict):
            if value.keys() != self.stim_pw.keys():
                raise KeyError
            else:
                for v in value.values():
                    if not isinstance(v, int):
                        raise TypeError
                    if v > limit:
                        v = limit
                    if v < 0:
                        v = 0
                self.stim_pw = value
                self.stim_pw_max = max(self.stim_pw.values())
        elif isinstance(value, int):
            # Force safe limit
            if value > limit:
                value = limit
            # Don't accept negative values
            if value < 0:
                value = 0
            # Modify only one channel
            if ch:
                k = 'ch'+str(ch)
                if k not in self.stim_pw:
                    raise KeyError
                else:
                    self.stim_pw[k] = value
                    self.stim_pw_max = max(self.stim_pw.values())
            # All channels at once
            else:
                self.stim_pw_max = value
                self.stim_pw = dict.fromkeys(stim_order,value)

    def set_stim_current(self, value, ch=None, proportion=None):
        """Change the stimulation current. When value is an int, ch or
        proportion are used to update a specific or all channels.

        Attributes:
            value (int/dict): current ampitude/s.
            ch (int): respective stimulation channel.
            proportion (dict): multipliers for every stimulation channel.
        """
        # Check for off status
        if self.status == 'off':
            self.stim_current_max = 0
            self.stim_current = dict.fromkeys(stim_order,0)
            return
        # Check safe limit
        limit = self.config_dict['stim_limit']
        if isinstance(value, dict):
            if values.keys() != self.stim_current.keys():
                raise KeyError
            else:
                for v in value.values():
                    if not isinstance(v, int):
                        raise TypeError
                    if v > limit:
                        v = limit
                    if v < 0:
                        v = 0
                self.stim_current = value
                self.stim_current_max = max(self.stim_current.values())
        elif isinstance(value, int):
            # Force safe limit
            if value > limit:
                value = limit
            # Don't accept negative values
            if value < 0:
                value = 0
            # Modify only one channel
            if ch:
                k = 'ch'+str(ch)
                if k not in self.stim_current:
                    raise KeyError
                else:
                    self.stim_current[k] = value
                    self.stim_current_max = max(self.stim_current.values())
            # Zero all channels
            elif value == 0:
                self.stim_current_max = 0
                self.stim_current = dict.fromkeys(stim_order,0)
            # Apply proportion dict
            elif isinstance(proportion, dict):
                if proportion.keys() != self.stim_current.keys():
                    raise KeyError
                else:
                    maxx = value
                    value = dict((k, int(maxx*proportion[k])) for k in self.stim_current)
                    self.stim_current = value
                    self.stim_current_max = max(self.stim_current.values())
        return

    def set_status(self, value):
        """Change system status.

        Attributes:
            value (string): new system status
        """
        if value in ('off','training','racing'):
            # Reset cycling data
            self.cycles = self.distance = 0
            self.status = value
            if self.status == 'off':
                # Zero stimulation
                self.stim_current = dict.fromkeys(stim_order,0)
        else:
            raise ValueError

    def set_config(self, param, value=None):
        """Change configuration parameters. Param can be a new configuration
        dictionary or a specific parameter with its new value in value.

        Attributes:
            param (dict/string): configuration dictionary or parameter name
            value (): parameter value when param is a parameter name
        """
        if isinstance(param, dict):
            self.config_dict = param.copy()
        else:
            self.config_dict[param] = value
        return

    def check_new_cycle(self, ignored=''):
        """Check for pedal turns.

        Attributes:
            ignored (string): 'cadence', 'cycles', 'distance' or a tuple
                of those. When flagged the parameter is not updated.
        """
        if (self.angle[-1] > 160) and (self.angle[-1] < 200):  # Flag half-turn
            self.passed_half_turn = True
        if self.passed_half_turn:
            # Get greater absolute difference between five consecutive angles
            pack = self.angle[-5:]
            pack_shifted = self.angle[-6:-1]
            pack_diff = [item-pack_shifted[i] for i, item in enumerate(pack)]
            greater_diff = max(pack_diff, key=abs)  # Greater absolute difference
            # Account for angles past 360 deg
            if abs(greater_diff) > 300:
                self.passed_half_turn = False  # Reset half-turn flag
                try:
                    if 'cadence' not in ignored:
                        # Simple mean and 6 for deg/s to rpm
                        self.cadence = sum(self.cycle_speed)/(6*len(self.cycle_speed))
                        # One crankset turn is equivalent to 1.5 wheel
                        # turn, the tire diameter is 66cm(26in) and 100k
                        # for cm to km, so pi*1.5*66[rpm] = 1[cm/min] and
                        # 60/100k[cm/min] = 1[km/h]
                        self.cadence = 3.1415*0.0594*self.cadence  # rpm to km/h
                    self.cycle_speed = []  # Reset list for new cycle
                    # Moving forward
                    if (greater_diff < 0):
                        if 'distance' not in ignored:
                            self.distance = (self.cycles*3.1415*1.5*66)/100000
                            if 'cycles' not in ignored:
                                self.cycles += 1  # Count turns
                except ZeroDivisionError as e:
                    rospy.logerr(e)

    def update_stim_output(self):
        """Update stimulation attributes according to latest measurements."""
        self.stim_pw_now = self.stim_pw.copy()
        if self.status == 'off':
            self.stim_current = dict.fromkeys(stim_order,0)
        else:
            for channel, current in self.stim_current.items():
                action = self.calculate_control_action(channel, self.angle[-1],
                    self.speed[-1], self.speed_ref)
                self.stim_current_now[channel] = round(action*current)

    def calculate_control_action(self, ch, angle, speed, speed_ref):
        """Return the control action according to specified inputs.

        Attributes:
            ch (string): 'chX' where X is the stimulation channel
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

        # If there is a change of signal, reset
        if ((error[-2] >= 0) and (error[-1] < 0)) or ((error[-2] < 0) and (error[-1] >= 0)):
            errorTemp = [0 for x in range(len(error))]
            errorTemp[-1] = error[-1]
            error = errorTemp

        signal = 0.5 + Kp*error[-1]+Ki*sum(error)

        # Saturation
        if signal > 1:
            signal = 1
            error[-1] = 0
        elif signal < 0:
            signal = 0
            error[-1] = 0

        return signal

    def automatic(self, stim_dict, increment, cadence, min_cadence, limit):
        """Cadence control applied to stimulation current amplitude.

        Attributes:
            stim_dict (dict): current in each stimulation channel
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
