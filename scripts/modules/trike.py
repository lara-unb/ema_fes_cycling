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

# Dictionary used for stimulation parameters
template_dict = {
    'Ch1': 0, 'Ch2': 0,
    'Ch3': 0, 'Ch4': 0,
    'Ch5': 0, 'Ch6': 0,
    'Ch7': 0, 'Ch8': 0
}


class Trike(object):
    """A class used to control the stimulation.

    Attributes:
        config_dict (dict): stores the static config parameters
    """
    def __init__(self, config_dict):
        self.config_dict = config_dict
        self.stim_pw = template_dict.copy()  # Pulse width for each channel
        self.stim_pw_now = template_dict.copy()  # Instant pulse width for each channel (index)
        self.stim_current = template_dict.copy()  # Adjustable max current for each channel
        self.stim_current_now = template_dict.copy()  # Instant current for each channel (index)
        self.stim_current_max = 0  # Max from stim_current

        # Other components
        self.on_off = False  # System on/off
        self.time = [0]  # List of sensor timestamps
        self.angle = [0]  # List of pedal angles
        self.speed = [0]  # List of pedal angular speeds
        self.speed_err = [0]  # List of speed error
        self.speed_ref = 300  # Reference speed

    def apply_initial_config(self):
        """Initialize the pulse width and current amplitude."""
        ini = self.config_dict['initial_current']
        proportion = self.config_dict['stim_proportion']
        pw = self.config_dict['pulse_width']
        # Update the current and pulse width dictionaries
        self.stim_current = dict((k, ini*proportion[k]) for k in self.stim_current)
        self.pw_dict = dict((k, pw) for k in self.pw_dict)

    def calculate(self):
        """Update stim attributes according to latest measurements."""
        for channel, current in self.stim_current.items():
            action = self.control_action(channel, self.angle[-1], self.speed[-1], self.speed_ref)
            self.stim_current_now[channel] = round(action*current)
            # Check safe limit
            if self.stim_current_now[channel] > self.config_dict['stim_limit']:
                self.stim_current_now[channel] = self.config_dict['stim_limit']
        self.stim_pw_now = self.stim_pw.copy()

    def control_action(self, ch, angle, speed, speed_ref):
        """Return the control action according to specified inputs.

        Attributes:
            ch (str): stim channel as in the angle parameters
            angle (double): pedal angle
            speed (double): pedal angular speed
            speed_ref (double): predefined reference speed
        """
        ramp_degrees = 10.0
        dth = (speed/speed_ref)*param_dict['Shift']
        dth = (speed/speed_ref)*self.config_dict['Shift']
        theta_min = self.config_dict[ch+"AngleMin"] - dth
        theta_max = self.config_dict[ch+"AngleMax"] - dth

        # Check if angle is in range (theta_min, theta_max)
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

    def update_stim_current(self, item):
        """Change the stimulation current for all channels. "Item" can be
        a dictionary with the new currents or simply an amount to add or
        subtract based on stim proportion.

        Attributes:
            item (dict/int): parameter to change current values.
        """
        if not isinstance(item, dict):
            amount = item
            proportion = self.config_dict['stim_proportion']
            max_current = self.stim_current_max+amount
            # Check safe limit
            if max_current > self.config_dict['stim_limit']:
                return
            item = dict((k, max_current*proportion[k]) for k in self.stim_current)
        else:
            pass
        self.stim_current_max = max(item.values())
        self.stim_current = item
        return

    def update_stim_pw(self, pw_dict):
        """Change the stimulation pulse width for all channels.

        Attributes:
            pw_dict (dict): pulse width dictionary
        """
        self.stim_pw = pw_dict

    def update_config(self, new, value=None):
        """Change configuration parameters. "New" can be a new configuration
        dictionary or a specific parameter with its new value in "value".

        Attributes:
            new (dict/str): configuration dictionary or parameter name
            value (): parameter value when "new" is a parameter name
        """
        if isinstance(new, dict):
            self.config_dict = new
        else:
            self.config_dict[new] = value
        return

    def update_measurements(self, time, angle, speed):
        """Update pedal data based on received measurements.

        Attributes:
            time (double): sensor timestamp
            angle (double): pedal angle
            speed (double): pedal angular speed
        """
        self.time.apeend(time)
        self.angle.apeend(angle)
        self.speed.apeend(speed)
        self.speed_err.append(self.speed_ref-speed)

    def get_latest_measurements(self):
        """Return latest pedal data: timestamp, angle, speed, speed error."""
        return self.time[-1], self.angle[-1], self.speed[-1], self.speed_err[-1]

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
