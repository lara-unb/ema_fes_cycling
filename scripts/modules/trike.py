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
from enum import auto
import rospy

# Stimulation channel mapping
stim_order = [
    'ch1', 'ch2',
    'ch3', 'ch4',
    'ch5', 'ch6',
    'ch7', 'ch8'
]

# Cycling types mapping
cycling_type = ['racing_conv', 'racing_matrix',
                'training_conv', 'training_matrix']


class Trike(object):
    """A class used to control the stimulation.

    Attributes:
        self.config_dict (dict): stores the static config parameters
        self.stim_current (dict): current peak for each channel
        self.stim_current_now (dict): instant current for each channel
        self.stim_current_max (int): max from all values of stim_current
        self.stim_pw (dict): pulse width peak for each channel
        self.stim_pw_now (dict): instant pulse width for each channel
        self.stim_pw_max (int): max from all values of stim_pw
        self.status (string): 'off', 'training', 'racing' or 5 automated
            automated: 'autopw-PC','autopw-PC'
        self.angle (list): appended pedal angles
        self.speed (list): appended angular speeds
        self.speed_err (list): appended speed errors
        self.time (list): appended sensor timestamps
        self.cycles (int): total number of pedal turns
        self.cadence (float): mean km/h speed of last cycle
        self.distance (float): total distance travelled in km
        self.speed_ref (int): reference speed
        self.passed_half_turn (bool): flag if 180 deg has passed
        self.cycle_speed (list): appended angular speeds from latest cycle
        self.autopw_initial (int): initial pulse width for automatic sequence
    """
    def __init__(self, config_dict):
        self.config_dict = config_dict
        self.stim_current = dict.fromkeys(stim_order,0)
        self.stim_current_now = dict.fromkeys(stim_order,0)
        self.stim_current_max = 0
        self.stim_pw = dict.fromkeys(stim_order,0)
        self.stim_pw_now = dict.fromkeys(stim_order,0)
        self.stim_pw_max = 0
        # Other components
        self.status = 'off'
        self.angle = 5*[0]
        self.speed = 5*[0]
        self.speed_err = 5*[0]
        self.time = 5*[0]
        self.cycles = 0
        self.cadence = 0
        self.distance = 0
        # Support components
        self.speed_ref = 300
        self.passed_half_turn = False
        self.cycle_speed = [0]
        # Automated components
        self.autopw_on = False
        self.autopw_initial = 0
        self.autopw_max_1 = 0
        self.autopw_max_2 = 0
        self.autopw_tcons_1 = 0
        self.autopw_tcons_2 = 0
        self.autopw_tramp_1 = 0
        self.autopw_tramp_2 = 0
        self.autopw_step = 0

    def get_latest_measurements(self):
        """Return latest trike data."""
        return (self.time[-1], self.angle[-1], self.speed[-1], self.speed_err[-1])

    def get_stim_list(self):
        """Return stimulation current and pulse width for all channels
        as lists.
        """
        current_list = 8*[0]
        pw_list = 8*[0]
        for i, channel in enumerate(stim_order):
            current_list[i] = self.stim_current_now[channel]
            pw_list[i] = self.stim_pw_now[channel]
        return current_list, pw_list

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

    def set_stim_current(self, value, ch=None, proportion=None):
        """Change the stimulation current. When value is an int, ch or
        proportion are used to update a specific or all channels.

        Attributes:
            value (int/dict): current ampitude/s.
            ch (int): respective stimulation channel.
            proportion (dict): multipliers for every stimulation channel (current).
        """
        # Check for off status
        if self.status == 'off':
            self.stim_current_max = 0
            self.stim_current = dict.fromkeys(stim_order,0)
            return
        # Check safe limit
        limit = self.config_dict['stim_limit'] if 'stim_limit' in self.config_dict else 110
        if isinstance(value, dict):
            if value.keys() != self.stim_current.keys():
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

    def set_stim_pw(self, value, ch=None, proportion=None):
        """Change the stimulation pulse width. When value is an int, ch
        is used to update a specific channel, when None all channels get
        the same value.

        Attributes:
            value (int/dict): pulse width ampitude/s.
            ch (int): respective stimulation channel.
            proportion (dict): multipliers for every stimulation channel (pulse width).
        """
        # Check safe limit
        limit = self.config_dict['pw_limit'] if 'pw_limit' in self.config_dict else 500
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
            elif value == 0:
                self.stim_pw_max = value
                self.stim_pw = dict.fromkeys(stim_order,value)
            # Apply proportion dict
            elif isinstance(proportion, dict):
                if proportion.keys() != self.stim_pw.keys():
                    raise KeyError
                else:
                    maxx = value
                    value = dict((k, int(maxx*proportion[k])) for k in self.stim_pw)
                    self.stim_pw = value
                    self.stim_pw_max = max(self.stim_pw.values())

    def set_status(self, value, zero_distance=True, proportion=None):
        """Change system status.

        Attributes:
            value (string): new system status
            zero_distance (bool): flag to reset distance
            proportion (dict): check active stim channel using current multipliers (rasp platform).
        """
        if value in ('off','training','racing','autopw','autopw-CC','autopw-CM','autopw-TC','autopw-TM'):
            if zero_distance:
                # Reset cycling data
                self.cycles = self.distance = 0
            self.status = value
            if self.status == 'off':
                # Zero stimulation
                self.stim_current_max = 0
                self.stim_current = dict.fromkeys(stim_order,0)
            elif 'autopw' in self.status:
                if self.status == 'autopw-PC': # PC plataform automatic sequence
                    # Reset all to maximum pulse width of active channels
                    active = [k for k, v in self.stim_current.items() if v != 0]
                    self.stim_pw_max = max([v for k, v in self.stim_pw.items() if k in active])
                    self.stim_pw = dict.fromkeys(stim_order,self.stim_pw_max)
                    # Set the starting point for the automatic sequence
                    self.autopw_initial = self.stim_pw_max
                    # Configure automated parameters to PC
                    self.autopw_tramp_1 = self.config_dict['autoPW_tramp_1'] # 1st ramp phase duration
                    self.autopw_tramp_2 = self.config_dict['autoPW_tramp_2']
                    self.autopw_tcons_1 = self.config_dict['autoPW_tcons_1']
                    self.autopw_tcons_2 = self.config_dict['autoPW_tcons_2']
                    self.autopw_max_1 = self.config_dict['autoPW_max_1']
                    self.autopw_max_2 = self.config_dict['autoPW_max_2']
                    self.autopw_step = self.config_dict['autoPW_step']
                    self.autopw_on = self.config_dict['autoPW_on']
                else:
                    # Configure automated parameters to Rasp
                    if self.status == 'autopw-CC': # Conventional Racing
                        automated_type = cycling_type[0]
                    elif self.status == 'autopw-CM': # Matrix Racing
                        automated_type = cycling_type[1]
                    elif self.status == 'autopw-TC': # Conventional Training
                        automated_type = cycling_type[2]
                    elif self.status == 'autopw-TM': # Matrix Training
                        automated_type = cycling_type[3]
                    # Load specific automated pw parameters
                    print(automated_type)
                    self.autopw_tramp_1 = self.config_dict['autoPW_tramp_1'][automated_type]  # 1st ramp phase duration
                    self.autopw_tramp_2 = self.config_dict['autoPW_tramp_2'][automated_type]  # 2nd ramp phase duration
                    self.autopw_tcons_1 = self.config_dict['autoPW_tcons_1'][automated_type]  # 1st constant phase duration
                    self.autopw_tcons_2 = self.config_dict['autoPW_tcons_2'][automated_type]  # 2nd constant phase duration
                    self.autopw_max_1 = self.config_dict['autoPW_max_1'][automated_type] # 1st Maximum pulse width
                    self.autopw_max_2 = self.config_dict['autoPW_max_2'][automated_type] # 2nd Maximum pulse width
                    self.autopw_step = self.config_dict['autoPW_step'][automated_type] # Pulse width change interval - step size
                    self.autopw_on = True
                    # Check if proportion exist
                    if isinstance(proportion, dict):
                        if proportion.keys() != self.stim_pw.keys():
                            raise KeyError
                        else:
                            active = [k for k, v in proportion.items() if v != 0]
                            self.stim_pw_max = max([v for k, v in self.stim_pw.items() if k in active])
                            self.stim_pw = dict.fromkeys(stim_order,self.stim_pw_max)
                            # Set the starting point for the automatic sequence
                            self.autopw_initial = self.stim_pw_max
                            print(self.stim_pw_max)
                    else:
                        self.autopw_initial = 0
                        print('Need to specify stim_proportion for this cycling mode')
                        print('Setting Initial Automated PUlse Width to 0')
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
            self.stim_current_max = 0
            self.stim_current = dict.fromkeys(stim_order,0)
        else:
            # Avoid unexpected changes inside the loop
            angle_now = self.angle[-1]
            speed_now = self.speed[-1]
            for channel, current in self.stim_current.items():
                action = self.calculate_control_action(channel, angle_now, speed_now, self.speed_ref)
                self.stim_current_now[channel] = round(action*current)

    def update_autopw_sequence(self, elapsed, proportion=None):
        """Define an automatic pulse width sequence based on elapsed time.
        The pulse width is incremented with 'autoPW_step' on the corse
        of 'autoPW_tramp' seconds and remains the same for 'autoPW_tcons'
        seconds. It goes on like this until 'autoPW_max'.

        Attributes:
            elapsed (float): time since the sequence was activated
            proportion (dict): multipliers for every stimulation channel (pulse width).
        """
        autopw_on = self.autopw_on # Sequence on/off
        pw_now = self.stim_pw_max  # Instant maximum pulse width from all channels
        # Either turned off or there is no point in increasing
        if (not autopw_on) or (pw_now >= self.autopw_max_2):
            return
        # Assume 1st sequence ongoing
        max_pw = self.autopw_max_1  # 1st plateau
        initial_pw = self.autopw_initial  # Starting point for the sequence
        time_ramp = self.autopw_tramp_1  # 1st ramp phase duration
        time_cons = self.autopw_tcons_1  # 1st constant phase duration
        interval = self.autopw_step  # Pulse width change interval - step size
        period = time_ramp+time_cons  # Cycle period
        # Check if on 2nd sequence or transitioning
        if pw_now >= max_pw:
            # Check 1st sequence total duration
            height_1 = max_pw-initial_pw
            duration_1 = ((height_1/interval)+(height_1%interval > 0))*period
            if (pw_now > max_pw) or (elapsed > duration_1):
                # 2nd sequence ongoing
                max_pw = self.autopw_max_2  # 2nd maximum pulse width
                initial_pw = max(self.autopw_max_1,initial_pw)  # Starting point for the sequence
                time_ramp = self.autopw_tramp_2  # 2nd ramp phase duration
                time_cons = self.autopw_tcons_2  # 2nd constant phase duration
                period = time_ramp+time_cons  # Cycle period
                elapsed -= duration_1  # Ignore the time spent on 1st sequence
        # Logic behind the sequence phases
        time_served = elapsed%period  # How much time of current period has passed
        # Find what's the current phase
        if (time_served < time_ramp):  # On transition ramp phase
            ramp_end = initial_pw+(1+int(elapsed/period))*interval  # Pulse width when ramp ends
            if ramp_end > max_pw:  # In case the maximum value is not multiple of interval
                ramp_end = max_pw
            ramp_start = initial_pw+int(elapsed/period)*interval  # Pulse width when ramp started
            ramp_slope = (ramp_end-ramp_start)/time_ramp
            pw_updated = int(ramp_start+time_served*ramp_slope)
        else:  # On constant phase
            # Confirm the constant value was reached
            pw_updated = initial_pw+(1+int(elapsed/period))*interval
            if pw_updated > max_pw:
                pw_updated = max_pw
        # Avoid unecessary updates
        if pw_updated != pw_now:
            # Check if proportion exist
            if isinstance(proportion, dict):
                if proportion.keys() != self.stim_pw.keys():
                    raise KeyError
                else:
                    maxx = pw_updated
                    pw_updated = dict((k, int(maxx*proportion[k])) for k in self.stim_pw)
                    self.stim_pw = pw_updated
                    self.stim_pw_max = max(pw_updated.values())
            else:
                self.stim_pw_max = pw_updated
                self.stim_pw = dict.fromkeys(stim_order,pw_updated)

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
