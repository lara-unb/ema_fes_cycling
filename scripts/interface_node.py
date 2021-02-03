#!/usr/bin/env python

"""

Particularly, this code defines and manage the user interface for the
embedded system.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

                    _________ NOTES _________

The 'menu_ref' dictionary defines the menu/screen structure and the
corresponding interaction between them. Each screen has its message and
submenus, which also have their messages and submenus. The 'welcome'
screen is the root and the screens that have submenus as {} are the
leaves, or action screens, where the user is prompt to make changes. All
the screens are stored in the self.screens dict of the Interface class
with their own characteristcs:
    - label (string): the screen label
    - msg (string): the screen message to be displayed
    - type (string):
        . 'root' has no parent, 
        . 'menu' might have next and prev,
        . 'action' prompts to a parameter change and has no select screen
    - parent (string): the label of its parent screen in the upper level
    - select (string): the label of its child screen, to go when selected
    - next (string): the label of the next screen, menu to the right
    - prev (string): the label of the previous screen, menu to the left

"""

# Python 2 and 3 compatibility
# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function
# from builtins import *

import rospy

# Import ROS msgs
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import Duration
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from ema_common_msgs.srv import Display
from ema_common_msgs.srv import SetUInt16

# Declare constants
DISPLAY_SIZE = (2,16)

# Dictionary with interface structure
menu_ref = {
    'welcome': {
        'msg':'~ EMA Trike ~',
        'submenus': {
            '0-cycling': {
                'msg':'Corrida',
                'submenus': {
                    'confirm-C': {
                        'msg':'Iniciar?\nNao      Sim',
                        'submenus': {
                            'cycling-C': {
                                'msg':'00.0km/h 00.00km\n 000mA   00\'00" ',
                                'submenus': {}
                            }
                        }
                    }
                }
            },
            '1-cycling': {
                'msg':'Treino',
                'submenus': {
                    'confirm-T': {
                        'msg':'Iniciar?\nNao      Sim',
                        'submenus': {
                            'cycling-T': {
                                'msg':'00.0km/h 00.00km\n 000mA   00\'00" ',
                                'submenus': {}
                            }
                        }
                    }
                }
            },
            '2-config': {
                'msg':'Config',
                'submenus': {
                    'imu': {
                        'msg':'Numero IMU',
                        'submenus': {
                            'change_imu': {
                                'msg':'IMU:\n0',
                                'submenus': {}
                            }
                        }
                    },
                    'pw': {
                        'msg':'Larg. Pulso',
                        'submenus': {
                            'change_pw': {
                                'msg':'Larg. Pulso:\n0 us',
                                'submenus': {}
                            }
                        }
                    },
                    'freq': {
                        'msg':'Frequencia',
                        'submenus': {
                            'change_freq': {
                                'msg':'Frequencia:\n0 Hz',
                                'submenus': {}
                            }
                        }
                    },
                    'current': {
                        'msg':'Corrente',
                        'submenus': {
                            'change_current': {
                                'msg':'Corrente:\n0 mA',
                                'submenus': {}
                            }
                        }
                    }
                }
            },
            'X-reboot': {
                'msg':'Religar',
                'submenus': {
                    'reboot_screen': {
                        'msg':'Religar?\nNao      Sim',
                        'submenus': {}
                    }
                }
            }
        }
    }
}


class Interface(object):
    """A class used to create the user interface menus.

    Attributes:
        ref_dict (dict): screen structure
        self.screen_now (dict): screen being displayed
        self.screens (dict): organized screens
        self.services (dict): ROS services - provided/requested
        self.topics (dict): ROS topics - published/subscribed
        self.display_t (object): current time
        self.display_dt (object): minimum display update interval
    """
    def __init__(self, ref_dict):
        rospy.loginfo('Initializing interface')
        self.screen_now = {}
        self.screens = {}
        self.services = {}
        self.topics = {'pub': {},'sub': {}}
        # Time to avoid excessive display update
        self.display_t = rospy.Time.now()
        self.display_dt = rospy.Duration(0.5)

        # Connect to vital services
        rospy.loginfo('Connecting to vital services')
        try:
            rospy.wait_for_service('display/write')
            self.services['display'] = rospy.ServiceProxy(
                'display/write', Display, persistent=True)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
            raise
        try:
            rospy.wait_for_service('trike/reboot')
            self.services['reboot'] = rospy.ServiceProxy(
                'trike/reboot', Empty)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
            raise
        try:
            rospy.wait_for_service('trike/kill_all')
            self.services['kill_all'] = rospy.ServiceProxy(
                'trike/kill_all', Empty)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
            raise

        self.display_write(self.format_msg('...', 0), 1, 0, True)

        # Organize the interface structure
        rospy.loginfo('Building screen structure')
        self.build_screen_group(ref_dict)
        for name, screen in self.screens.items():
            # Get the first screen displayed
            if screen['type'] == 'root':
                self.screen_now = screen
                break

        self.update_display()
        self.display_write(self.format_msg('Iniciando...', 0), 1, 0, False)

        # Connect to other services
        rospy.loginfo('Connecting to other services')
        try:
            # rospy.wait_for_service('imu/set_imu_number', timeout=30.0)
            self.services['set_imu_number'] = rospy.ServiceProxy(
                'imu/set_imu_number', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('trike/set_pulse_width', timeout=30.0)
            self.services['set_pulse_width'] = rospy.ServiceProxy(
                'trike/set_pulse_width', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            # rospy.wait_for_service('stimulator/set_frequency', timeout=30.0)
            self.services['set_stim_freq'] = rospy.ServiceProxy(
                'stimulator/set_frequency', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('trike/set_init_intensity', timeout=30.0)
            self.services['set_init_intensity'] = rospy.ServiceProxy(
                'trike/set_init_intensity', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('trike/change_intensity', timeout=30.0)
            self.services['intensity'] = rospy.ServiceProxy(
                'trike/change_intensity', SetBool, persistent=True)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('trike/set_status', timeout=30.0)
            self.services['set_status'] = rospy.ServiceProxy(
                'trike/set_status', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)

        # Setup the topics
        self.topics['sub']['cadence'] = rospy.Subscriber('trike/cadence',
            Float64, self.cadence_callback)
        self.topics['sub']['distance'] = rospy.Subscriber('trike/distance',
            Float64, self.distance_callback)
        self.topics['sub']['duration'] = rospy.Subscriber('trike/elapsed',
            Duration, self.duration_callback)
        self.topics['sub']['buttons'] = rospy.Subscriber('button/action',
            UInt8, self.button_callback)

        # Get the initial value for imu number, current, pw, freq, etc...
        rospy.loginfo('Loading parameter values')
        self.load_parameters()

        self.set_status('off')  # Make sure controller is off
        self.display_write(self.format_msg('APERTE UM BOTAO', 0), 1, 0, False)
        rospy.loginfo('Ready!')

    def shutdown(self):
        """Perform the shutdown procedures for the interface."""
        self.set_status('off')  # Make sure controller is off

    def build_screen_group(self, structure_dict, parent=''):
        """Recursively transfer the structure from the dict to the class.

        Attributes:
            structure_dict (dict): screen structure
            parent (string): upper level parent screen
        """
        if structure_dict:
            siblings = sorted(structure_dict)  # Menus in alphabetical order
            for label, menu in structure_dict.items():
                new_screen = {}
                new_screen['label'] = label
                new_screen['msg'] = menu['msg']
                new_screen['type'] = 'menu'  # menu, root, action
                new_screen['parent'] = parent
                new_screen['select'] = ''
                new_screen['next'] = ''
                new_screen['prev'] = ''
                # Get prev/next screen on same level (nearest siblings):
                position = siblings.index(label)
                if position != len(siblings)-1:  # Last item doesn't have next
                    new_screen['next'] = siblings[position+1]
                if position != 0:  # First item doesn't have previous
                    new_screen['prev'] = siblings[position-1]
                # Get where to go when selected (first child):
                if menu['submenus']:
                    new_screen['select'] = menu['submenus'].keys()[0]
                else:
                    new_screen['type'] = 'action'
                # Screen type is root:
                if (not parent) and (len(siblings) == 1):
                    new_screen['type'] = 'root'
                # Add screen to their group:
                self.screens[label] = new_screen
                self.build_screen_group(menu['submenus'], label)

    def load_parameters(self):
        """Load parameters with their initial values."""
        screen_ref = {
            'change_imu': None,
            'change_pw': None,
            'change_freq': None,
            'change_current': None
        }
        # Check services availability
        if self.services.get('set_imu_number'):
            screen_ref['change_imu'] = str(rospy.get_param('imu/wireless_id/pedal'))
        if self.services.get('set_pulse_width'):
            screen_ref['change_pw'] = str(rospy.get_param('trike/pulse_width'))
        if self.services.get('set_stim_freq'):
            screen_ref['change_freq'] = str(rospy.get_param('stimulator/freq'))
        if self.services.get('set_init_intensity'):
            screen_ref['change_current'] = str(rospy.get_param('trike/training_current'))
        # Adapt for multiple cycling modes/screens
        if self.services.get('intensity'):
            for k, v in self.screens.items():
                if k == 'cycling-C':
                    screen_ref[k] = str(rospy.get_param('trike/racing_current'))
                elif k == 'cycling-T':
                    screen_ref[k] = str(rospy.get_param('trike/training_current'))
        else:
            for k, v in self.screens.items():
                if k in {'cycling-C', 'cycling-T'}:
                    screen_ref[k] = None
        # Load the parameters to each screen msg
        for k, v in screen_ref.items():
            if v:
                msg_now = self.screens[k]['msg']
                # Adapt for multiple cycling modes/screens
                if 'cycling' in k:
                    param_new = float(v)
                    msg_new, _ = self.update_parameter(msg_now, param_new, 'mA')
                    self.screens[k]['msg'] = msg_new
                else:
                    param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str
                    self.screens[k]['msg'] = self.screens[k]['msg'].replace(param_now,v)
            else:
                line2_begin = self.screens[k]['msg'].index('\n')+1
                self.screens[k]['msg'] = self.screens[k]['msg'][:line2_begin]+'Indisponivel'

    def update_display(self):
        """Display the current screen."""
        msg_lst = self.screen_now['msg'].split('\n')
        # Chop, add the arrows and center msg:
        for idx, linemsg in enumerate(msg_lst):
            if self.screen_now['type'] == 'menu':
                linemsg = self.format_msg(linemsg, 2)
                if self.screen_now['prev']:
                    linemsg = '< '+linemsg
                else:
                    linemsg = '  '+linemsg
                if self.screen_now['next']:
                    linemsg += ' >'
                else:
                    linemsg += '  '
            else:
                linemsg = self.format_msg(linemsg, 0)
            self.display_write(linemsg, idx, 0, len(msg_lst)<2)

    def format_msg(self, msg, margin):
        """Format msg according to display size and desired margin.

        Attributes:
            msg (string): the msgs to be formatted
            margin (int): symmetric blank space amount for side padding
        """
        msg = msg[0:(DISPLAY_SIZE[1]-(2*margin))]
        remainder = DISPLAY_SIZE[1]-(2*margin)-len(msg)
        paddingL = remainder/2
        paddingR = (remainder/2)+(remainder%2)
        msg = (paddingL*' ')+msg+(paddingR*' ')
        return msg

    def update_parameter(self, string, param_new, unit=''):
        """Identify a certain number by its unit, modify its value if
        different and return updated string and changed line. The number
        format is maintained and values greater than it allows are limited,
        e.g. 99.9 if new is 100.1 and old was 99.8 

        Attributes:
            string (str): string to be searched and modified
            param_new (int/float): updated parameter value
            unit (str): unit of measurement
        """
        # Only non negative values
        if param_new < 0:
            param_new = 0
        string_split = string.split()  # Split items
        split_lines = string.split('\n')  # Split lines
        # Get old value from string
        if unit:
            contains_unit = [item for item in string_split if unit in item]
            if not contains_unit:
                return string, None
            idx = 0
            # Deal with substrings and unit variants like km/h and km
            if len(contains_unit) > 1:
                for idx, item in enumerate(contains_unit):
                    letters = ''.join([char for char in item if char.isalpha()])
                    if letters == unit:
                        break
            param_string = contains_unit[idx]  # Old value with unit
            param_string = param_string[:param_string.index(unit)+len(unit)]  # Strip mixed like 00'00"
            temp = param_string.replace(unit,'')
            temp = temp.replace('.','0')
            try:
                temp = [idx for idx, char in enumerate(temp) if not char.isdigit()]
                param_string = param_string[temp[-1]+1:]
            except IndexError as e:
                pass
            param_now = param_string.replace(unit,'')  # Old value only
        else:  # Consider only one number in string
            param_string = [x for x in string_split if x.replace('.','',1).isdigit()][0]
            if not param_string:
                return string, None
            param_now = param_string  # Old value is the same as string
        # Check if new value is valid and format it
        if '.' in param_now:  # Decimal
            left_format = len(param_now[:param_now.index('.')])  # Digits to the left of '.'
            right_format = len(param_now[param_now.index('.')+1:])  # Digits to the right of '.'
            # Format the new value as an attempt to replace
            attempt = ('{:0'+str(left_format+1+right_format)+'.'+str(right_format)+'f}').format(param_new)
            # No difference in value so no update
            if attempt == param_now:
                return string, None
            limit = float(('9'*left_format)+'.'+('9'*right_format))  # Format limitation
            param_new = str(limit) if param_new > limit else attempt
        else:  # No decimal
            param_new = int(round(param_new))
            char_width = len(param_now)
            # Format the new value as an attempt to replace
            attempt = str(param_new).zfill(char_width)  # Leading zeros
            # No difference in value so no update
            if attempt == param_now:
                return string, None
            limit = int(char_width*'9')  # Format limitation
            param_new = str(limit) if param_new > limit else attempt
        # Return the updated string and the number of the changed line
        changed_line = [idx for idx, s in enumerate(split_lines) if param_string in s][0]
        updated_string = string.replace(param_string, param_new+unit)
        return updated_string, changed_line

    def cadence_callback(self, data):
        """ROS Topic callback to get the crankset cadence.

        Attributes:
            data (Float64): latest msg for cadence in km/h
        """
        if self.screen_now['label'] in {'cycling-C', 'cycling-T'}:
            param_new = data.data
            if param_new < 0:
                param_new = float(0)
            # Update displayed value
            msg_now = self.screen_now['msg']
            msg_new, line = self.update_parameter(msg_now, param_new, 'km/h')
            if line is not None:  # None if there was no change
                self.screen_now['msg'] = msg_new
                # split_lines = msg_new.split('\n')
                # update = self.format_msg(split_lines[line], 0)
                # # Avoid excessive display update
                # if (rospy.Time.now()-self.display_t) > self.display_dt:
                #     self.display_write(update, line, 0, False)

    def distance_callback(self, data):
        """ROS Topic callback to get the estimated distance travelled.

        Attributes:
            data (Float64): latest msg for distance in km
        """
        if self.screen_now['label'] in {'cycling-C', 'cycling-T'}:
            param_new = data.data
            if param_new < 0:
                param_new = float(0)
            # Update displayed value
            msg_now = self.screen_now['msg']
            msg_new, line = self.update_parameter(msg_now, param_new, 'km')
            if line is not None:  # None if there was no change
                self.screen_now['msg'] = msg_new

    def duration_callback(self, data):
        """ROS Topic callback to get the cycling duration.

        Attributes:
            data (Duration): latest msg for elapsed time in rospy.Duration
        """
        if self.screen_now['label'] in {'cycling-C', 'cycling-T'}:
            param_new = abs(data.data.to_sec())
            t_min, t_sec = divmod(param_new, 60)
            # Update displayed value
            msg_now = self.screen_now['msg']
            # Update for min and sec assuming they are on the same line
            msg_new, line1 = self.update_parameter(msg_now, int(t_min), '\'')
            msg_new, line2 = self.update_parameter(msg_new, int(t_sec), '"')
            line = next((line for line in [line1, line2] if line is not None), None)
            if line is not None:  # None if there was no change
                self.screen_now['msg'] = msg_new
                # Update the display every second
                self.update_display()  # Cadence and distance will be updated here as well 

    def button_callback(self, data):
        """ROS Topic callback to classify button commands from the user
        and act accordingly.

        Attributes:
            data (UInt8): latest msg from the buttons
        """
        data = data.data  # Bring out the command
        if data == 1:
            self.output_response('single_left')
        elif data == 2:
            self.output_response('single_right')
        elif data == 3:
            self.output_response('both')
        elif data == 4:
            self.output_response('double_left')
        elif data == 5:
            self.output_response('double_right')
        else:
            rospy.logwarn('Invalid button input.')

    def output_response(self, action):
        """Output based on user input and current screen.

        Attributes:
            action (string): interpreted user action
        """
        if self.screen_now['type'] == 'root':
            if action == 'both':
                self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
                self.kill_all()
                rospy.sleep(3)  # Avoid changing the display
                return
            else:
                self.screen_now = self.screens[self.screen_now['select']]
        elif self.screen_now['type'] == 'menu':
            try:
                if action == 'both':  # Kill all nodes and rely on launch respawn
                    self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
                    self.kill_all()
                    rospy.sleep(3)  # Avoid changing the display
                elif action == 'single_left':  # Previous same level menu
                    output_screen_name = self.screen_now['prev']
                    self.screen_now = self.screens[output_screen_name]
                elif action == 'single_right':  # Next same level menu
                    output_screen_name = self.screen_now['next']
                    self.screen_now = self.screens[output_screen_name]
                elif action == 'double_left':  # Previous upper level menu
                    output_screen_name = self.screen_now['parent']
                    self.screen_now = self.screens[output_screen_name]
                elif action == 'double_right':  # Next down level menu
                    output_screen_name = self.screen_now['select']
                    self.screen_now = self.screens[output_screen_name]
                    # Turn control on when cycling screen is selected
                    if self.screen_now['label'] == 'cycling-C':
                        self.set_status('racing')  # Racing mode
                    elif self.screen_now['label'] == 'cycling-T':
                        self.set_status('training')  # Training mode
            except KeyError as e:  # e.g. the menu doesnt have a next
                pass
        elif self.screen_now['type'] == 'action':
            if self.screen_now['label'] == 'change_imu':
                self.change_imu(action)
                return
            elif self.screen_now['label'] == 'change_pw':
                self.change_pw(action)
                return
            elif self.screen_now['label'] == 'change_freq':
                self.change_freq(action)
                return
            elif self.screen_now['label'] == 'change_current':
                self.change_current(action)
                return
            elif self.screen_now['label'] in {'cycling-C', 'cycling-T'}:
                self.cycling(action)
                return
            elif self.screen_now['label'] == 'reboot_screen':
                self.reboot_screen(action)
                return
        self.update_display()
        return

    def change_imu(self, button):
        """Deal with user action on the change imu number screen.

        Attributes:
            button (string): describes button event
        """
        # Get present screen msg and isolate its present param
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str
        # Change the displayed value
        if button in {'single_left', 'single_right'}:
            if param_now:  # Param value exists
                request = -1 if button == 'single_left' else 1  # Drecease or increase
                attempt = int(param_now)+request
                if attempt > 0 and attempt <= 10:
                    param_new = str(attempt)
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                    line2_begin = self.screen_now['msg'].index('\n')+1
                    update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                    self.display_write(update, 1, 0, False)
        # Kill all nodes and rely on launch respawn
        elif button == 'both':
            self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
        # Check param and go to parent, previous upper level menu
        elif button == 'double_left':
            if param_now:  # Param value exists
                param_new = str(rospy.get_param('imu/wireless_id/pedal'))
                self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.screen_now = self.screens[self.screen_now['parent']]
            self.update_display()
        # Confirm modification
        elif button == 'double_right':
            stat = 'ERRO'
            if param_now:  # Param value exists
                result = self.set_imu_number(int(param_now))
                if result:
                    self.screen_now['msg'] = msg_now.replace(param_now,result)
                    stat = 'OK'
                else:
                    # If error reset the value
                    param_new = str(rospy.get_param('imu/wireless_id/pedal'))
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.screen_now = self.screens[self.screen_now['parent']]
            self.display_write(self.format_msg(stat, 0), 1, 0, True)
            rospy.sleep(5)
            self.update_display()

    def change_pw(self, button):
        """Deal with user action on the change pw screen.

        Attributes:
            button (string): describes button event
        """
        # Get present screen msg and isolate its present param
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str
        # Change the displayed value
        if button in {'single_left', 'single_right'}:
            if param_now:  # Param value exists
                request = -10 if button == 'single_left' else 10  # Drecease or increase
                attempt = int(param_now)+request
                if attempt >= 0 and attempt <= 500:
                    param_new = str(attempt)
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                    line2_begin = self.screen_now['msg'].index('\n')+1
                    update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                    self.display_write(update, 1, 0, False)
        # Kill all nodes and rely on launch respawn
        elif button == 'both':
            self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
        # Check param and go to parent, previous upper level menu
        elif button == 'double_left':
            if param_now:  # Param value exists
                param_new = str(rospy.get_param('trike/pulse_width'))
                self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.screen_now = self.screens[self.screen_now['parent']]
            self.update_display()
        # Confirm modification
        elif button == 'double_right':
            stat = 'ERRO'
            if param_now:  # Param value exists
                result = self.set_pulse_width(int(param_now))
                if result:
                    self.screen_now['msg'] = msg_now.replace(param_now,result)
                    stat = 'OK'
                else:
                    # If error reset the value
                    param_new = str(rospy.get_param('trike/pulse_width'))
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.screen_now = self.screens[self.screen_now['parent']]
            self.display_write(self.format_msg(stat, 0), 1, 0, True)
            rospy.sleep(5)
            self.update_display()

    def change_freq(self, button):
        """Deal with user action on the change frequency screen.

        Attributes:
            button (string): describes button event
        """
        # Get present screen msg and isolate its present param
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str
        # Change the displayed value
        if button in {'single_left', 'single_right'}:
            if param_now:  # Param value exists
                request = -1 if button == 'single_left' else 1  # Drecease or increase
                attempt = int(param_now)+request
                if attempt >= 0 and attempt <= 100:
                    param_new = str(attempt)
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                    line2_begin = self.screen_now['msg'].index('\n')+1
                    update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                    self.display_write(update, 1, 0, False)
        # Kill all nodes and rely on launch respawn
        elif button == 'both':
            self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
        # Check param and go to parent, previous upper level menu
        elif button == 'double_left':
            if param_now:  # Param value exists
                param_new = str(rospy.get_param('stimulator/freq'))
                self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.screen_now = self.screens[self.screen_now['parent']]
            self.update_display()
        # Confirm modification
        elif button == 'double_right':
            stat = 'ERRO'
            if param_now:  # Param value exists
                result = self.set_stim_freq(int(param_now))
                if result:
                    self.screen_now['msg'] = msg_now.replace(param_now,result)
                    stat = 'OK'
                else:
                    # If error reset the value
                    param_new = str(rospy.get_param('stimulator/freq'))
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.screen_now = self.screens[self.screen_now['parent']]
            self.display_write(self.format_msg(stat, 0), 1, 0, True)
            rospy.sleep(5)
            self.update_display()

    def change_current(self, button):
        """Deal with user action on the change current screen.

        Attributes:
            button (string): describes button event
        """
        # Get present screen msg and isolate its present param
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str
        # Change the displayed value
        if button in {'single_left', 'single_right'}:
            if param_now:  # Param value exists
                request = -2 if button == 'single_left' else 2  # Drecease or increase
                attempt = int(param_now)+request
                if attempt >= 0 and attempt <= 120:
                    param_new = str(attempt)
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                    line2_begin = self.screen_now['msg'].index('\n')+1
                    update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                    self.display_write(update, 1, 0, False)
        # Kill all nodes and rely on launch respawn
        elif button == 'both':
            self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
        # Check param and go to parent, previous upper level menu
        elif button == 'double_left':
            if param_now:  # Param value exists
                param_new = str(rospy.get_param('trike/training_current'))
                self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.screen_now = self.screens[self.screen_now['parent']]
            self.update_display()
        # Confirm modification
        elif button == 'double_right':
            stat = 'ERRO'
            if param_now:  # Param value exists
                result = self.set_init_intensity(int(param_now))
                if result:
                    self.screen_now['msg'] = msg_now.replace(param_now,result)
                    stat = 'OK'
                    # Replace on training screen as well
                    cycling_msg = self.screens['cycling-T']['msg']
                    param_new = int(result)
                    msg_new, line = self.update_parameter(cycling_msg, param_new, 'mA')
                    if line is not None:  # None if there was no change
                        self.screens['cycling-T']['msg'] = msg_new
                else:
                    # If error reset the value
                    param_new = str(rospy.get_param('trike/training_current'))
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.screen_now = self.screens[self.screen_now['parent']]
            self.display_write(self.format_msg(stat, 0), 1, 0, True)
            rospy.sleep(5)
            self.update_display()

    def cycling(self, button):
        """Deal with user action on the cycling training screen.

        Attributes:
            button (string): describes button event
        """
        # Kill all nodes and rely on launch respawn
        if button == 'both':
            self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
            self.set_status('off')  # Make sure controller is off
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
            return
        # Request a change in intensity and modify the displayed value
        else:
            # Incapable of changing param
            if 'Indisponivel' in self.screen_now['msg']:
                return
            # Replace param value and update display
            request = 0 if button in {'single_left', 'double_left'} else 1  # Drecease or increase
            result = self.change_intensity(request)
            if result:
                msg_now = self.screen_now['msg']
                param_new = int(result)
                msg_new, line = self.update_parameter(msg_now, param_new, 'mA')
                if line is not None:  # None if there was no change
                    self.screen_now['msg'] = msg_new
                    split_lines = msg_new.split('\n')
                    update = self.format_msg(split_lines[line], 0)
                    # Avoid excessive display update
                    if (rospy.Time.now()-self.display_t) > self.display_dt:
                        self.display_write(update, line, 0, False)
        return

    def reboot_screen(self, button):
        """Deal with user action on the reboot screen.

        Attributes:
            button (string): describes button event
        """
        # Kill all nodes and rely on launch respawn
        if button == 'both':
            self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
        # Go to parent, previous upper level menu
        elif button == 'double_left':
            self.screen_now = self.screens[self.screen_now['parent']]
            self.update_display()
        # Confirm reboot or display error msg
        elif button == 'double_right':
            result = self.reboot()
            if result:
                self.display_write(self.format_msg('Aguarde...', 0), 1, 0, True)
                rospy.sleep(3)  # Avoid changing the display
            else:
                self.screen_now = self.screens[self.screen_now['parent']]
                self.display_write(self.format_msg('ERRO', 0), 1, 0, True)
                rospy.sleep(5)
                self.update_display()

    def display_write(self, msg, line, position, clear):
        """Call a ROS Service to write to the display.

        Attributes:
            msg (string): the msgs to be displayed
            line (int): the display line to print, 0 is top
            position (int): the display column to print, 0 is left
            clear (bool): to clear display or not
        """
        try:
            rospy.wait_for_service('display/write', timeout=1.0)
            resp = self.services['display'](message=msg, line=line,
                position=position, clear=clear)
            self.display_t = rospy.Time.now()
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return

    def reboot(self):
        """Call a ROS Service to reboot the machine."""
        try:
            rospy.wait_for_service('trike/reboot')
            self.services['reboot']()
            return True
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return False

    def kill_all(self):
        """Call a ROS Service to shutdown all nodes."""
        try:
            rospy.wait_for_service('trike/kill_all')
            self.services['kill_all']()
            return True
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return False

    def set_imu_number(self, req):
        """Call a ROS Service to set a different IMU number.

        Attributes:
            req (int): new IMU number from 0 to 10
        """
        try:
            rospy.wait_for_service('imu/set_imu_number', timeout=1.0)
            resp = self.services['set_imu_number'](req)
            return resp.message  # Return the IMU now as str
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return

    def set_pulse_width(self, req):
        """Call a ROS Service to set the stim pulse width.

        Attributes:
            req (int): new pulse width
        """
        try:
            rospy.wait_for_service('trike/set_pulse_width', timeout=1.0)
            resp = self.services['set_pulse_width'](req)
            return resp.message  # Return the pulse width as str
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return

    def set_stim_freq(self, req):
        """Call a ROS Service to set the stim frequency.

        Attributes:
            req (int): new frequency
        """
        try:
            rospy.wait_for_service('stimulator/set_frequency', timeout=1.0)
            resp = self.services['set_stim_freq'](req)
            return resp.message  # Return the IMU now as str
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return

    def set_init_intensity(self, req):
        """Call a ROS Service to set the initial stim intensity.

        Attributes:
            req (int): new initial intensity
        """
        try:
            rospy.wait_for_service('trike/set_init_intensity', timeout=1.0)
            resp = self.services['set_init_intensity'](req)
            return resp.message  # Return the intensity as str
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return
        return

    def change_intensity(self, req):
        """Call a ROS Service to request a change in intensity.

        Attributes:
            req (bool): 0 to decrease and 1 to increase
        """
        try:
            rospy.wait_for_service('trike/change_intensity', timeout=1.0)
            resp = self.services['intensity'](req)
            return resp.message  # Return the intensity now as str
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return

    def set_status(self, req):
        """Call a ROS Service to change the cycling mode.

        Attributes:
            req (UInt16): 'off', 'training' or 'racing'
        """
        enum = ['off','training','racing']
        try:
            rospy.wait_for_service('trike/set_status', timeout=1.0)
            resp = self.services['set_status'](enum.index(req))
            return resp.message  # Return the mode
        except (rospy.ServiceException, rospy.ROSException, IndexError) as e:
            rospy.logerr(e)
        return


def main():
    # Init display node
    rospy.loginfo('Initializing node')
    rospy.init_node('interface')
    
    # Create interface auxiliary class
    rospy.loginfo('Creating auxiliary class')
    aux = Interface(menu_ref)

    # Prepare function to be executed when shutting down
    rospy.on_shutdown(aux.shutdown)

    # Keep python from exiting until the node stops
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
