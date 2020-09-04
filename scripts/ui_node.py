#!/usr/bin/env python

"""

Interface draft 

The 'menu_ref' dictionary defines the menu/screen structure and the
corresponding interaction between them. Each screen has its message and
submenus, which also have their messages and submenus. The 'welcome'
screen is the root and the screens that have submenus as None are the
leaves, or action screens, where the user is prompt to make changes.

"""

import rospy

# Import ros msgs:
from std_srvs.srv import SetBool
from ema_common_msgs.srv import Display
from ema_common_msgs.srv import SetUInt16

# import utilities:
import os

# Declare constants:
DISPLAY_SIZE = (2,16)

# Dictionary with interface structure:
menu_ref = {
    'welcome': {
        'msg':'~ EMA Trike ~',
        'submenus': {
            'config': {
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
            'init': {
                'msg':'Iniciar',
                'submenus': {
                    'cycling': {
                        'msg':'Corrente:\n0 mA',
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
        self.screens (dict): dict with all screens
        self.services (dict): dict with all services
    """
    def __init__(self, ref_dict):
        rospy.loginfo('Initializing interface...')
        self.screen_now = {}
        self.screens = {}
        self.services = {}

        # Organize the interface structure
        rospy.loginfo('Building screen structure...')
        self.build_screen_group(ref_dict)
        for name, screen in self.screens.items():
            if screen['type'] == 'root':
                self.screen_now = screen
                break

        # List services:
        try:
            rospy.loginfo('Checking and establishing services...')

            rospy.wait_for_service('display/write')
            self.services['display'] = rospy.ServiceProxy(
                'display/write', Display, persistent=True)

            # rospy.wait_for_service('imu/set_imu_number')
            # self.services['set_imu_number'] = rospy.ServiceProxy(
            #     'imu/set_imu_number', SetUInt16)

            rospy.wait_for_service('control/set_pulse_width')
            self.services['set_pulse_width'] = rospy.ServiceProxy(
                'control/set_pulse_width', SetUInt16)

            # rospy.wait_for_service('stimulator/set_freq')
            # self.services['set_stim_freq'] = rospy.ServiceProxy(
            #     'stimulator/set_freq', SetUInt16)

            rospy.wait_for_service('control/set_init_intensity')
            self.services['set_init_intensity'] = rospy.ServiceProxy(
                'control/set_init_intensity', SetUInt16)

            rospy.wait_for_service('control/change_intensity')
            self.services['intensity'] = rospy.ServiceProxy(
                'control/change_intensity', SetBool, persistent=True)

            rospy.wait_for_service('control/on_off')
            self.services['on_off'] = rospy.ServiceProxy(
                'control/on_off', SetBool)

        except rospy.ServiceException as e:
            rospy.logerr(e)

        rospy.loginfo('Loading parameter values...')
        self.update_parameters()
        self.update_display()
        rospy.loginfo('Ready!')

    def display_write(self, msg, line, position, clear):
        """Calls a ROS Service to write to the display

        Attributes:
            msg (string): the messages to be displayed
            line (int): the display line to print, 0 is top
            position (int): the display column to print, 0 is left
            clear (bool): to clear display or not
        """
        rospy.wait_for_service('display/write')
        try:
            resp = self.services['display'](message=msg, line=line,
                position=position, clear=clear)
        except rospy.ServiceException as e:
            self.services['display'] = rospy.ServiceProxy('display/write',
                Display, persistent=True)
            rospy.logwarn(e)
        return

    def set_imu_number(self, req):
        """Calls a ROS Service to set a different IMU number

        Attributes:
            req (int): new IMU number from 0 to 10
        """
        rospy.wait_for_service('imu/set_imu_number')
        try:
            resp = self.services['set_imu_number'](req)
            return resp.message  # Return the IMU now as str
        except rospy.ServiceException as e:
            rospy.logwarn(e)
        return

    def set_pulse_width(self, req):
        """Calls a ROS Service to set the stim pulse width

        Attributes:
            req (int): new pulse width
        """
        rospy.wait_for_service('control/set_pulse_width')
        try:
            resp = self.services['set_pulse_width'](req)
            return resp.message  # Return the pulse width as str
        except rospy.ServiceException as e:
            rospy.logwarn(e)
        return

    def set_freq(self, req):
        """Calls a ROS Service to set the stim frequency

        Attributes:
            req (int): new frequency
        """

        # TODO
        return

    def set_init_intensity(self, req):
        """Calls a ROS Service to set the initial stim intensity

        Attributes:
            req (int): new initial intensity
        """
        rospy.wait_for_service('control/set_init_intensity')
        try:
            resp = self.services['set_init_intensity'](req)
            return resp.message  # Return the intensity as str
        except rospy.ServiceException as e:
            rospy.logwarn(e)
        return
        return

    def change_intensity(self, req):
        """Calls a ROS Service to request a change in intensity

        Attributes:
            req (bool): 0 to decrease and 1 to increase
        """
        rospy.wait_for_service('control/change_intensity')
        try:
            resp = self.services['intensity'](req)
            return resp.message  # Return the intensity now as str
        except rospy.ServiceException as e:
            self.services['intensity'] = rospy.ServiceProxy('control/change_intensity',
                SetBool, persistent=True)
            rospy.logwarn(e)
        return

    def on_off(self, req):
        """Calls a ROS Service to turn control on/off

        Attributes:
            req (bool): 0 to turn off and 1 to turn on
        """
        rospy.wait_for_service('control/on_off')
        try:
            resp = self.services['on_off'](req)
            return resp.message  # Return the control state (on/off)
        except rospy.ServiceException as e:
            rospy.logwarn(e)
        return

    def build_screen_group(self, structure_dict, parent=''):
        """Recursively transfer the structure from the dict to the class.

        Attributes:
            structure_dict (dict): screen structure
            parent (string): upper level parent screen
        """
        if structure_dict:
            siblings = list(structure_dict)
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
        return

    def update_parameters(self):
        """Load parameters with their initial values."""
        imu = str(rospy.get_param('imu/wireless_id/pedal'))
        pw = '500'
        freq = str(rospy.get_param('stimulator/freq'))
        current = str(rospy.get_param('control/initial_current'))

        aux = {
            'change_imu': imu,
            'change_pw': pw,
            'change_freq': freq,
            'change_current': current,
            'cycling': current
        }

        for k, v in aux.items():
            param_now = ''.join(x for x in self.screens[k]['msg'] if x.isdigit())  # int in str
            self.screens[k]['msg'] = self.screens[k]['msg'].replace(param_now,v)
        return

    def translate_input(self, key):
        """Classify the user input and act accordingly.

        Attributes:
            key (int): key pressed
        """
        if key == 1:
            return 'single_left'
        elif key == 2:
            return 'single_right'
        elif key == 11:
            return 'double_left'
        elif key == 22:
            return 'double_right'
        elif (key == 12) or (key == 21):
            return 'both'
        else:
            return

    def output_response(self, action):
        """Output based on user input and current screen.

        Attributes:
            action (string): interpreted user action
        """
        if self.screen_now['type'] == 'root':
            if action == 'both':
                output_screen_name = 'welcome'
                self.screen_now = self.screens[output_screen_name]
            else:
                self.screen_now = self.screens[self.screen_now['select']]

        elif self.screen_now['type'] == 'menu':
            try:
                if action == 'both':  # Reset
                    output_screen_name = 'welcome'
                    self.screen_now = self.screens[output_screen_name]
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
            except KeyError:
                pass

        elif self.screen_now['type'] == 'action':
            try:
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
                elif self.screen_now['label'] == 'cycling':
                    self.cycling(action)
                    return
            except KeyError:
                pass

        self.update_display()
        return

    def format_msg(self, msg, margin):
        """Format message according to display size and desired margin.

        Attributes:
            msg (string): the messages to be formatted
            margin (int): symmetric blank space amount for side padding
        """
        msg = msg[0:(DISPLAY_SIZE[1]-(2*margin))]
        remainder = DISPLAY_SIZE[1]-(2*margin)-len(msg)
        paddingL = remainder/2
        paddingR = (remainder/2)+(remainder%2)
        msg = (paddingL*' ')+msg+(paddingR*' ')
        return msg

    def update_display(self):
        """Display the current screen."""
        if self.screen_now['label'] == 'cycling':
            resp = self.on_off(True)

        msg_lst = self.screen_now['msg'].split('\n')

        # Chop and center msg:
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

            self.display_write(linemsg, idx, 0, (not idx))

    def change_imu(self, button):
        """Deal with user action on the change imu screen.

        Attributes:
            button (string): describes button event
        """
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str

        if button == 'both':  # Reset
            param_new = str(rospy.get_param('imu/wireless_id/pedal'))
            self.screen_now['msg'] = msg_now.replace(param_now,param_new)

            output_screen_name = 'welcome'
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        elif button == 'double_left':  # Previous upper level menu
            param_new = str(rospy.get_param('imu/wireless_id/pedal'))
            self.screen_now['msg'] = msg_now.replace(param_now,param_new)

            output_screen_name = self.screen_now['parent']
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        elif button == 'double_right':  # Confirm modification
            result = self.set_imu_number(int(param_now))
            if result:
                self.screen_now['msg'] = msg_now.replace(param_now,result)
        else:  # single_left or single_right
            request = -1 if button == 'single_left' else 1  # Drecease or increase
            attempt = int(param_now)+request
            if attempt > 0 and attempt <= 10:
                param_new = str(attempt)
                self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                line2_begin = self.screen_now['msg'].index('\n')+1
                update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                self.display_write(update, 1, 0, False)
        return

    def change_pw(self, button):
        """Deal with user action on the change pw screen.

        Attributes:
            button (string): describes button event
        """
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str

        if button == 'both':  # Reset
            param_new = str(rospy.get_param('control/pulse_width'))
            self.screen_now['msg'] = msg_now.replace(param_now,param_new)

            output_screen_name = 'welcome'
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        elif button == 'double_left':  # Previous upper level menu
            param_new = str(rospy.get_param('control/pulse_width'))
            self.screen_now['msg'] = msg_now.replace(param_now,param_new)

            output_screen_name = self.screen_now['parent']
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        elif button == 'double_right':  # Confirm modification
            result = self.set_pulse_width(int(param_now))
            if result:
                self.screen_now['msg'] = msg_now.replace(param_now,result)
        else:  # single_left or single_right
            request = -10 if button == 'single_left' else 10  # Drecease or increase
            attempt = int(param_now)+request
            if attempt >= 0 and attempt <= 500:
                param_new = str(attempt)
                self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                line2_begin = self.screen_now['msg'].index('\n')+1
                update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                self.display_write(update, 1, 0, False)
        return

    def change_freq(self, button):
        """Deal with user action on the change frequency screen.

        Attributes:
            button (string): describes button event
        """
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str

        if button == 'both':  # Reset
            output_screen_name = 'welcome'
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        elif button == 'double_left':  # Previous upper level menu
            output_screen_name = self.screen_now['parent']
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        return

    def change_current(self, button):
        """Deal with user action on the change current screen.

        Attributes:
            button (string): describes button event
        """
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str

        if button == 'both':  # Reset
            param_new = str(rospy.get_param('control/initial_current'))
            self.screen_now['msg'] = msg_now.replace(param_now,param_new)

            output_screen_name = 'welcome'
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        elif button == 'double_left':  # Previous upper level menu
            param_new = str(rospy.get_param('control/initial_current'))
            self.screen_now['msg'] = msg_now.replace(param_now,param_new)

            output_screen_name = self.screen_now['parent']
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        elif button == 'double_right':  # Confirm modification
            result = self.set_init_intensity(int(param_now))
            if result:
                self.screen_now['msg'] = msg_now.replace(param_now,result)
                p = ''.join(x for x in self.screens['cycling']['msg'] if x.isdigit())  # int in str
                self.screens['cycling']['msg'] = msg_now.replace(p,result)
        else:  # single_left or single_right
            request = -2 if button == 'single_left' else 2  # Drecease or increase
            attempt = int(param_now)+request
            if attempt >= 0 and attempt <= 120:
                param_new = str(attempt)
                self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                line2_begin = self.screen_now['msg'].index('\n')+1
                update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                self.display_write(update, 1, 0, False)
        return

    def cycling(self, button):
        """Deal with user action on the cycling training screen.

        Attributes:
            button (string): describes button event
        """
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str

        if button == 'both':  # Reset
            param_new = str(rospy.get_param('control/initial_current'))
            self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.on_off(False)

            output_screen_name = 'welcome'
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        elif button == 'double_left':  # Previous upper level menu
            param_new = str(rospy.get_param('control/initial_current'))
            self.screen_now['msg'] = msg_now.replace(param_now,param_new)
            self.on_off(False)

            output_screen_name = self.screen_now['parent']
            self.screen_now = self.screens[output_screen_name]
            self.update_display()
        else:  # single_left or single_right
            request = 0 if button == 'single_left' else 1  # Drecease or increase
            result = self.change_intensity(request)
            if result:
                self.screen_now['msg'] = msg_now.replace(param_now,result)
                line2_begin = self.screen_now['msg'].index('\n')+1
                update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                self.display_write(update, 1, 0, False)
            return
        return

def main():
    # Init display node:
    rospy.init_node('interface')
    
    # Create interface auxiliary class:
    aux = Interface(menu_ref)

    # Node loop:
    while not rospy.is_shutdown():
        try:
            key = input()
            action = aux.translate_input(key)
            aux.output_response(action)

        except (NameError, SyntaxError, EOFError):
            pass

    # Call service to turn off control
    aux.on_off(False)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
