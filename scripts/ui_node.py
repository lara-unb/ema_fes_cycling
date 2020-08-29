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
from ema_common_msgs.srv import display

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
                                'msg':'IMU:',
                                'submenus': {}
                            }
                        }
                    },
                    'pw': {
                        'msg':'Larg. Pulso',
                        'submenus': {
                            'change_pw': {
                                'msg':'Larg. Pulso:',
                                'submenus': {}
                            }
                        }
                    },
                    'freq': {
                        'msg':'Frequencia',
                        'submenus': {
                            'change_freq': {
                                'msg':'Frequencia:',
                                'submenus': {}
                            }
                        }
                    },
                    'current': {
                        'msg':'Corrente',
                        'submenus': {
                            'change_current': {
                                'msg':'Corrente:',
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
                        'msg':'Corrente:',
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
        self.screen_now = {}
        self.screens = {}
        self.services = {}

        # Organize the interface structure
        self.build_screen_group(ref_dict)
        for name, screen in self.screens.items():
            if screen['type'] == 'root':
                self.screen_now = screen
                break

        # List services:
        try:
            rospy.wait_for_service('control/change_intensity')
            self.services['intensity'] = rospy.ServiceProxy('control/change_intensity',
                SetBool, persistent=True)
            rospy.wait_for_service('display/write')
            self.services['display'] = rospy.ServiceProxy('display/write',
                display, persistent=True)
        except rospy.ServiceException as e:
            rospy.logerr(e)

        self.initialize()
        self.update_display()

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

    def initialize(self):
        """Load parameters with their initial values."""

        imu = str(rospy.get_param('imu/wireless_id').values()[0])
        pw = '500'
        freq = str(rospy.get_param('stimulator/freq'))
        current = str(rospy.get_param('control/initial_current'))

        self.screens['change_imu']['msg'] += '\n'+imu
        self.screens['change_pw']['msg'] += '\n'+pw+' us'
        self.screens['change_freq']['msg'] += '\n'+freq+' Hz'
        self.screens['change_current']['msg'] += '\n'+current+' mA'
        self.screens['cycling']['msg'] += '\n'+current+' mA'

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

    def display_output(self, msg, line, position, clear):
        """Send screen content to the display.

        Attributes:
            msg (string): the messages to be displayed
            line (int): the display line to print, 0 is top
            position (int): the display column to print, 0 is left
            clear (bool): to clear display or not
        """
        rospy.wait_for_service('display/write')
        try:
            resp = self.services['display'](message=msg,line=line,
                position=position,clear=clear)
        except rospy.ServiceException as e:
            self.services['display'] = rospy.ServiceProxy('display/write',
                display, persistent=True)
            rospy.logwarn(e)
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
                if action == 'single_left':  # Previous same level menu
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
                elif action == 'both':  # Reset
                    output_screen_name = 'welcome'
                    self.screen_now = self.screens[output_screen_name]
            except KeyError:
                pass

        elif self.screen_now['type'] == 'action':
            try:
                if (action == 'single_left') or (action == 'single_right'):
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
                elif action == 'double_left':  # Previous upper level menu
                    output_screen_name = self.screen_now['parent']
                    self.screen_now = self.screens[output_screen_name]
                elif action == 'both':  # Reset
                    output_screen_name = 'welcome'
                    self.screen_now = self.screens[output_screen_name]
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

            self.display_output(linemsg, idx, 0, (not idx))

    def change_imu(self, button):
        """Deal with user action on the change imu screen.

        Attributes:
            button (string): describes button event
        """

    def change_pw(self, button):
        """Deal with user action on the change pw screen.

        Attributes:
            button (string): describes button event
        """

    def change_freq(self, button):
        """Deal with user action on the change frequency screen.

        Attributes:
            button (string): describes button event
        """

    def change_current(self, button):
        """Deal with user action on the change current screen.

        Attributes:
            button (string): describes button event
        """

    def cycling(self, button):
        """Deal with user action on the cycling training screen.

        Attributes:
            button (string): describes button event
        """
        request = 0 if button == 'single_left' else 1  # Drecease or increase
        rospy.wait_for_service('control/change_intensity')
        try:
            resp = self.services['intensity'](request)
        except rospy.ServiceException as e:
            self.services['intensity'] = rospy.ServiceProxy('control/change_intensity',
                SetBool, persistent=True)
            rospy.logwarn(e)
        if resp.success:
            new_msg = resp.message+' mA'
            end = self.screen_now['msg'].index('\n')+1
            self.screen_now['msg'] = self.screen_now['msg'][0:end]+new_msg
            new_msg = self.format_msg(new_msg, 0)
            self.display_output(new_msg, 1, 0, False)
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

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
