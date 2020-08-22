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
from ema_common_msgs.msg import Stimulator

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
        self.output_screen('  Iniciando...\n')
        self.screen_now = {}
        self.screens = {}
        self.services = {}

        # Organize the interface structure
        self.output_screen('  UI...\n')
        self.build_screen_group(ref_dict)
        for name, screen in self.screens.items():
            if screen['type'] == 'root':
                self.screen_now = screen
                break

        # List services:
        self.output_screen('  Servicos...\n')
        try:
            rospy.wait_for_service('control/change_intensity')
            self.services['intensity'] = rospy.ServiceProxy('control/change_intensity',
                SetBool, persistent=True)
        except rospy.ServiceException as e:
            rospy.logerr(e)

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

    def output_screen(self, msg):
        """Format and send screen content to the display.

        Attributes:
            msg (string): the message to be displayed
        """
        os.system('cls' if os.name == 'nt' else 'clear')
        print '++++++++++++++++'
        print msg
        print '++++++++++++++++'
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
                    elif self.screen_now['label'] == 'change_pw':
                        self.change_pw(action)
                    elif self.screen_now['label'] == 'change_freq':
                        self.change_freq(action)
                    elif self.screen_now['label'] == 'change_current':
                        self.change_current(action)
                    elif self.screen_now['label'] == 'cycling':
                        self.cycling(action)
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

    def update_display(self):
        """Display the current screen."""
        original_msg = self.screen_now['msg']
        msg = ''

        # Chop and center msg leaving some padding space:
        msg_lst = original_msg.split('\n')

        for line in msg_lst:
            margin = 2 if (self.screen_now['type'] == 'menu') else 0
            formatted_line  = line[0:(DISPLAY_SIZE[1]-(2*margin))]
            remainder = DISPLAY_SIZE[1]-len(formatted_line)-(2*margin)
            paddingN = remainder/2
            paddingM = (remainder/2)+(remainder%2)
            formatted_line = (paddingN*' ')+formatted_line+(paddingM*' ')

            if self.screen_now['type'] == 'menu':
                if self.screen_now['prev']:
                    formatted_line = '< '+formatted_line
                else:
                    formatted_line = '  '+formatted_line

                if self.screen_now['next']:
                    formatted_line += ' >'
                else:
                    formatted_line += '  '

            if (len(msg_lst) == 1) or (line != msg_lst[-1]):
                formatted_line += '\n'

            msg += formatted_line

        self.output_screen(msg)

    def change_imu(self, button):
        """Deal with user action on the change imu screen

        Attributes:
            button (string): describes button event
        """

    def change_pw(self, button):
        """Deal with user action on the change pw screen

        Attributes:
            button (string): describes button event
        """

    def change_freq(self, button):
        """Deal with user action on the change frequency screen

        Attributes:
            button (string): describes button event
        """

    def change_current(self, button):
        """Deal with user action on the change current screen

        Attributes:
            button (string): describes button event
        """

    def cycling(self, button):
        """Deal with user action on the cycling training screen

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
            new_value = resp.message
            self.screen_now['msg'] = 'Corrente:\n'+new_value+' mA'
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
