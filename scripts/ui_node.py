#!/usr/bin/env python

"""

Interface draft 

The 'menu_ref' dictionary defines the menu/screen structure and the
corresponding interaction between them. Each screen has its message and
submenus, which also have their messages and submenus. The 'welcome'
screen is the root and the screens that have submenus as None are the
leaves, or action screens, where the user is prompt to make changes.

"""

import os

DISPLAY_SIZE = (2,16)

menu_ref = {
    'welcome': {
        'msg':'~ EMA Trike ~',
        'submenus': {
            'config': {
                'msg':'Config',
                'submenus': {
                    'pw': {
                        'msg':'Larg. Pulso',
                        'submenus': {
                            'change_pw': {
                                'msg':'Larg. Pulso:',
                                'submenus': None
                            }
                        }
                    },
                    'freq': {
                        'msg':'Frequencia',
                        'submenus': {
                            'change_freq': {
                                'msg':'Frequencia:',
                                'submenus': None
                            }
                        }
                    },
                    'current': {
                        'msg':'Corrente',
                        'submenus': {
                            'change_current': {
                                'msg':'Corrente:',
                                'submenus': None
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
                        'submenus': None
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
    """
    def __init__(self, ref_dict):
        self.screens = {}
        self.build_screen_group(ref_dict)
        self.screen_now = self.screens['welcome']
        self.update_display()

    def build_screen_group(self, structure_dict, parent=None):
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
                new_screen['select'] = None
                new_screen['next'] = None
                new_screen['prev'] = None

                # Get prev/next screen on same level (nearest siblings)
                position = siblings.index(label)
                if position != len(siblings)-1:  # Last item
                    new_screen['next'] = siblings[position+1]
                if position != 0:  # First item
                    new_screen['prev'] = siblings[position-1]

                # Get where to go when selected (first child)
                if menu['submenus']:
                    children = list(menu['submenus'].keys())
                    new_screen['select'] = children[0]
                else:
                    new_screen['type'] = 'action'

                # Screen type is root
                if (parent == None) and (len(siblings) == 1):
                    new_screen['type'] = 'root'

                # Add screen to their group
                self.screens[label] = new_screen

                self.build_screen_group(menu['submenus'], label)
        return

    def output_screen(self, msg):
        """Format and send screen content to the display.

        Attributes:
            msg (string): the message to be displayed
        """

        #
        # Add code to center and chop if long, based on display size
        #
        os.system('cls' if os.name == 'nt' else 'clear')
        print '++++++++++++++++'
        print msg+'\n'
        print '++++++++++++++++'
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
                if action == 'single_left':
                    output_screen_name = self.screen_now['prev']
                    self.screen_now = self.screens[output_screen_name]
                elif action == 'single_right':
                    output_screen_name = self.screen_now['next']
                    self.screen_now = self.screens[output_screen_name]
                elif action == 'double_left':
                    output_screen_name = self.screen_now['parent']
                    self.screen_now = self.screens[output_screen_name]
                elif action == 'double_right':
                    output_screen_name = self.screen_now['select']
                    self.screen_now = self.screens[output_screen_name]
                elif action == 'both':
                    output_screen_name = 'welcome'
                    self.screen_now = self.screens[output_screen_name]
            except KeyError:
                pass

        elif self.screen_now['type'] == 'action':
            if (action == 'single_left') or (action == 'single_right'):
                # if self.screen_now['label'] == 'change_pw':
                #     self.change_pw(action)
                # elif self.screen_now['label'] == 'change_freq':
                #     self.change_freq(action)
                # elif self.screen_now['label'] == 'change_current':
                #     self.change_current(action)
                # elif self.screen_now['label'] == 'cycling':
                #     self.cycling(action)
                pass
            elif action == 'double_left':
                output_screen_name = self.screen_now['parent']
                self.screen_now = self.screens[output_screen_name]
            elif action == 'double_right':
                pass
            elif action == 'both':
                output_screen_name = 'welcome'
                self.screen_now = self.screens[output_screen_name]

        self.update_display()
        return

    def update_display(self):
        """Change display to current screen."""
        msg = self.screen_now['msg']

        if self.screen_now['type'] == 'root':
            pass
        elif self.screen_now['type'] == 'menu':
            if self.screen_now['prev']:
                msg = '< '+msg
            if self.screen_now['next']:
                msg += ' >'
        elif self.screen_now['type'] == 'action':
            pass

        self.output_screen(msg)

    # def change_pw(self):

    # def change_freq(self):

    # def change_current(self):

    # def cycling(self):


def main():
    aux = Interface(menu_ref)

    while True:
        try:
            key = input()
            action = aux.translate_input(key)
            aux.output_response(action)

        except (NameError, SyntaxError, EOFError):
            pass


if __name__ == '__main__':
    main()