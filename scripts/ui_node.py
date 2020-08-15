#!/usr/bin/env python

import time

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
    '''A class used to create the user interface menus
    Attributes:
        menu_idx:
        menu_max:
        submenu_idx:
        screen:
    '''
    def __init__(self, ref_dict):
        self.screen_now = None
        self.screens = {}
        self.build_screen_group(ref_dict, None)
        self.welcome()

    def build_screen_group(self, structure_dict, parent):
        if structure_dict:
            siblings = list(structure_dict)
            print siblings
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

    def interaction(self, key):
        # if self.screen_now['type'] == 'root':

        # elif self.screen_now['type'] == 'menu':

        # else:


        if (key == 1) and (self.submenu_idx == 0):
            if self.menu_idx > 0:
                self.menu_idx -= 1

        elif (key == 2) and (self.submenu_idx == 0):
            if self.menu_idx < self.menu_max:
                self.menu_idx += 1

        elif (key == 11) and (self.submenu_idx > 0):
            self.submenu_idx -= 1

        elif (key == 22) and (self.submenu_idx <= 2):
            self.submenu_idx += 1

    def update_display(self, msg):
        #
        # Code to center and chop if long missing
        #
        print '++++++++++++++++'
        print msg+'\n'
        print '++++++++++++++++'

    def get_screen_type(self, screen):
        return screen['type']

    def std_menu(self):
        msg = self.screen_now['msg']
        if self.screen_now['prev']:
            msg = '< '+msg
        if self.screen_now['next']:
            msg += ' >'
        self.update_display(msg)

    def welcome(self):
        self.screen_now = self.screens['welcome']
        self.update_display('~ '+self.screen_now['msg']+' ~')
        time.sleep(2)
        self.screen_now = self.screens[self.screen_now['select']]
        self.std_menu()

    # def action(self):

    # def change_pw(self):

    # def change_freq(self):

    # def change_current(self):

    # def cycling(self):


def main():
    aux = Interface(menu_ref)
    print aux.screens

    while True:
        try:
            key = input()
            aux.interaction(key)
            screen = aux.screen_now
            screen_type = aux.get_screen_type(screen)

            if screen_type == 'root':
                aux.welcome()
            elif screen_type == 'menu':
                aux.std_menu()
            elif screen_type == 'action':
                aux.action()

        except (NameError, SyntaxError, EOFError):
            pass


if __name__ == '__main__':
    main()