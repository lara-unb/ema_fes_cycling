#!/usr/bin/env python

import time

DISPLAY_SIZE = (2,16)

class Interface(object):
    """A class used to create the user interface menus
    Attributes:
        menu_idx:
        menu_max:
        submenu_idx:
        screen:
    """
    def __init__(self):
        self.menu_idx = 0
        self.menu_max = 0
        self.submenu_idx = 0
        self.screen = "welcome"
        self.menu_welcome()

    def action_event(self, key):

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
        print "++++++++++++++++"
        print msg + "\n"
        print "++++++++++++++++"

    def check_screen(self):
        return self.screen 

    def menu_welcome(self):
        msg = "~ EMA Trike ~"
        self.update_display(msg)
        time.sleep(2)
        self.screen = "init_or_config"
        self.menu_init_or_config()

    def menu_init_or_config(self):
        self.menu_max = 1

        if self.menu_idx == 0:
            if self.submenu_idx == 0:
                msg = "Configurar  >"
                self.update_display(msg)
            elif self.submenu_idx == 1:
                self.submenu_config()
        elif self.menu_idx == 1:
            if self.submenu_idx == 0:
                msg = "<  Iniciar"
                self.update_display(msg)
            elif self.submenu_idx == 1:
                self.submenu_cycling()

    def submenu_cycling(self):
        msg = "Corrente:"
        self.update_display(msg)

    def submenu_config(self):
        msg = "Larg. Pulso > "
        self.update_display(msg)

def main():
    aux = Interface()

    while True:
        try:
            key = input()
            aux.action_event(key)

            if aux.check_screen() == "welcome":
                aux.menu_welcome()

            elif aux.check_screen() == "init_or_config":
                aux.menu_init_or_config()

            elif aux.check_screen() == "cycling":
                aux.menu_cycling()

        except (NameError, SyntaxError, EOFError):
            pass

if __name__ == '__main__':
    main()