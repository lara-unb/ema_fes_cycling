#!/usr/bin/env python

"""

Particularly, this code defines the user interface for the embedded
system. The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of stablishing the serial comm
and treating that raw measurement. For more information, check:
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

import rospy

# Import ROS msgs
from std_msgs.msg import UInt8
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


def kill_node_callback(req):
    """ROS Service handler to shutdown this node.

    Attributes:
        req (Empty): empty input
    """
    # Shutdown this node and rely on roslaunch respawn to restart
    rospy.loginfo('Node shutdown: service request')
    rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
    return {}


class Interface(object):
    """A class used to create the user interface menus.

    Attributes:
        ref_dict (dict): screen structure
        self.screen_now (dict): screen being displayed
        self.screens (dict): dict with all screens
        self.services (dict): dict with all ROS services
    """
    def __init__(self, ref_dict):
        rospy.loginfo('Initializing interface')
        self.screen_now = {}
        self.screens = {}
        self.services = {}
        self.topics = {'pub': {},'sub': {}}

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
            rospy.wait_for_service('control/kill_all')
            self.services['kill_all'] = rospy.ServiceProxy(
                'control/kill_all', Empty)
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
            rospy.wait_for_service('imu/set_imu_number', timeout=1.0)
            self.services['set_imu_number'] = rospy.ServiceProxy(
                'imu/set_imu_number', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('control/set_pulse_width', timeout=1.0)
            self.services['set_pulse_width'] = rospy.ServiceProxy(
                'control/set_pulse_width', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('stimulator/set_frequency', timeout=1.0)
            self.services['set_stim_freq'] = rospy.ServiceProxy(
                'stimulator/set_frequency', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('control/set_init_intensity', timeout=1.0)
            self.services['set_init_intensity'] = rospy.ServiceProxy(
                'control/set_init_intensity', SetUInt16)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('control/change_intensity', timeout=1.0)
            self.services['intensity'] = rospy.ServiceProxy(
                'control/change_intensity', SetBool, persistent=True)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        try:
            rospy.wait_for_service('control/on_off', timeout=1.0)
            self.services['on_off'] = rospy.ServiceProxy(
                'control/on_off', SetBool)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)

        # Set the buttons topic
        self.topics['sub']['buttons'] = rospy.Subscriber('button/action', UInt8, self.button_callback)

        # Get the initial value for imu number, current, pw, freq, etc...
        rospy.loginfo('Loading parameter values')
        self.update_parameters()
        self.display_write(self.format_msg('APERTE UM BOTAO', 0), 1, 0, False)
        rospy.loginfo('Ready!')

    def shutdown(self):
        """Send a turn off command to the controller."""
        self.on_off(False)

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

    def update_parameters(self):
        """Load parameters with their initial values."""
        screen_ref = {
            'change_imu': None,
            'change_pw': None,
            'change_freq': None,
            'change_current': None,
            'cycling': None
        }
        # Check services availability
        if self.services.get('set_imu_number'):
            screen_ref['change_imu'] = str(rospy.get_param('imu/wireless_id/pedal'))
        if self.services.get('set_pulse_width'):
            screen_ref['change_pw'] = str(rospy.get_param('control/pulse_width'))
        if self.services.get('set_stim_freq'):
            screen_ref['change_freq'] = str(rospy.get_param('stimulator/freq'))
        if self.services.get('set_init_intensity'):
            screen_ref['change_current'] = str(rospy.get_param('control/initial_current'))
        if self.services.get('intensity'):
            screen_ref['cycling'] = str(rospy.get_param('control/initial_current'))

        for k, v in screen_ref.items():
            if v:
                param_now = ''.join(x for x in self.screens[k]['msg'] if x.isdigit())  # int in str
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
            self.display_write(linemsg, idx, 0, (not idx))

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
                self.display_write(self.format_msg('Desligando...', 0), 1, 0, True)
                self.kill_all()
                rospy.sleep(3)  # Avoid changing the display
                return
            else:
                self.screen_now = self.screens[self.screen_now['select']]
        elif self.screen_now['type'] == 'menu':
            try:
                if action == 'both':  # Shutdown
                    self.display_write(self.format_msg('Desligando...', 0), 1, 0, True)
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
                    if self.screen_now['label'] == 'cycling':
                        self.on_off(True)
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
            elif self.screen_now['label'] == 'cycling':
                self.cycling(action)
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
        # Shutdown
        elif button == 'both':
            self.display_write(self.format_msg('Desligando...', 0), 1, 0, True)
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
            self.screen_now = self.screens[self.screen_now['parent']]
            self.display_write(self.format_msg(stat, 0), 1, 0, True)
            rospy.sleep(5)  # Wait 2s
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
        # Shutdown
        elif button == 'both':
            self.display_write(self.format_msg('Desligando...', 0), 1, 0, True)
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
        # Check param and go to parent, previous upper level menu
        elif button == 'double_left':
            if param_now:  # Param value exists
                param_new = str(rospy.get_param('control/pulse_width'))
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
            self.screen_now = self.screens[self.screen_now['parent']]
            self.display_write(self.format_msg(stat, 0), 1, 0, True)
            rospy.sleep(5)  # Wait 2s
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
        # Shutdown
        elif button == 'both':
            self.display_write(self.format_msg('Desligando...', 0), 1, 0, True)
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
            self.screen_now = self.screens[self.screen_now['parent']]
            self.display_write(self.format_msg(stat, 0), 1, 0, True)
            rospy.sleep(5)  # Wait 2s
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
        # Shutdown
        elif button == 'both':
            self.display_write(self.format_msg('Desligando...', 0), 1, 0, True)
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
        # Check param and go to parent, previous upper level menu
        elif button == 'double_left':
            if param_now:  # Param value exists
                param_new = str(rospy.get_param('control/initial_current'))
                self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                self.screens['cycling']['msg'] = self.screen_now['msg']
            self.screen_now = self.screens[self.screen_now['parent']]
            self.update_display()
        # Confirm modification
        elif button == 'double_right':
            stat = 'ERRO'
            if param_now:  # Param value exists
                result = self.set_init_intensity(int(param_now))
                if result:
                    self.screen_now['msg'] = msg_now.replace(param_now,result)
                    self.screens['cycling']['msg'] = self.screen_now['msg']
                    stat = 'OK'
            self.screen_now = self.screens[self.screen_now['parent']]
            self.display_write(self.format_msg(stat, 0), 1, 0, True)
            rospy.sleep(5)  # Wait 2s
            self.update_display()

    def cycling(self, button):
        """Deal with user action on the cycling training screen.

        Attributes:
            button (string): describes button event
        """
        # Get present screen msg and isolate its present param
        msg_now = self.screen_now['msg']
        param_now = ''.join(x for x in msg_now if x.isdigit())  # int in str
        # Shutdown
        if button == 'both':
            self.display_write(self.format_msg('Desligando...', 0), 1, 0, True)
            self.on_off(False)
            self.kill_all()
            rospy.sleep(3)  # Avoid changing the display
        # Change the displayed value and request a change in intensity
        else:
            if param_now:  # Param value exists
                request = 0 if button in {'single_left', 'double_left'} else 1  # Drecease or increase
                result = self.change_intensity(request)
                if result:
                    param_new = result
                    self.screen_now['msg'] = msg_now.replace(param_now,param_new)
                    line2_begin = self.screen_now['msg'].index('\n')+1
                    update = self.format_msg(self.screen_now['msg'][line2_begin:], 0)
                    self.display_write(update, 1, 0, False)

    def kill_all(self):
        """Call a ROS Service to shutdown all nodes."""
        try:
            rospy.wait_for_service('control/kill_all')
            self.services['kill_all']()
            return True
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
            # Shutdown itself
            rospy.loginfo('Node shutdown: kill all failed')
            rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
        return False

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
        except (rospy.ServiceException, rospy.ROSException) as e:
            self.services['display'] = rospy.ServiceProxy('display/write',
                Display, persistent=True)
            rospy.logerr(e)
        return

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
            rospy.wait_for_service('control/set_pulse_width', timeout=1.0)
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
            rospy.wait_for_service('control/set_init_intensity', timeout=1.0)
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
            rospy.wait_for_service('control/change_intensity', timeout=1.0)
            resp = self.services['intensity'](req)
            return resp.message  # Return the intensity now as str
        except (rospy.ServiceException, rospy.ROSException) as e:
            self.services['intensity'] = rospy.ServiceProxy('control/change_intensity',
                SetBool, persistent=True)
            rospy.logerr(e)
        return

    def on_off(self, req):
        """Call a ROS Service to turn control on/off.

        Attributes:
            req (bool): 0 to turn off and 1 to turn on
        """
        try:
            rospy.wait_for_service('control/on_off', timeout=1.0)
            resp = self.services['on_off'](req)
            return resp.message  # Return the control state (on/off)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return


def main():
    # Init display node
    rospy.loginfo('Initializing node')
    rospy.init_node('interface')
    
    # Create interface auxiliary class
    rospy.loginfo('Creating auxiliary class')
    aux = Interface(menu_ref)

    # List provided services
    rospy.loginfo('Setting up services')
    services = {}
    services['kill_node'] = rospy.Service('interface/kill_node', Empty, kill_node_callback)
 
    # Call service to turn off control when stopping
    rospy.on_shutdown(aux.shutdown)

    # Keep python from exiting until the node stops
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
