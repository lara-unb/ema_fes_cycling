#!/usr/bin/env python

"""

Particularly, this code establishes the dynamic reconfiguration server: it
watches and defines rules for dynamic parameter changes.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29

                    _________ NOTES _________

Everytime a parameter changes, the callback is executed. The code identifies
which parameter was changed by the level number. Levels are integers declared
in the .cfg file to identify each parameter and is passed to the callback
to express what was modified. Rules and restrictions are applied and some
parameters are related to others.

Level identifies each parameter and consists of 5 numbers described here:
    - First 2: refer to its group
    - Next 2: refer to the element/parameter itself
    - Last: refer to the stimulator channel affected

"""

# # Python 2 and 3 compatibility
# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function
# from builtins import *

import rospy
from ema_fes_cycling.cfg import TrikeServerConfig  # pkgname.cfg, cfgfilenameConfig

# Import ROS msgs
from std_msgs.msg import String

# Other imports
import dynamic_reconfigure.server

# Global variables
global reverse_ref
global callback_ref

# Set initial param values
reverse_ref = {}
callback_ref = {}

# Reference dict with all elements and respective levels
reverse_ref = {
    'ch12_enable':       99001,
    'ch34_enable':       99003,
    'ch56_enable':       99005,
    'ch78_enable':       99007,

    'shift':             99010,
    'ramp_start':        99013,
    'ramp_end':          99015,
    'mark_assistance':   99020,
    'autoC_enable':      99030,
    'autoC_shift':       99040,
    'autoC_velocity':    99050,
    'all_link_current':  99060,

    'ch12_link_current': 12100,
    'ch1_current':       12111,
    'ch2_current':       12112,
    'ch1_pulse_width':   12121,
    'ch2_pulse_width':   12122,

    'ch12_link_angle':   12200,
    'ch1_angle_min':     12211,
    'ch1_angle_max':     12221,
    'ch2_angle_min':     12212,
    'ch2_angle_max':     12222,

    'ch34_link_current': 34100,
    'ch3_current':       34113,
    'ch4_current':       34114,
    'ch3_pulse_width':   34123,
    'ch4_pulse_width':   34124,

    'ch34_link_angle':   34200,
    'ch3_angle_min':     34213,
    'ch3_angle_max':     34223,
    'ch4_angle_min':     34214,
    'ch4_angle_max':     34224,

    'ch56_link_current': 56100,
    'ch5_current':       56115,
    'ch6_current':       56116,
    'ch5_pulse_width':   56125,
    'ch6_pulse_width':   56126,

    'ch56_link_angle':   56200,
    'ch5_angle_min':     56215,
    'ch5_angle_max':     56225,
    'ch6_angle_min':     56216,
    'ch6_angle_max':     56226,

    'ch78_link_current': 78100,
    'ch7_current':       78117,
    'ch8_current':       78118,
    'ch7_pulse_width':   78127,
    'ch8_pulse_width':   78128,

    'ch78_link_angle':   78200,
    'ch7_angle_min':     78217,
    'ch7_angle_max':     78227,
    'ch8_angle_min':     78218,
    'ch8_angle_max':     78228,
}


class Param(object):
    """A class used to categorize and represent a server parameter.

    Attributes:
        level (int): numerical identifier
        group (int): group number
        element (str): param name
        channel (int): stim channel affected
        dual (int): dual param level when applicable
    """
    def __init__(self, ref_dict, lev):
        """
        Args:
            ref_dict (dict): a reference dictionary with levels and
                respective parameters
            lev (int): new Param numerical identifier
        """
        self.level = lev
        self.group = int(str(lev)[0:2])
        self.element = ref_dict[lev]['name']
        self.channel = int(str(lev)[4])
        self.dual = None

        # Some parameters can have a dual, the equivalent parameter for
        # the other channel in its group, e.g. ch2_current is ch1_current's
        # dual and ch7_pulse_width is ch8_pulse_width's dual. Their level
        # differs only by the last digit, the channel. General parameters
        # dont have dual.
        if (self.group != 99) and (self.channel != 0):
            if (self.channel % 2) == 0:  # Even channel
                self.dual = self.level-1
            else:  # Odd channel
                self.dual = self.level+1


def initialize_groups(config):
    """Collapse the groups in the dynamic reconfigure GUI.

    Attributes:
        config (dict): server dictionary with its parameters
    """
    for pair in ['12','34','56','78']: # all channel groups
        config['groups']['groups']['ch'+pair]['state'] = False
    return config

def enable_updated(config, item):
    """Deal with Enable parameters to turn on/off stim.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref
    global reverse_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    pair = str(item.channel)+str(item.channel+1)
    retset = set([item.element])

    if not config[item.element]:  # Group deactivated
        # Hide/collapse group
        config['groups']['groups']['ch'+pair]['state'] = False
        # Zero the currents for safety
        c1 = Param(callback_ref, reverse_ref['ch'+str(item.channel)+'_current'])
        c2 = Param(callback_ref, c1.dual)
        config[c1.element] = config[c2.element] = 0
        retset.update(current_updated(config, c1))
        retset.update(current_updated(config, c2))
    else:
        # Show/expand group
        config['groups']['groups']['ch'+pair]['state'] = True
    return retset

def general_updated(config, item):
    """Deal with General parameter changes in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    return set([item.element])

def link_current_updated(config, item):
    """Deal with link_current parameter changes in the server.

    When a link_current is activated, both channels in a group will have
    the current and pulse width parameters bound to their duals.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref
    global reverse_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    if config[item.element]:  # Link activated
        retset = set([item.element])
        for elem in ('current', 'pulse_width'):
            odd_element = 'ch'+str(item.group)[0]+'_'+elem
            even_element = 'ch'+str(item.group)[1]+'_'+elem
            odd_level = reverse_ref[odd_element]
            even_level = reverse_ref[even_element]
            # Reset values to the element min
            reseter = min(config[odd_element], config[even_element])
            if config[odd_element] < config[even_element]:
                config[even_element] = reseter
                callback_ref[even_level]['prev'] = reseter  # Update previous value
                retset.add(even_element)
            elif config[odd_element] > config[even_element]: 
                config[odd_element] = reseter
                callback_ref[odd_level]['prev'] = reseter  # Update previous value
                retset.add(odd_element)
        return retset
    return set([item.element])

def link_angle_updated(config, item):
    """Deal with link angle parameter changes in the server.

    When a link angle is activated, both channels in a group will have
    the angle min and angle max parameters bound to their duals.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref
    global reverse_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    if config[item.element]:  # Link activated
        retset = set([item.element])
        for elem in ('angle_min', 'angle_max'):
            odd_element = 'ch'+str(item.group)[0]+'_'+elem
            even_element = 'ch'+str(item.group)[1]+'_'+elem
            even_level = reverse_ref[even_element]
            # Keep odd angle values and reset even
            reseter = (config[odd_element]+180)%360
            config[even_element] = reseter  # Update dual
            callback_ref[even_level]['prev'] = reseter  # Update previous value
            retset.add(even_element)
        return retset
    return set([item.element])

def current_updated(config, item):
    """Deal with current parameter changes in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref

    prev = callback_ref[item.level]['prev']  # Value before update
    # Prevents the user from abruptly increasing the current
    if (config[item.element]-prev) > 2:  # Check for how much it changed
        config[item.element] = prev+2
    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    # Modifies the current for all active channels
    if config['all_link_current']:
        retset = set([item.element])
        # Get enabled channels
        active = [i for i in config if 'enable' in i]
        active = [v[2:4] for v in active if config[v]]
        active = ''.join(active)
        # Find the current minimum
        value = min([config['ch'+t+'_current'] for t in active])
        # Prevents the user from abruptly increasing the current
        if (config[item.element]-value) > 2:  # Check for how much it changed
            config[item.element] = value+2
        callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
        active.replace(str(item.channel),'')
        # Change the current for all other active channels
        for channel in active:
            c1 = Param(callback_ref, reverse_ref['ch'+channel+'_current'])
            config[c1.element] = config[item.element]
            callback_ref[c1.level]['prev'] = config[c1.element]  # Update previous value
            retset.update([c1.element])
        return retset
    else:
        # Modifies the current as a pair
        if config['ch'+str(item.group)+'_link_current']:
            item_dual = Param(callback_ref, item.dual)  # Linked channel
            config[item_dual.element] = config[item.element]  # Update dual
            callback_ref[item_dual.level]['prev'] = config[item_dual.element]  # Update previous value
            return set([item.element, item_dual.element])
    return set([item.element])

def pulse_width_updated(config, item):
    """Deal with pulse width parameter changes in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    if config['ch'+str(item.group)+'_link_current']:
        item_dual = Param(callback_ref, item.dual)  # Linked channel
        config[item_dual.element] = config[item.element]  # Update dual
        callback_ref[item_dual.level]['prev'] = config[item_dual.element]  # Update previous value
        return set([item.element, item_dual.element])
    return set([item.element])

def angle_updated(config, item):
    """Deal with angle parameter changes in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref

    prev = callback_ref[item.level]['prev']  # Value before update
    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    # Modifies the angle as a pair
    if config['ch'+str(item.group)+'_link_angle']:
        item_dual = Param(callback_ref, item.dual)  # Linked channel
        diff = config[item.element]-prev
        reseter = (config[item_dual.element]+diff)%360
        config[item_dual.element] = reseter  # Update dual
        callback_ref[item_dual.level]['prev'] = reseter  # Update previous value
        return set([item.element, item_dual.element])
    return set([item.element])


# Dict with levels, elements, function handlers and previous value
callback_ref = {
    99001: {'name': 'ch12_enable',       'flag': enable_updated,       'prev': False},
    99003: {'name': 'ch34_enable',       'flag': enable_updated,       'prev': False},
    99005: {'name': 'ch56_enable',       'flag': enable_updated,       'prev': False},
    99007: {'name': 'ch78_enable',       'flag': enable_updated,       'prev': False},

    99010: {'name': 'shift',             'flag': general_updated,      'prev':    45},
    99013: {'name': 'ramp_start',        'flag': general_updated,      'prev':    25},
    99015: {'name': 'ramp_end',          'flag': general_updated,      'prev':    20},
    99020: {'name': 'mark_assistance',   'flag': general_updated,      'prev': False},
    99030: {'name': 'autoC_enable',      'flag': general_updated,      'prev': False},
    99040: {'name': 'autoC_shift',       'flag': general_updated,      'prev':    10},
    99050: {'name': 'autoC_velocity',    'flag': general_updated,      'prev':    48},
    99060: {'name': 'all_link_current',  'flag': general_updated,      'prev': False},

    12100: {'name': 'ch12_link_current', 'flag': link_current_updated, 'prev':  True},
    12111: {'name': 'ch1_current',       'flag': current_updated,      'prev':     0},
    12112: {'name': 'ch2_current',       'flag': current_updated,      'prev':     0},
    12121: {'name': 'ch1_pulse_width',   'flag': pulse_width_updated,  'prev':   500},
    12122: {'name': 'ch2_pulse_width',   'flag': pulse_width_updated,  'prev':   500},

    12200: {'name': 'ch12_link_angle',   'flag': link_angle_updated,   'prev':  True},
    12211: {'name': 'ch1_angle_min',     'flag': angle_updated,        'prev':   280},
    12221: {'name': 'ch1_angle_max',     'flag': angle_updated,        'prev':    15},
    12212: {'name': 'ch2_angle_min',     'flag': angle_updated,        'prev':   280},
    12222: {'name': 'ch2_angle_max',     'flag': angle_updated,        'prev':    15},

    34100: {'name': 'ch34_link_current', 'flag': link_current_updated, 'prev':  True},
    34113: {'name': 'ch3_current',       'flag': current_updated,      'prev':     0},
    34114: {'name': 'ch4_current',       'flag': current_updated,      'prev':     0},
    34123: {'name': 'ch3_pulse_width',   'flag': pulse_width_updated,  'prev':   500},
    34124: {'name': 'ch4_pulse_width',   'flag': pulse_width_updated,  'prev':   500},

    34200: {'name': 'ch34_link_angle',   'flag': link_angle_updated,   'prev':  True},
    34213: {'name': 'ch3_angle_min',     'flag': angle_updated,        'prev':   280},
    34223: {'name': 'ch3_angle_max',     'flag': angle_updated,        'prev':    15},
    34214: {'name': 'ch4_angle_min',     'flag': angle_updated,        'prev':   280},
    34224: {'name': 'ch4_angle_max',     'flag': angle_updated,        'prev':    15},

    56100: {'name': 'ch56_link_current', 'flag': link_current_updated, 'prev':  True},
    56115: {'name': 'ch5_current',       'flag': current_updated,      'prev':     0},
    56116: {'name': 'ch6_current',       'flag': current_updated,      'prev':     0},
    56125: {'name': 'ch5_pulse_width',   'flag': pulse_width_updated,  'prev':   500},
    56126: {'name': 'ch6_pulse_width',   'flag': pulse_width_updated,  'prev':   500},

    56200: {'name': 'ch56_link_angle',   'flag': link_angle_updated,   'prev':  True},
    56215: {'name': 'ch5_angle_min',     'flag': angle_updated,        'prev':   100},
    56225: {'name': 'ch5_angle_max',     'flag': angle_updated,        'prev':   195},
    56216: {'name': 'ch6_angle_min',     'flag': angle_updated,        'prev':   100},
    56226: {'name': 'ch6_angle_max',     'flag': angle_updated,        'prev':   195},

    78100: {'name': 'ch78_link_current', 'flag': link_current_updated, 'prev':  True},
    78117: {'name': 'ch7_current',       'flag': current_updated,      'prev':     0},
    78118: {'name': 'ch8_current',       'flag': current_updated,      'prev':     0},
    78127: {'name': 'ch7_pulse_width',   'flag': pulse_width_updated,  'prev':   500},
    78128: {'name': 'ch8_pulse_width',   'flag': pulse_width_updated,  'prev':   500},

    78200: {'name': 'ch78_link_angle',   'flag': link_angle_updated,   'prev':  True},
    78217: {'name': 'ch7_angle_min',     'flag': angle_updated,        'prev':   100},
    78227: {'name': 'ch7_angle_max',     'flag': angle_updated,        'prev':   195},
    78218: {'name': 'ch8_angle_min',     'flag': angle_updated,        'prev':   100},
    78228: {'name': 'ch8_angle_max',     'flag': angle_updated,        'prev':   195},
}


def callback(config, level):
    """Deal with any change in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        level (int): numerical identifier of changed parameter
    """
    global callback_ref

    changes_set = set()
    # Call the appropriate function based on changed param
    if level > 0:
        p = Param(callback_ref, level)
        changes_set.update(callback_ref[level]['flag'](config, p))

    return config

def main():
    # Init dynamic reconfigure server node
    rospy.init_node('trike_config')

    srv = dynamic_reconfigure.server.Server(
            TrikeServerConfig, callback)  # cfgfilenameConfig, callbackname

    # Initialize GUI groups deactivated
    srv.update_configuration(initialize_groups(srv.config))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
