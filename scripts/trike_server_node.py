#!/usr/bin/env python

"""

Particularly, this code establishes the dynamic reconfiguration server: it
watches and defines rules for dynamic parameter changes.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of stablishing the serial comm
and treating that raw measurement. For more information, check:
http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29

                    _________ NOTES _________

Everytime a parameter changes, the callback is executed. The code identifies
which parameter was changed by the level number. Levels are integers declared
in the .cfg file to identify each parameter and is passed to the callback
to express what was modified. Rules and restrictions are applied and some
parameters are related to others. At last, the name of modified parameters
are gathered to be published on a topic.

Level identifies each parameter and consists of 5 numbers described here:
    - First 2: refer to its group
    - Next 2: refer to the element/parameter itself
    - Last: refer to the stimulator channel affected

Ch12: used with quadriceps (rectus), refers to stimulator channels 1 and 2
Ch34: used with hamstrings, refers to stimulator channels 3 and 4
Ch56: used with gluteal, refers to stimulator channels 5 and 6
Ch78: used with quadriceps (lateral), refers to stimulator channels 7 and 8

"""

# Python 2 and 3 compatibility
# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function
# from builtins import *

import rospy
from ema_fes_cycling.cfg import TrikeServerConfig  # pkgname.cfg, cfgfilenameConfig

# Import ROS msgs:
from std_msgs.msg import String

# Other imports:
import dynamic_reconfigure.server

# Global variables:
global pub
global reverse_ref
global callback_ref

# Set initial param values:
pub = {}
reverse_ref = {}
callback_ref = {}

# Reference dict with all elements and respective levels:
reverse_ref = {
    'Ch12Enable':      99001,
    'Ch34Enable':      99003,
    'Ch56Enable':      99005,
    'Ch78Enable':      99007,

    'Shift':           99010,
    'ramp_start':      99013,
    'ramp_end':        99015,
    'MarkAssistance':  99020,
    'AutoCEnable':     99030,
    'AutoCShift':      99040,
    'AutoCVelocity':   99050,

    'Ch12LinkCurrent': 12100,
    'Ch1Current':      12111,
    'Ch2Current':      12112,
    'Ch1PulseWidth':   12121,
    'Ch2PulseWidth':   12122,

    'Ch12LinkAngle':   12200,
    'Ch1AngleMin':     12211,
    'Ch1AngleMax':     12221,
    'Ch2AngleMin':     12212,
    'Ch2AngleMax':     12222,

    'Ch34LinkCurrent': 34100,
    'Ch3Current':      34113,
    'Ch4Current':      34114,
    'Ch3PulseWidth':   34123,
    'Ch4PulseWidth':   34124,

    'Ch34LinkAngle':   34200,
    'Ch3AngleMin':     34213,
    'Ch3AngleMax':     34223,
    'Ch4AngleMin':     34214,
    'Ch4AngleMax':     34224,

    'Ch56LinkCurrent': 56100,
    'Ch5Current':      56115,
    'Ch6Current':      56116,
    'Ch5PulseWidth':   56125,
    'Ch6PulseWidth':   56126,

    'Ch56LinkAngle':   56200,
    'Ch5AngleMin':     56215,
    'Ch5AngleMax':     56225,
    'Ch6AngleMin':     56216,
    'Ch6AngleMax':     56226,

    'Ch78LinkCurrent': 78100,
    'Ch7Current':      78117,
    'Ch8Current':      78118,
    'Ch7PulseWidth':   78127,
    'Ch8PulseWidth':   78128,

    'Ch78LinkAngle':   78200,
    'Ch7AngleMin':     78217,
    'Ch7AngleMax':     78227,
    'Ch8AngleMin':     78218,
    'Ch8AngleMax':     78228,
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
        # the other channel in its group, e.g. Ch2Current is Ch1Current's
        # dual and Ch7PulseWidth is Ch8PulseWidth's dual. Their level
        # differs only by the last digit, the channel. General parameters
        # dont have dual.
        if (self.group != 99) and (self.channel != 0):
            if (self.channel % 2) == 0:  # Even channel
                self.dual = self.level - 1
            else:  # Odd channel
                self.dual = self.level + 1


def initializeGroups(config):
    """Collapse the groups in the dynamic reconfigure GUI.

    Attributes:
        config (dict): server dictionary with its parameters
    """
    for pair in ['12','34','56','78']: # all channel groups
        config['groups']['groups']['Ch' + pair]['state'] = False
    return config


def enableUpdated(config, item):
    """Deal with Enable parameters to turn on/off stim.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref
    global reverse_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    pair = str(item.channel) + str(item.channel + 1)
    retset = set([item.element])

    if not config[item.element]:  # Group deactivated
        # Hide/collapse group
        config['groups']['groups']['Ch' + pair]['state'] = False

        # Zero the currents for safety
        c1 = Param(callback_ref, reverse_ref['Ch' + str(item.channel) + 'Current'])
        c2 = Param(callback_ref, c1.dual)
        config[c1.element] = config[c2.element] = 0
        retset.update(currentUpdated(config, c1))
        retset.update(currentUpdated(config, c2))
    else:
        # Show/expand group
        config['groups']['groups']['Ch' + pair]['state'] = True

    return retset


def generalUpdated(config, item):
    """Deal with General parameter changes in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    return set([item.element])


def linkCurrentUpdated(config, item):
    """Deal with LinkCurrent parameter changes in the server.

    When a LinkCurrent is activated, both channels in a group will have
    the Current and PulseWidth parameters bound to their duals.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref
    global reverse_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value

    if config[item.element]:  # Link activated
        retset = set([item.element])
        for elem in ('Current', 'PulseWidth'):
            odd_element = 'Ch' + str(item.group)[0] + elem
            even_element = 'Ch' + str(item.group)[1] + elem
            odd_level = reverse_ref[odd_element]
            even_level = reverse_ref[even_element]

            # Reset values to the element min:
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


def linkAngleUpdated(config, item):
    """Deal with LinkAngle parameter changes in the server.

    When a LinkAngle is activated, both channels in a group will have
    the AngleMin and AngleMax parameters bound to their duals.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref
    global reverse_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value

    if config[item.element]:  # Link activated
        retset = set([item.element])
        for elem in ('AngleMin', 'AngleMax'):
            odd_element = 'Ch' + str(item.group)[0] + elem
            even_element = 'Ch' + str(item.group)[1] + elem
            even_level = reverse_ref[even_element]

            # Keep odd angle values and reset even:
            reseter = (config[odd_element] + 180) % 360
            config[even_element] = reseter  # Update dual
            callback_ref[even_level]['prev'] = reseter  # Update previous value
            retset.add(even_element)
        return retset

    return set([item.element])


def currentUpdated(config, item):
    """Deal with Current parameter changes in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref

    prev = callback_ref[item.level]['prev']  # Value before update

    # Prevents the user from abruptly increasing the current:
    if (config[item.element] - prev) > 2:  # Check for how much it changed
        config[item.element] = prev + 2

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value

    # Modifies the current as a pair:
    if config['Ch' + str(item.group) + 'LinkCurrent']:
        item_dual = Param(callback_ref, item.dual)  # Linked channel
        config[item_dual.element] = config[item.element]  # Update dual
        callback_ref[item_dual.level]['prev'] = config[item_dual.element]  # Update previous value
        return set([item.element, item_dual.element])

    return set([item.element])


def pulseWidthUpdated(config, item):
    """Deal with PulseWidth parameter changes in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value

    if config['Ch' + str(item.group) + 'LinkCurrent']:
        item_dual = Param(callback_ref, item.dual)  # Linked channel
        config[item_dual.element] = config[item.element]  # Update dual
        callback_ref[item_dual.level]['prev'] = config[item_dual.element]  # Update previous value
        return set([item.element, item_dual.element])

    return set([item.element])


def angleUpdated(config, item):
    """Deal with Angle parameter changes in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        item (Param): parameter class instance
    """
    global callback_ref

    prev = callback_ref[item.level]['prev']  # Value before update
    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value

    # Modifies the angle as a pair:
    if config['Ch' + str(item.group) + 'LinkAngle']:
        item_dual = Param(callback_ref, item.dual)  # Linked channel
        diff = config[item.element] - prev
        reseter = (config[item_dual.element] + diff) % 360
        config[item_dual.element] = reseter  # Update dual
        callback_ref[item_dual.level]['prev'] = reseter  # Update previous value
        return set([item.element, item_dual.element])

    return set([item.element])


# Dict w/ levels, elements, function handlers and previous value:
callback_ref = {
    99001: {'name': 'Ch12Enable',      'flag': enableUpdated,     'prev': False},
    99003: {'name': 'Ch34Enable',      'flag': enableUpdated,     'prev': False},
    99005: {'name': 'Ch56Enable',      'flag': enableUpdated,     'prev': False},
    99007: {'name': 'Ch78Enable',      'flag': enableUpdated,     'prev': False},

    99010: {'name': 'Shift',           'flag': generalUpdated,    'prev':     0},
    99013: {'name': 'ramp_start',      'flag': generalUpdated,    'prev':  15.0},
    99015: {'name': 'ramp_end',        'flag': generalUpdated,    'prev':  10.0},
    99020: {'name': 'MarkAssistance',  'flag': generalUpdated,    'prev': False},
    99030: {'name': 'AutoCEnable',     'flag': generalUpdated,    'prev': False},
    99040: {'name': 'AutoCShift',      'flag': generalUpdated,    'prev':    10},
    99050: {'name': 'AutoCVelocity',   'flag': generalUpdated,    'prev':    48},

    12100: {'name': 'Ch12LinkCurrent', 'flag': linkCurrentUpdated,'prev':  True},
    12111: {'name': 'Ch1Current',      'flag': currentUpdated,    'prev':     0},
    12112: {'name': 'Ch2Current',      'flag': currentUpdated,    'prev':     0},
    12121: {'name': 'Ch1PulseWidth',   'flag': pulseWidthUpdated, 'prev':   500},
    12122: {'name': 'Ch2PulseWidth',   'flag': pulseWidthUpdated, 'prev':   500},

    12200: {'name': 'Ch12LinkAngle',   'flag': linkAngleUpdated,  'prev':  True},
    12211: {'name': 'Ch1AngleMin',     'flag': angleUpdated,      'prev':   280},
    12221: {'name': 'Ch1AngleMax',     'flag': angleUpdated,      'prev':     0},
    12212: {'name': 'Ch2AngleMin',     'flag': angleUpdated,      'prev':   100},
    12222: {'name': 'Ch2AngleMax',     'flag': angleUpdated,      'prev':   180},

    34100: {'name': 'Ch34LinkCurrent', 'flag': linkCurrentUpdated,'prev':  True},
    34113: {'name': 'Ch3Current',      'flag': currentUpdated,    'prev':     0},
    34114: {'name': 'Ch4Current',      'flag': currentUpdated,    'prev':     0},
    34123: {'name': 'Ch3PulseWidth',   'flag': pulseWidthUpdated, 'prev':   500},
    34124: {'name': 'Ch4PulseWidth',   'flag': pulseWidthUpdated, 'prev':   500},

    34200: {'name': 'Ch34LinkAngle',   'flag': linkAngleUpdated,  'prev':  True},
    34213: {'name': 'Ch3AngleMin',     'flag': angleUpdated,      'prev':    30},
    34223: {'name': 'Ch3AngleMax',     'flag': angleUpdated,      'prev':   105},
    34214: {'name': 'Ch4AngleMin',     'flag': angleUpdated,      'prev':   210},
    34224: {'name': 'Ch4AngleMax',     'flag': angleUpdated,      'prev':   285},

    56100: {'name': 'Ch56LinkCurrent', 'flag': linkCurrentUpdated,'prev':  True},
    56115: {'name': 'Ch5Current',      'flag': currentUpdated,    'prev':     0},
    56116: {'name': 'Ch6Current',      'flag': currentUpdated,    'prev':     0},
    56125: {'name': 'Ch5PulseWidth',   'flag': pulseWidthUpdated, 'prev':   500},
    56126: {'name': 'Ch6PulseWidth',   'flag': pulseWidthUpdated, 'prev':   500},

    56200: {'name': 'Ch56LinkAngle',   'flag': linkAngleUpdated,  'prev':  True},
    56215: {'name': 'Ch5AngleMin',     'flag': angleUpdated,      'prev':    70},
    56225: {'name': 'Ch5AngleMax',     'flag': angleUpdated,      'prev':   170},
    56216: {'name': 'Ch6AngleMin',     'flag': angleUpdated,      'prev':   250},
    56226: {'name': 'Ch6AngleMax',     'flag': angleUpdated,      'prev':   350},

    78100: {'name': 'Ch78LinkCurrent', 'flag': linkCurrentUpdated,'prev':  True},
    78117: {'name': 'Ch7Current',      'flag': currentUpdated,    'prev':     0},
    78118: {'name': 'Ch8Current',      'flag': currentUpdated,    'prev':     0},
    78127: {'name': 'Ch7PulseWidth',   'flag': pulseWidthUpdated, 'prev':   500},
    78128: {'name': 'Ch8PulseWidth',   'flag': pulseWidthUpdated, 'prev':   500},

    78200: {'name': 'Ch78LinkAngle',   'flag': linkAngleUpdated,  'prev':  True},
    78217: {'name': 'Ch7AngleMin',     'flag': angleUpdated,      'prev':   280},
    78227: {'name': 'Ch7AngleMax',     'flag': angleUpdated,      'prev':     0},
    78218: {'name': 'Ch8AngleMin',     'flag': angleUpdated,      'prev':   100},
    78228: {'name': 'Ch8AngleMax',     'flag': angleUpdated,      'prev':   180},
}


def callback(config, level):
    """Deal with any change in the server.

    Attributes:
        config (dict): server dictionary with its parameters
        level (int): numerical identifier of changed parameter
    """
    global pub
    global callback_ref

    changes_set = set()

    # Call the appropriate function based on changed param:
    if level > 0:
        p = Param(callback_ref, level)
        changes_set.update(callback_ref[level]['flag'](config, p))

        # Confirm something changed:
        if changes_set:
            updateMsg = String()
            for s in changes_set:
                updateMsg.data += (s + ' ')
            # Send updates:
            pub['update'].publish(updateMsg)

    return config


def main():
    global pub

    # Init dynamic reconfigure server node:
    rospy.init_node('trike_config')

    # List published topics:
    pub['update'] = rospy.Publisher('server/update', String, queue_size=10)

    srv = dynamic_reconfigure.server.Server(
            TrikeServerConfig, callback)  # cfgfilenameConfig, callbackname

    # Initialize GUI groups deactivated:
    srv.update_configuration(initializeGroups(srv.config))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
