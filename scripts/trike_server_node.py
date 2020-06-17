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

Be careful when changing parameters, their attributes are used by other
pieces of code.

Everytime a parameter changes the callback is executed. The code identifies
which parameter was changed by the level number. Levels are integers declared
in the .cfg file to identify each parameter and is passed to the callback
to express what was modified. At last, rules and restrictions are applied
and some parameters are related to others.

Ch12: used with quadriceps (medial), refers to stimulator channels 1 and 2
Ch34: used with hamstrings, refers to stimulator channels 3 and 4
Ch56: used with gluteal, refers to stimulator channels 5 and 6
Ch78: used with quadriceps (lateral), refers to stimulator channels 7 and 8

"""

import rospy
from ema_fes_cycling.cfg import TrikeServerConfig  # pkgname.cfg, cfgfilenameConfig

# Other imports:
import dynamic_reconfigure.server


# Auxiliary class to categorize parameters:
class Param:

    def __init__(self, ref_dict, lev):
        self.level = lev  # Stores the level
        self.group = str(lev)[0:2]  # First 2 digits refer to group (99,12,34,56,78)
        self.element = ref_dict[lev]['name']  # Gets the element according to dict
        self.channel = str(lev)[4]  # Last digit refers to channel 1-8
        self.dual = None  # Level for the same element of dual channel

    # Get the dual element level:
    def getElementDual(self):
        # For Ch1Current element, returns Ch2Current level;
        # for Ch8PulseWidth, returns Ch7PulseWidth level, so forth ...
        ch = int(str(self.level)[-1])
        if (ch != 0):
            if (ch % 2 == 0):  # Even channel
                self.dual = self.level - 1  # Same group, same element, diff channel
            else:  # Odd channel
                self.dual = self.level + 1  # Same group, same element, diff channel

        return self.dual  # Level of the dual


# Handler for changes in enabled parameters:
def enableCheck(config):
    for pair in ['12', '34', '56', '78']:  # All channel groups
        if not config['Ch' + pair + 'Enable']:  # Group deactivated
            config['groups']['groups']['Ch' + pair]['state'] = False  # Hide/collapse group
            config['Ch' + pair[0] + 'Current'] = 0
            config['Ch' + pair[1] + 'Current'] = 0
        else:
            config['groups']['groups']['Ch' + pair]['state'] = True

    return


# Handler for changes in general parameters:
def generalUpdated(config, item):  # Item from Param class
    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    return


# Handler for changes in current link parameters:
def linkCurrentUpdated(config, item):  # item from Param class
    # Reset group current and pulse width when link is activated:
    if config[item.element]:  # Link activated
        for elem in ('Current', 'PulseWidth'):
            odd_element = 'Ch' + item.group[0] + elem
            even_element = 'Ch' + item.group[1] + elem
            odd_level = reverse_ref[odd_element]
            even_level = reverse_ref[even_element]

            # Reset current values to the min and link:
            reseter = min(config[odd_element],config[even_element])
            config[odd_element] = config[even_element] = reseter
            callback_ref[odd_level]['prev'] = reseter
            callback_ref[even_level]['prev'] = reseter

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    return


# Handler for changes in angle link parameters:
def linkAngleUpdated(config, item):  # Item from Param class
    # Reset group min and max angles when link is activated:
    if config[item.element]:  # Link activated
        for elem in ('AngleMin', 'AngleMax'):
            odd_element = 'Ch' + item.group[0] + elem
            even_element = 'Ch' + item.group[1] + elem
            even_level = reverse_ref[even_element]

            # Reset angle values and link:
            reseter = (config[odd_element] + 180) % 360
            config[even_element] = reseter  # Update dual
            callback_ref[even_level]['prev'] = reseter  # Update previous value

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    return


# Handler for changes in current parameters:
def currentUpdated(config, item):  # Item from Param class
    prev = callback_ref[item.level]['prev']  # Value before update

    # Prevents the user from abruptly increasing the current:
    if (config[item.element] - prev) > 2:  # Check for how much it changed
        config[item.element] = prev + 2

    # Modifies the current as a pair:
    if config['Ch' + item.group + 'LinkCurrent']:
        item_dual = Param(callback_ref, item.getElementDual())  # Linked channel
        config[item_dual.element] = config[item.element]  # Update dual
        callback_ref[item_dual.level]['prev'] = config[item_dual.element]  # Update previous value

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    return


# Handler for changes in pulse width parameters:
def pulseWidthUpdated(config, item):  # Item from Param class
    # Modifies the pulse width as a pair:
    if config['Ch' + item.group + 'LinkCurrent']:
        item_dual = Param(callback_ref, item.getElementDual())  # Linked channel
        config[item_dual.element] = config[item.element]  # Update dual
        callback_ref[item_dual.level]['prev'] = config[item_dual.element]  # Update previous value

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    return


# Handler for changes in angle parameters:
def angleUpdated(config, item):  # Item from Param class
    prev = callback_ref[item.level]['prev']  # Value before update

    # Modifies the angle as a pair:
    if config['Ch' + item.group + 'LinkAngle']:
        item_dual = Param(callback_ref, item.getElementDual())  # Linked channel
        diff = config[item.element] - prev
        reseter = (config[item_dual.element] + diff) % 360
        config[item_dual.element] = reseter  # Update dual
        callback_ref[item_dual.level]['prev'] = reseter  # Update previous value

    callback_ref[item.level]['prev'] = config[item.element]  # Update previous value
    return

# Reference dict with all elements and respective levels:
reverse_ref = {
    'Shift':           99010,
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

# Reference dict with all levels, elements, function handlers and previous value:
callback_ref = {
    99010: {'name': 'Shift',           'flag': generalUpdated,    'prev':     0},
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


# Called when a parameter is changed:
def callback(config, level):
    # Always check if groups are enabled for safety.
    enableCheck(config)

    # Doesn't check for level = 0 and -1 and calls the appropriate function
    # based on changed param.
    if level > 0:
        callback_ref[level]['flag'](config, Param(callback_ref, level))

    return config


def main():
    # Init dynamic reconfigure server node:
    rospy.init_node('reconfig')

    srv = dynamic_reconfigure.server.Server(
            TrikeServerConfig, callback)  # cfgfilenameConfig, callbackname

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
