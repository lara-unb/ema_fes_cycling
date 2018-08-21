#!/usr/bin/env python

import rospy
import ema.modules.stimulator as stimulator

# import ros msgs
from ema_common_msgs.msg import Stimulator
from std_msgs.msg import String

def callback(data, topic):
    global stim_manager

    if topic == 'ccl_update':
        for c, m, pw, pc in zip(data.channel, data.mode, data.pulse_width, data.pulse_current):

            if c not in stim_manager.channel_stim:
                rospy.logwarn('channel %d not in channel_stim!', c)
            else:
                rospy.logdebug('ccl_update in channel %d', c)
                rospy.logdebug('mode %d', m)
                rospy.logdebug('pulse_width %d', pw)
                rospy.logdebug('pulse_current %d', pc)

                stim_manager.ccl_mode[c] = m
                stim_manager.ccl_pulse_width[c] = pw
                stim_manager.ccl_pulse_current[c] = pc

        stim_manager.ccl_update(mode = stim_manager.ccl_mode,
                                pulse_width = stim_manager.ccl_pulse_width,
                                pulse_current = stim_manager.ccl_pulse_current)
    else:
        for c, m, pw, pc in zip(data.channel, data.mode, data.pulse_width, data.pulse_current):
            rospy.logdebug('single_pulse in channel %d', c)
            rospy.logdebug('pulse_width %d', pw)
            rospy.logdebug('pulse_current %d', pc)
            stim_manager.single_pulse(channel_number = c,
                                      pulse_width = pw,
                                      pulse_current = pc)

def main():
    # define stim_manager as global so it can be accessed in callback
    global stim_manager

    # init stimulator node
    rospy.init_node('stimulator', anonymous=False)

    # get stimulator config
    stim_manager = stimulator.Stimulator(rospy.get_param('/ema_trike/stimulator'))

    # list subscribed topics
    sub_ccl = rospy.Subscriber('stimulator/ccl_update', Stimulator, callback = callback, callback_args = 'ccl_update')
    sub_single_pulse = rospy.Subscriber('stimulator/single_pulse', Stimulator, callback = callback, callback_args = 'single_pulse')

    # initialize stimulator ccl mode
    stim_manager.ccl_initialize()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # stop stimulator ccl mode
    stim_manager.ccl_stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
