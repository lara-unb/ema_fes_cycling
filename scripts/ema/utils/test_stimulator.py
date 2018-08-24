#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import sys
import time

def signal_handler(signal, frame):
        global stim_manager
        print 'closing'
        stim_manager.ccl_stop()
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

import ema.modules.stimulator as stimulator

def runExamples():
    # Single pulse example 1
    print 'Single pulse example 1'

    config_dict1 = {
        'port': '/dev/ttyUSB0',
        'channel_stim': [],
        'channel_lf': [],
        'n_factor': 0,
        'ts1': 20,
        'ts2': 1.5
    }

    stim_manager = stimulator.Stimulator(config_dict1)
    stim_manager.single_pulse(channel_number = 3, pulse_width = 200, pulse_current = 120)

    # Single pulse example 2
    print 'Single pulse example 2'

    config_dict2 = {
        'port': '/dev/ttyUSB0',
        'channel_stim': [],
        'channel_lf': [],
        'n_factor': 0,
        'ts1': 20,
        'ts2': 1.5
    }

    stim_manager = stimulator.Stimulator(config_dict2)
    stim_manager.single_pulse(channel_number = 6, pulse_width = 221, pulse_current = 55)

    # Channel list mode initialisation example 1
    print 'Channel list mode initialisation example 1'

    config_dict3 = {
        'port': '/dev/ttyUSB0',
        'channel_stim': [1, 2, 5],
        'channel_lf': [5],
        'n_factor': 1,
        'ts1': 50,
        'ts2': 5
    }

    stim_manager = stimulator.Stimulator(config_dict3)
    stim_manager.ccl_initialize()

    # Channel list mode initialisation example 2
    print 'Channel list mode initialisation example 2'

    config_dict4 = {
        'port': '/dev/ttyUSB0',
        'channel_stim': [2, 3, 6, 8],
        'channel_lf': [2, 3],
        'n_factor': 2,
        'ts1': 16.5,
        'ts2': 6
    }

    stim_manager = stimulator.Stimulator(config_dict4)
    stim_manager.ccl_initialize()

    # Channel list mode update example
    print 'Channel list mode update example'

    config_dict5 = {
        'port': '/dev/ttyUSB0',
        'channel_stim': [2, 3, 6, 8],
        'channel_lf': [2, 3],
        'n_factor': 2,
        'ts1': 16.5,
        'ts2': 6
    }

    stim_manager = stimulator.Stimulator(config_dict5)
    stim_manager.ccl_initialize()
    stim_manager.ccl_update(mode = {2: 'single', 3: 'triplet', 6: 'doublet', 8: 'doublet'},
                            pulse_width = {2: 100, 3: 200, 6: 300, 8: 400},
                            pulse_current = {2: 52, 3: 55, 6: 72, 8: 92})

def run():
    global stim_manager

    # examples from docs, mostly to test if buildPacket follows protocol
    #runExamples()

    # my tests: oscilloscope at 2V/div and 2ms/div, trigger at 6V on CH1
    config_dict = {
        'port': '/dev/ttyUSB0',
        'channel_stim': [1, 2],
        'channel_lf': [],
        'n_factor': 0,
        'ts1': 15,
        'ts2': 5
    }

    stim_manager = stimulator.Stimulator(config_dict)

    # test 1: using every function in ccl mode
    print 'test 1: using every function in ccl mode'
    stim_manager.ccl_initialize()

    key = raw_input("press enter to run, 's' to skip")
    while key != 's' and key != 'q':
        for pc in range(2,8,2):
            for pw in range(100,501):
                print 'CH2:','pc',pc,'pc',pw,'doublet'
                stim_manager.ccl_update(mode = {1: 'single', 2: 'doublet'},
                                        pulse_width = {1: 500, 2: pw},
                                        pulse_current = {1: 6, 2: pc})

            for pw in reversed(range(100,501)):
                print 'CH2:','pc',pc,'pc',pw, 'doublet'
                stim_manager.ccl_update(mode = {1: 'single', 2: 'doublet'},
                                        pulse_width = {1: 500, 2: pw},
                                        pulse_current = {1: 6, 2: pc})

        for pc in reversed(range(2,8,2)):
            for pw in range(100,501):
                print 'CH2:','pc',pc,'pc',pw, 'triplet'
                stim_manager.ccl_update(mode = {1: 'single', 2: 'triplet'},
                                        pulse_width = {1: 500, 2: pw},
                                        pulse_current = {1: 6, 2: pc})

            for pw in reversed(range(100,501)):
                print 'CH2:','pc',pc,'pc',pw, 'triplet'
                stim_manager.ccl_update(mode = {1: 'single', 2: 'triplet'},
                                        pulse_width = {1: 500, 2: pw},
                                        pulse_current = {1: 6, 2: pc})

        key = raw_input("press enter to rerun test 1, 'q' to quit")

    stim_manager.ccl_stop()

    # test 2: using single_pulse to generate arbitrary waveforms
    print 'test 2: using single_pulse to generate arbitrary waveforms'

    key = raw_input("press enter to run, 's' to skip")
    while key != 's' and key != 'q':
        for pw in range(100,501):
            stim_manager.single_pulse(channel_number = 1, pulse_width = pw, pulse_current = 6)
            time.sleep(0.018)

        key = raw_input("press enter to re-run test 2, 'q' to quit")
