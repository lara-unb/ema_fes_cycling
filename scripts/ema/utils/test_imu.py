#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import signal
import sys

def signal_handler(signal, frame):
        global dng_device, wl_device2, wl_device3

        print('You pressed Ctrl+C!')
        wl_device2.stopStreaming()
        wl_device3.stopStreaming()
        dng_device.close()
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

from ema.libs.yei import threespace_api as ts_api

def run():
    global dng_device, wl_device2, wl_device3

    com_port = '/dev/ttyACM0'
    dng_device = ts_api.TSDongle(com_port=com_port)

    if dng_device is not None:

        wl_device2 = ts_api.TSWLSensor(logical_id=2, dongle=dng_device)
        wl_device3 = ts_api.TSWLSensor(logical_id=3, dongle=dng_device)

        ## Get data from IMU #2 without streaming
        print("==================================================")
        print("Getting the filtered tared quaternion orientation for IMU #2.")
        quat = wl_device2.getTaredOrientationAsQuaternion()
        if quat is not None:
            print(quat)
        print("==================================================")
        print("Getting the raw sensor data for IMU #2.")
        data = wl_device2.getAllRawComponentSensorData()
        if data is not None:
            print("[%f, %f, %f] --Gyro\n[%f, %f, %f] --Accel\n[%f, %f, %f] --Comp"
                                                                            % data)
        print("==================================================")
        print("Getting the LED color of the device for IMU #2.")
        led = wl_device2.getLEDColor()
        if led is not None:
            print(led)
        print("==================================================")

        # Get data from IMU #3 with streaming
        wl_device2.setStreamingTiming(interval = 1000, duration = 0, delay = 0)
        wl_device2.setStreamingSlots(slot1 = 'getUntaredOrientationAsQuaternion')
        wl_device2.startStreaming()

        # Get data from IMU #3 with streaming
        wl_device3.setStreamingTiming(interval = 1000, duration = 0, delay = 0)
        wl_device3.setStreamingSlots(slot1 = 'getUntaredOrientationAsQuaternion',)
        wl_device3.startStreaming()

        while(True):
            ts2, data2 = wl_device2.getLatestStreamData(10.0)
            ts3, data3 = wl_device3.getLatestStreamData(10.0)
            print "[imu #2] ts = {0}, data = {1}".format(ts2,data2)
            print "[imu #3] ts = {0}, data = {1}".format(ts3,data3)
