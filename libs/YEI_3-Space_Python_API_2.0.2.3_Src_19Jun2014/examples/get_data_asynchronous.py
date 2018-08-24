## Reading a YEI 3-Space Sensor device's orientation with streaming using
## Python 2.7, PySerial 2.6, and YEI 3-Space Python API

import threespace_api as ts_api
import time

################################################################################
############### First streaming data over a wireless connection ################
################################################################################

## If the COM port is not known or the device type is not known for the 3-Space
## Sensor device, we must do a search for the devices. We can do this by calling
## the getComPorts function which returns a lists of COM port information.
## (both known 3-Space Sensor devices and unknown devices)
## getComPorts also as a parameter called filter that takes a mask that denotes
## what type of 3-Space Sensor device can be found. If filter is not used or set
## to None all connected 3-Space Sensor devices and unknown devices are found.
## Each COM port information is a list containing
## (COM port name, friendly name, 3-Space Sensor device type)
## This example makes use of the filter parameter of getComPorts and just
## searches for Dongle devices.
device_list = ts_api.getComPorts(filter=ts_api.TSS_FIND_DNG)


## Only one 3-Space Sensor device is needed so we are just going to take the
## first one from the list.
com_port = device_list[0]
dng_device = ts_api.TSDongle(com_port=com_port)

## If a connection to the COM port fails, None is returned.
if dng_device is not None:
    ## Now this assumes that the Wireless device and Dongle device have already
    ## been paired previously.
    wl_device = dng_device[0]
    
    if wl_device is not None:
        ## Setting up the streaming session for getting the tared orientation
        ## of the Wireless device as a quaternion, the battery percent, and the
        ## button state
        ## setStreamingTiming(interval, duration, delay) in microseconds
        wl_device.setStreamingTiming(interval=0, duration=100000000, delay=0)
        wl_device.setStreamingSlots(slot0='getTaredOrientationAsQuaternion',
                                    slot1='getBatteryPercentRemaining',
                                    slot2='getButtonState')
        wl_device.startStreaming()
        ## We can also record the data
        wl_device.startRecordingData()
        start_time = time.clock()
        while time.clock() - start_time < 5:
            print(wl_device.stream_last_data)
            print("=======================================\n")
        
        wl_device.stopRecordingData()
        wl_device.stopStreaming()
        print("stream_data length = {0}".format(len(wl_device.stream_data)))
    
    ## Now close the port.
    dng_device.close()

################################################################################
################# Second using a broadcaster to get streaming ##################
################# data for every 3-Space Sensor device known ###################
################################################################################
print("=============================")
print("Broadcaster calls")
print("=============================")
device_list = ts_api.getComPorts()

all_list = []
sensor_list = []
for device_port in device_list:
    com_port, friendly_name, device_type = device_port
    device = None
    if device_type == "USB":
        device = ts_api.TSUSBSensor(com_port=com_port)
    elif device_type == "DNG":
        device = ts_api.TSDongle(com_port=com_port)
    elif device_type == "WL":
        device = ts_api.TSWLSensor(com_port=com_port)
    elif device_type == "EM":
        device = ts_api.TSEMSensor(com_port=com_port)
    elif device_type == "DL":
        device = ts_api.TSDLSensor(com_port=com_port)
    elif device_type == "BT":
        device = ts_api.TSBTSensor(com_port=com_port)
    
    if device is not None:
        all_list.append(device)
        if device_type != "DNG":
            sensor_list.append(device)
        else:
            for i in range(6): # Only checking the first six logical indexes
                sens = device[i]
                if sens is not None:
                    sensor_list.append(sens)

## The YEI 3-Space Python API has a global broadcaster called global_broadcaster
## which is an instance of Broadcaster
ts_api.global_broadcaster.setStreamingTiming(   interval=0,
                                                duration=110000000,
                                                delay=1000000,
                                                delay_offset=12000,
                                                filter=sensor_list)
ts_api.global_broadcaster.setStreamingSlots(
                                        slot0='getTaredOrientationAsQuaternion',
                                        slot1='getAllRawComponentSensorData',
                                        slot2='getButtonState',
                                        filter=sensor_list)
ts_api.global_broadcaster.startStreaming(filter=sensor_list)
ts_api.global_broadcaster.startRecordingData(filter=sensor_list)
time.sleep(5)
ts_api.global_broadcaster.stopRecordingData(filter=sensor_list)
ts_api.global_broadcaster.stopStreaming(filter=sensor_list)
for sensor in sensor_list:
    print('Sensor({0}) stream_data len={1}'.format( sensor.serial_number_hex,
                                                    len(sensor.stream_data)))

## Now close the ports.
for device in all_list:
    device.close()









