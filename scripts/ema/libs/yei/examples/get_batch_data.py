## Getting the streaming batch data from the YEI 3-Space Sensor devices with
## Python 2.7, PySerial 2.6, and YEI 3-Space Python API

import threespace_api as ts_api
import time

################################################################################
################## First getting data over a wired connection ##################
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
## searches for wired 3-Space Sensor devices excluding Dongle devices.
filter_flag = ts_api.TSS_FIND_ALL_KNOWN^ts_api.TSS_FIND_DNG
device_list = ts_api.getComPorts(filter=filter_flag)


## Only one 3-Space Sensor device is needed so we are just going to take the
## first one from the list.
com_port, friendly_name, device_type = device_list[0]
device = None
if device_type == "USB":
    device = ts_api.TSUSBSensor(com_port=com_port)
elif device_type == "WL":
    device = ts_api.TSWLSensor(com_port=com_port)
elif device_type == "EM":
    device = ts_api.TSEMSensor(com_port=com_port)
elif device_type == "DL":
    device = ts_api.TSDLSensor(com_port=com_port)
elif device_type == "BT":
    device = ts_api.TSBTSensor(com_port=com_port)

## If a connection to the COM port fails, None is returned.
if device is not None:
    print(device)
    ## Set the stream slots for getting the tared orientation of the device as a
    ## quaternion, the raw component data, and the button state
    device.setStreamingSlots(   slot0='getTaredOrientationAsQuaternion',
                                slot1='getAllRawComponentSensorData',
                                slot2='getButtonState')
    
    ## Now we can start getting the streaming batch data from the device.
    print("==================================================")
    print("Getting the streaming batch data.")
    start_time = time.clock()
    while time.clock() - start_time < 1:
        print(device.getStreamingBatch())
        print("=======================================\n")
    ## Now close the port.
    device.close()

################################################################################
################ Second getting data over a wireless connection ################
################################################################################
device_list = ts_api.getComPorts(filter=ts_api.TSS_FIND_DNG)


## Only one 3-Space Sensor Dongle device is needed so we are just going to
## take the first one from the list.
com_port = device_list[0]
dng_device = ts_api.TSDongle(com_port=com_port)

## If a connection to the COM port fails, None is returned.
if dng_device is not None:
    ## Now we need to get our Wireless device from our Dongle device.
    ## Indexing into the TSDongle instance like it was a list will return a
    ## TSWLSensor instance.
    wl_device = dng_device[0]
    
    ## Set the stream slots for getting the tared orientation of the device as a
    ## quaternion, the raw component data, and the button state
    wl_device.setStreamingSlots(slot0='getTaredOrientationAsQuaternion',
                                slot1='getAllRawComponentSensorData',
                                slot2='getButtonState')
    
    ## Now we can start getting the streaming batch data from the device.
    print("==================================================")
    print("Getting the streaming batch data.")
    start_time = time.clock()
    while time.clock() - start_time < 1:
        print(wl_device.getStreamingBatch())
        print("=======================================\n")
    
    ## Now close the port.
    dng_device.close()



