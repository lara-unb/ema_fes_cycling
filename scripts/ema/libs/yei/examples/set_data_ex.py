## Setting data for the YEI 3-Space Sensor devices with Python 2.7,
## PySerial 2.6, and YEI 3-Space Python API

import threespace_api as ts_api
import time

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
## searches for 3-Space Sensor USB devices.
device_list = ts_api.getComPorts(filter=ts_api.TSS_FIND_USB)


## Only one 3-Space Sensor device is needed so we are just going to take the
## first one from the list.
com_port = device_list[0]
device = ts_api.TSUSBSensor(com_port=com_port)
## If a connection to the COM port fails, None is returned.
if device is not None:
    ## Now we can start getting information from the device.
    ## The class instances have all of the functionality that corresponds to the
    ## 3-Space Sensor device type it is representing.
    print("==================================================")
    print("Setting the tared data of the device to an arbitrary quaternion.")
    if device.tareWithQuaternion([0, 0, 0, 1]):
        quat = device.getTareAsQuaternion()
        if quat is not None:
            print("The arbitrary quaternion read from the device:")
            print(quat)
    print("==================================================")
    print("Setting the LED color of the device to RED.")
    if device.setLEDColor((1, 0, 0)):
        time.sleep(2)
        print("Setting the LED color of the device to default BLUE.")
        device.setLEDColor((0, 0, 1))
    print("==================================================")
    
    ## Now close the port.
    device.close()



