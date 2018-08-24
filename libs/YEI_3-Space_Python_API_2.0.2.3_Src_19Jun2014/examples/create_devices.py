## Creating a serial port for the YEI 3-Space Sensor devices with Python 2.7,
## PySerial 2.6, and YEI 3-Space Python API

import threespace_api as ts_api

## If the COM port is already known and the device type is known for the 3-Space
## Sensor device, we can just create the appropriate instance without doing a
## search.
com_port = "COM12"
try:
    device = ts_api.TSUSBSensor(com_port=com_port)
except:
    print("No device on {0}".format(com_port))
## If a connection to the COM port fails, None is returned.
else:
    print(device)

    ## Now close the port.
    if device is not None:
        device.close()

## If the COM port is not known or the device type is not known for the 3-Space
## Sensor device, we must do a search for the devices. We can do this by calling
## the getComPorts function which returns a lists of COM port information.
## (both known 3-Space Sensor devices and unknown devices)
## getComPorts also as a parameter called filter that takes a mask that denotes
## what type of 3-Space Sensor device can be found. If filter is not used or set
## to None all connected 3-Space Sensor devices and unknown devices are found.
## Each COM port information is a list containing
## (COM port name, friendly name, 3-Space Sensor device type)
device_list = ts_api.getComPorts()


## Now go through our list of devices and create the appropriate instance by
## using the devices' type and COM port. Unknown devices could be 3-Space Sensor
## devices that were connected via a serial connection. We can find out if there
## is any 3-Space Sensor devices by using the getSensorInfo function. However,
## we must send commands over the port which could make devices other than
## 3-Space Sensor devices act strangely. So, be sure to know what port your
## 3-Space Sensor device(s) are on when calling this function.
## getSensorInfo returns a list of information that it found from the COM port
## (Friendly name, 3-Space Type, 3-Space ID, 3-Space Firmware Version String,
##  3-Space Hardware Version String, is in bootloader mode)
for device_port in device_list:
    com_port, friendly_name, device_type = device_port
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
    elif device_type == "BTL":
        device = "Device on " + com_port + " is in bootloader mode."
    elif device_type == "???":
        port_info = ts_api.getDeviceInfoFromComPort(com_port, poll_device=True)
        device_type = port_info[1]
        is_bootloader = port_info[5]
        if is_bootloader:
            device = "Device on %s is in bootloader mode." % com_port
        elif device_type == "USB":
            device = ts_api.TSUSBSensor(com_port=com_port)
        elif device_type == "EM":
            device = ts_api.TSEMSensor(com_port=com_port)
        ## None of the COM ports in the list were 3-Space Sensor devices
        else:
            device = "Device on %s is not a 3-Space Sensor device." % com_port
    
    ## If a connection to the COM port fails, None is returned.
    print(device)
    
    ## Now close the port.
    if device is not None and type(device) is not str:
        device.close()




