## Pairing the YEI 3-Space Sensor Wireless devices with the YEI 3-Space Sensor
## Dongle devices for a wireless connection with Python 2.7, PySerial 2.6, and
## YEI 3-Space Python API

import threespace_api as ts_api

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
## searches for Wireless devices and Dongle devices.
device_list = ts_api.getComPorts(filter=ts_api.TSS_FIND_DNG|ts_api.TSS_FIND_WL)


## Now go through our known list of 3-Space Sensor devices and create the
## appropriate instance by using the devices' type and COM port
dng_device = None
wl_device = None
for device_port in device_list:
    com_port, friendly_name, device_type = device_port
    if device_type == "DNG":
        dng_device = ts_api.TSDongle(com_port=com_port)
    elif device_type == "WL":
        wl_device = ts_api.TSWLSensor(com_port=com_port)

## If a connection to the COM port fails, None is returned.
if dng_device is not None and wl_device is not None:
    ## We must first set the Pan ID and Channel of the Dongle device and
    ## Wireless device to the same value before pairing.
    pan_id = 1
    channel = 26
    dng_device.setWirelessPanID(pan_id)
    dng_device.setWirelessChannel(channel)
    wl_device.setWirelessPanID(pan_id)
    wl_device.setWirelessChannel(channel)
    
    ## Now we can start pairing the Dongle device and Wireless device.
    ## The TSDongle class has a convenience function for pairing with Wireless
    ## devices.
    
    ## This function is setSensorToDongle. It has 2 parameters:
    ## idx - the index into the Dongle device's wireless table
    ## hw_id - the serial number of the Wireless device to be paired
    
    dng_device.setSensorToDongle(idx=0, hw_id=wl_device.serial_number)
    
    ## serial_number is just one of many certain attributes that the 3-Space
    ## Sensor classes have that hold information of the 3-Space Sensor devices
    ## that would be redundant to always request from the device and would cost
    ## a lot of time.
    
    ## Now we can check if the Wireless device was paired by indexing into the
    ## TSDongle instance like it was a list.
    print(dng_device[0])
    
    ## Now commit our wireless settings
    dng_device.commitWirelessSettings()
    wl_device.commitWirelessSettings()
    
    ## Now close the ports.
    dng_device.close()
    wl_device.close()



