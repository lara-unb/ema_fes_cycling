## Communicating with a paired YEI 3-Space Sensor Wireless device and YEI
## 3-Space Sensor Dongle device wirelessly with Python 2.7, PySerial 2.6, and
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
#device_list = ts_api.getComPorts(filter=ts_api.TSS_FIND_DNG)


## Only the 3-Space Sensor Dongle device is connected so we are just going to
## take the first one from the list.
com_port = '/dev/ttyACM0'#device_list[0]
dng_device = ts_api.TSDongle(com_port=com_port)

## If a connection to the COM port fails, None is returned.
if dng_device is not None:
    ## Now we need to get our Wireless device from our Dongle device.
    ## Indexing into the TSDongle instance like it was a list will return a
    ## TSWLSensor instance.
    wl_device = ts_api.TSWLSensor(logical_id=3, dongle=dng_device)

    ## Now we can start getting information from the Wireless device.
    ## The class instances have all of the functionality that corresponds to the
    ## 3-Space Sensor device type it is representing.
    print("==================================================")
    print("Getting the filtered tared quaternion orientation.")
    quat = wl_device.getTaredOrientationAsQuaternion()
    if quat is not None:
        print(quat)
    print("==================================================")
    print("Getting the raw sensor data.")
    data = wl_device.getAllRawComponentSensorData()
    if data is not None:
        print("[%f, %f, %f] --Gyro\n[%f, %f, %f] --Accel\n[%f, %f, %f] --Comp"
                                                                        % data)
    print("==================================================")
    print("Getting the LED color of the device.")
    led = wl_device.getLEDColor()
    if led is not None:
        print(led)
    print("==================================================")

    ## Now close the port.
    dng_device.close()
