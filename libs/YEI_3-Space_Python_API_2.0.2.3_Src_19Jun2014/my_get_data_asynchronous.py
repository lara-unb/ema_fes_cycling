## Reading a YEI 3-Space Sensor device's orientation with streaming using
## Python 2.7, PySerial 2.6, and YEI 3-Space Python API

import threespace_api as ts_api
import time

################################################################################
################# Second using a broadcaster to get streaming ##################
################# data for every 3-Space Sensor device known ###################
################################################################################
print("=============================")
print("Broadcaster calls")
print("=============================")

all_list = []
sensor_list = []

device = ts_api.TSDongle(com_port='/dev/ttyACM0')

if device is not None:
    all_list.append(device)
    
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
                                        slot1='getButtonState',
                                        filter=sensor_list)
ts_api.global_broadcaster.startStreaming(filter=sensor_list)

for sensor in sensor_list:
    print sensor
    print sensor.getStreamingBatch()

time.sleep(5)
ts_api.global_broadcaster.stopStreaming(filter=sensor_list)

## Now close the ports.
for device in all_list:
    device.close()
