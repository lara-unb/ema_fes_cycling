#IMU functions
from ema.libs.yei import threespace_api as ts_api

class IMU:
    def __init__(self, config_dict):
        self.config_dict = config_dict
        self.broadcast = False
        self.streaming = False
        self.devices = {}
        self.dongles = []
        self.imus = []
        self.wired_imus = []
        self.wireless_imus = []
        self.sensor_list = []

        for name in config_dict['dev_names']:
            dev_type = config_dict['dev_type'][name]

            if dev_type == 'DNG':
                wired_port = config_dict['wired_port'][name]
                self.devices[name] = ts_api.TSDongle(com_port=wired_port)
                self.dongles.append(name)

            elif dev_type == 'WL' and config_dict['broadcast'] == False:
                imu_mode = config_dict['imu_mode'][name]
                self.imus.append(name)

                if imu_mode == 'wired':
                    wired_port = config_dict['wired_port'][name]
                    self.devices[name] = ts_api.TSWLSensor(com_port=wired_port)
                    self.wired_imus.append(name)

                if imu_mode == 'wireless':
                    wireless_dng = config_dict['wireless_dng'][name]
                    wireless_id = config_dict['wireless_id'][name]

                    self.devices[name] = ts_api.TSWLSensor(logical_id=wireless_id, dongle=self.devices[wireless_dng])
                    self.wireless_imus.append(name)

        if config_dict['autocalibrate'] == True:
            self.autocalibrate()
            
        if config_dict['streaming'] == True:
            self.streaming = True
            
            self.streaming_interval = config_dict['streaming_interval']
            self.streaming_duration = config_dict['streaming_duration']
            self.streaming_delay = config_dict['streaming_delay']
            
            if config_dict['broadcast'] == False:
                self.streaming_slots = config_dict['streaming_slots']
                
                for name in self.imus:
                    if self.streaming_duration == 'unlimited':
                        self.streaming_duration = 0xFFFFFFFF
                        
                    # Set IMU streams to the appropriate timing
                    self.devices[name].setStreamingTiming(interval=self.streaming_interval,
                                                          duration=self.streaming_duration,
                                                          delay=self.streaming_delay)
                    
                    # Set IMU slots according to config file
                    padded_slots = list(self.streaming_slots[name])
                    for i in range(0,8):
                        try:
                            padded_slots[i]
                        except IndexError:
                            padded_slots.append('null')
                            
                    self.devices[name].setStreamingSlots(slot0=padded_slots[0],
                                                         slot1=padded_slots[1],
                                                         slot2=padded_slots[2],
                                                         slot3=padded_slots[3],
                                                         slot4=padded_slots[4],
                                                         slot5=padded_slots[5],
                                                         slot6=padded_slots[6],
                                                         slot7=padded_slots[7])
                    
                    # Start streaming
                    # G: for some reason, startStreaming was a bad idea. without it, we get to 67Hz
                    #self.devices[name].startStreaming()
            else:
                self.broadcast = True
                
                for i in range(6): # Only checking the first six logical indexes
                    sens = self.devices['pc'][i]
                    if sens is not None:
                        name = str(i)
                        self.sensor_list.append(sens)
                        self.devices[name] = sens
                        self.imus.append(name)
                
                if self.streaming_duration == 'unlimited':
                    self.streaming_duration = 0xFFFFFFFF
                    
                # Set IMU streams to the appropriate timing
                ts_api.global_broadcaster.setStreamingTiming(interval=self.streaming_interval,
                                                             duration=self.streaming_duration,
                                                             delay=self.streaming_delay,
                                                             delay_offset=12000,
                                                             filter=self.sensor_list)
                
                # Set IMU slots to getQuaternion, getGyroData, getAccelData and getButtonState
                ts_api.global_broadcaster.setStreamingSlots(slot0='getTaredOrientationAsQuaternion',
                                                            slot1='getNormalizedGyroRate',
                                                            slot2='getCorrectedAccelerometerVector',
                                                            slot3='getButtonState',
                                                            filter=self.sensor_list)
                
                # Start streaming
                ts_api.global_broadcaster.startStreaming(filter=self.sensor_list)

        self.setRightHandedAxis()

########################################
# Calibration
########################################

    def calibrate(self, name): ## G: beginGyroscopeAutoCalibration, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'calibrate: ', name
            return self.devices[name].beginGyroscopeAutoCalibration()

        else:
            print 'calibrate not defined for dev_type = ', dev_type
            return 0

########################################
# Set euler to YXZ
########################################

    def setEulerToYXZ(self, name): ## G: setEulerAngleDecompositionOrder with angle_order = 1, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'setEulerToYXZ: ', name
            return self.devices[name].setEulerAngleDecompositionOrder(angle_order=0x01)

        else:
            #print 'setEulerToYXZ not defined for dev_type = ', dev_type
            return 0

########################################
# Tare with current orientation
########################################

    def tare(self, name): ## G: tareWithCurrentOrientation, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'tare: ', name
            return self.devices[name].tareWithCurrentOrientation()

        else:
            print 'tare not defined for dev_type = ', dev_type
            return 0

########################################
# Check Buttons
########################################

    def getButtonState(self, name): ## G: getButtonState, works with TSWLSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'checkButtons: ', name
            return self.devices[name].getButtonState()

        else:
            print 'checkButtons not defined for dev_type = ', dev_type
            return 0

########################################
# Get Quaternion
########################################

    def getQuaternion(self, name):
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'getQuaternion: ', name
            return self.devices[name].getTaredOrientationAsQuaternion()

        else:
            print 'getQuaternion not defined for dev_type = ', dev_type
            return 0

########################################
# Get Euler Angles
########################################

    def getEulerAngles(self, name): ## G: getTaredOrientationAsEulerAngles, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'getEulerAngles: ', name
            return self.devices[name].getTaredOrientationAsEulerAngles()

        else:
            print 'getEulerAngles not defined for dev_type = ', dev_type
            return 0

########################################
# Get Gyro Data
########################################

    def getGyroData(self, name): ## G: getNormalizedGyroRate, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'getGyroData: ', name
            return self.devices[name].getNormalizedGyroRate()

        else:
            print 'getGyroData not defined for dev_type = ', dev_type
            return 0

    def getAccelData(self, name): ## G: getCorrectedAccelerometerVector, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'getAccelData: ', name
            return self.devices[name].getCorrectedAccelerometerVector()

        else:
            print 'getAccelData not defined for dev_type = ', dev_type
            return 0

    def getStreamingData(self, name): ## G: getStreamingBatch, need TSSensor (don't do for dongle)
        dev_type = self.config_dict['dev_type'][name]

        if dev_type == 'WL':
            #print 'getStreamingBatch: ', name
            return self.devices[name].getStreamingBatch()

        else:
            print 'getStreamingBatch not defined for dev_type = ', dev_type
            return 0
    
    def shutdown(self):
        if self.streaming == True:
            for name in self.imus:
                print 'shutting down'
            
                # Stop streaming
                self.devices[name].stopStreaming()

    def autocalibrate(self):
        for name in self.imus:
            print "Calibrating", name
            calibrationError = 10
            count = 0
            while calibrationError > 0.1 :
                count = count + 1
                ang = []
                while(len(ang) < 3):
                    self.setEulerToYXZ(name)
                    self.calibrate(name)
                    self.tare(name)
                    ang = self.getEulerAngles(name)
                oldError = calibrationError
                calibrationError = ang[0] + ang[1] + ang[2]

                if(oldError != calibrationError):
                    print "Error:", calibrationError
                else:
                    print "Stopped calibrating",name,"due to error stabilization after",count,"attempts"
                    break
            print "Done"

    def setRightHandedAxis(self):
        # axes definitions
        # 0: X: R, Y: U, Z: F (left-handed system, standard operation)
        # 1: X: R, Y: F, Z: U (right-handed system)
        # 2: X: U, Y: R, Z: F (right-handed system)
        # 3: X: F, Y: R, Z: U (left-handed system)
        # 4: X: U, Y: F, Z: R (left-handed system)
        # 5: X: F, Y: U, Z: R (right-handed system)
        axes = 0
        x_inverted = 0
        y_inverted = 1
        z_inverted = 0

        axis_direction_byte = (x_inverted << 5) |(y_inverted << 4)  | (z_inverted << 3) | axes

        for name in self.imus:
            print "Changing", name, "to right handed axis"
            print "axis_direction_byte:", '{:08b}'.format(axis_direction_byte)
            self.devices[name].setAxisDirections(axis_direction_byte)

    def setLeftHandedAxis(self):
        # axes definitions
        # 0: X: R, Y: U, Z: F (left-handed system, standard operation)
        # 1: X: R, Y: F, Z: U (right-handed system)
        # 2: X: U, Y: R, Z: F (right-handed system)
        # 3: X: F, Y: R, Z: U (left-handed system)
        # 4: X: U, Y: F, Z: R (left-handed system)
        # 5: X: F, Y: U, Z: R (right-handed system)
        axes = 0
        x_inverted = 0
        y_inverted = 0
        z_inverted = 0

        axis_direction_byte = (x_inverted << 5) |(y_inverted << 4)  | (z_inverted << 3) | axes

        for name in self.imus:
            print "Changing", name, "to left handed axis"
            print "axis_direction_byte:", '{:08b}'.format(axis_direction_byte)
            self.devices[name].setAxisDirections(axis_direction_byte)

########################################
# Single Command
########################################

    def singleCommand(self, command): ## G: equivalent to writeRead, but using command bytes directly. delete?
        try:
            if self.serial_port is not None:
                self.serial_port.write(">" + str(self.address) + "," + command + "\n") # e escreve na porta
                dados = readData(self.serial_port)
                dados = dados.split(",")
                if int(dados[0]) == 0:
                    return dados
                else:
                    return "No answer"
            else:
                return 'Port error'

        except ValueError:
            return 'Error'
        return dados

def readData(port): ## G: yei api doesn't do separate reading, no worries. delete.
    dados = ''
    data = ''
    i = 1
    while dados == "":
        port.flush()
        data = port.read(port.inWaiting()) # le da porta bytearray
        dados = data.decode()  # transforma bytearray em string
        i += 1
        if i > 700:
            dados = 'No answer'
            break
    return dados
