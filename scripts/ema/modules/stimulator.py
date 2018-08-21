#!/usr/bin/env python
import binascii
import serial
import struct

command_dict = {
    'channelListModeInitialization': {
        'id': 0,
        'field_names': ['Ident', 'Check', 'N_Factor', 'Channel_Stim', 'Channel_Lf', 'X', 'Group_Time', 'Main_Time'],
        'field_size': {'Ident':2, 'Check':3, 'N_Factor':3, 'Channel_Stim':8, 'Channel_Lf':8, 'X': 2, 'Group_Time':5, 'Main_Time':11}
    },
    'channelListModeUpdate': {
        'id': 1,
        'field_names': ['Ident', 'Check',
                        'Mode1', 'X1', 'Pulse_Width1', 'Pulse_Current1',
                        'Mode2', 'X2', 'Pulse_Width2', 'Pulse_Current2',
                        'Mode3', 'X3', 'Pulse_Width3', 'Pulse_Current3',
                        'Mode4', 'X4', 'Pulse_Width4', 'Pulse_Current4',
                        'Mode5', 'X5', 'Pulse_Width5', 'Pulse_Current5',
                        'Mode6', 'X6', 'Pulse_Width6', 'Pulse_Current6',
                        'Mode7', 'X7', 'Pulse_Width7', 'Pulse_Current7',
                        'Mode8', 'X8', 'Pulse_Width8', 'Pulse_Current8'],
        'field_size': {'Ident':2, 'Check':5,
                       'Mode1':2, 'X1':3, 'Pulse_Width1':9, 'Pulse_Current1':7,
                       'Mode2':2, 'X2':3, 'Pulse_Width2':9, 'Pulse_Current2':7,
                       'Mode3':2, 'X3':3, 'Pulse_Width3':9, 'Pulse_Current3':7,
                       'Mode4':2, 'X4':3, 'Pulse_Width4':9, 'Pulse_Current4':7,
                       'Mode5':2, 'X5':3, 'Pulse_Width5':9, 'Pulse_Current5':7,
                       'Mode6':2, 'X6':3, 'Pulse_Width6':9, 'Pulse_Current6':7,
                       'Mode7':2, 'X7':3, 'Pulse_Width7':9, 'Pulse_Current7':7,
                       'Mode8':2, 'X8':3, 'Pulse_Width8':9, 'Pulse_Current8':7}
    },
    'channelListModeStop': {
        'id': 2,
        'field_names': ['Ident', 'Check'],
        'field_size': {'Ident':2, 'Check':5}
    },
    'singlePulseGeneration': {
        'id': 3,
        'field_names': ['Ident', 'Check', 'Channel_Number', 'X', 'Pulse_Width', 'Pulse_Current'],
        'field_size': {'Ident':2, 'Check':5, 'Channel_Number':3, 'X': 2, 'Pulse_Width':9,  'Pulse_Current':7}
    }
}

reverse_command_dict = {
    0: 'channelListModeInitialization',
    1: 'channelListModeUpdate',
    2: 'channelListModeStop',
    3: 'singlePulseGeneration'
}

mode_dict = {
    'single': 0,
    'doublet': 1,
    'triplet': 2
}

class Stimulator:
    def __init__(self, config_dict):
        self.config_dict = config_dict

        try:
            self.channel_stim = config_dict['channel_stim']
        except KeyError:
            self.channel_stim = []
            print 'channel_stim not found in config_dict, defaulting to []'

        try:
            self.channel_lf = config_dict['channel_lf']
        except KeyError:
            self.channel_lf = []
            print 'channel_lf not found in config_dict, defaulting to []'

        try:
            self.n_factor = config_dict['n_factor']
        except KeyError:
            self.n_factor = 0
            print 'n_factor not found in config_dict, defaulting to 0'

        try:
            self.freq = config_dict['freq']
            self.ts1 = round((1/float(self.freq))*1000)
        except KeyError:
            print 'freq not found in config_dict, using ts1 for Main_Time'
            try:
                self.ts1 = config_dict['ts1']
            except KeyError:
                print 'ts1 not found in config_dict, defaulting to 20ms (50 Hz)'
                self.ts1 = 20

        try:
            self.ts2 = config_dict['ts2']
        except KeyError:
            print 'ts2 not found in config_dict, defaulting to 1.5'
            self.ts2 = 1.5

        try:
            self.port = config_dict['port']
        except KeyError:
            print 'port not found in config_dict, defaulting to /dev/ttyUSB0'
            self.port = '/dev/ttyUSB0'

        # Build ccl_settings and set them to default values
        self.ccl_mode = {}
        self.ccl_pulse_width = {}
        self.ccl_pulse_current = {}

        for c in self.channel_stim:
            self.ccl_mode[c] = 'single'
            self.ccl_pulse_width[c] = 0
            self.ccl_pulse_current[c] = 0

        # Connect to stimulator
        self.serial_port = serial.Serial(port=self.port,
                                         baudrate=115200,
                                         bytesize=serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_TWO,
                                         rtscts=True)

    def ccl_initialize(self):
        cmd = 'channelListModeInitialization'
        ident = command_dict[cmd]['id']

        nf = self.n_factor                                       # grab from config: n_factor
        cs = self._bitset(0,[x - 1 for x in self.channel_stim])  # grab from config: channel_stim
        clf = self._bitset(0,[x - 1 for x in self.channel_lf])   # grab from config: channel_lf
        gt = int(round((self.ts2 - 1.5) / 0.5))                  # grab from config: ts2
        mt = int(round((self.ts1 - 1.0) / 0.5))                  # grab from config: ts1

        # print 'gt',gt
        # print 'mt',mt
        # print 'ts1',self.ts1
        # print 'ts2',self.ts2

        check = (nf + cs + clf + gt + mt) % 8

        fields = {
            'Ident': ident,
            'Check': check,
            'N_Factor': nf,
            'Channel_Stim': cs,
            'Channel_Lf': clf,
            'Group_Time': gt,
            'Main_Time': mt,
            'X': 0 #filler
        }

        pkt = self._buildPacket(cmd,fields)

        return self._writeRead(pkt)

    def ccl_update(self, mode, pulse_width, pulse_current):
        cmd = 'channelListModeUpdate'
        ident = command_dict[cmd]['id']

        check = 0

        for c in self.channel_stim: # grab from config: channel_stim
            check += mode_dict[mode[c]]
            check += pulse_width[c]
            check += pulse_current[c]

        check = check % 32

        fields = {
            'Ident': ident,
            'Check': check
        }

        for c in self.channel_stim:
            fields['Mode' + str(c)] = mode_dict[mode[c]]
            fields['Pulse_Width' + str(c)] = pulse_width[c]
            fields['Pulse_Current' + str(c)] = pulse_current[c]
            fields['X' + str(c)] = 0 # filler

        pkt = self._buildPacket(cmd,fields)

        return self._writeRead(pkt)

    def ccl_stop(self):
        cmd = 'channelListModeStop'
        ident = command_dict[cmd]['id']
        check = 0

        fields = {
            'Ident': ident,
            'Check': check
        }

        pkt = self._buildPacket(cmd,fields)

        return self._writeRead(pkt)

    def single_pulse(self, channel_number, pulse_width, pulse_current):
        cmd = 'singlePulseGeneration'
        ident = command_dict[cmd]['id']

        cn = channel_number-1# value in range: 1-8 coming from user
        pw = pulse_width# value in range: 0,10..500 us coming from user
        pc = pulse_current# value in range: 0..127 mA coming from user

        check = (cn + pw + pc) % 32

        fields = {
            'Ident': ident,
            'Check': check,
            'Channel_Number': cn,
            'Pulse_Width': pw,
            'Pulse_Current': pc,
            'X': 0 #filler
        }

        pkt = self._buildPacket(cmd,fields)

        # print binascii.hexlify(pkt)

        return self._writeRead(pkt)

    def _buildPacket(self, name, fields):
        # get command description
        command = command_dict[name]

        bit_count = 0
        command_bytes = []

        # starting byte has bit 7 set, all others have bit 7 clear
        command_byte = 0b10000000

        for field in command['field_names']:
            # if field is not present, just move on
            # (added because of variable packet size in update command)
            try:
                fv = fields[field]
                fs = command['field_size'][field]
            except KeyError:
                continue

            # if field is a filler ('X'), just update bit_count and move on
            if field[0] == 'X':
                bit_count += command['field_size'][field]
                continue

            # print 'field:',field
            # print 'fv:','{:011b}'.format(fv)
            # print 'fs:',fs

            # figure out where the field should be placed in byte
            shift = 7 - (bit_count + fs)

            # field fits in current command_byte or should be split
            if shift >= 0:
                # insert fv in the right place in command_byte
                command_byte |= fv << shift
                bit_count += fs

                # print 'command_byte','{:08b}'.format(command_byte)

                # check if byte is full and we should start a new one
                if bit_count == 7:
                    command_bytes.append(command_byte)
                    command_byte = 0b00000000
                    bit_count = 0
            else:
                # field is split! get the size and shift of each part
                fs_current = fs + shift
                fs_next = -shift

                shift_right = -shift
                shift_left = 7 - fs_next

                # insert fv in the right place in current command_byte
                command_byte |= fv >> shift_right

                # print 'command_byte','{:08b}'.format(command_byte)

                # append current command_byte to command_bytes
                command_bytes.append(command_byte)
                command_byte = 0b00000000
                bit_count = 0

                # clear bits in fv that have already been used
                fv = self._bitclear(fv, range(fs-fs_current,fs))

                # insert remaining fv in the right place in command_byte
                command_byte |= fv << shift_left
                bit_count += fs_next

                # print 'command_byte','{:08b}'.format(command_byte)

                # check if byte is full and we should start a new one
                if bit_count == 7:
                    command_bytes.append(command_byte)
                    command_byte = 0b00000000
                    bit_count = 0

        # print 'final results:'
        # for i in range(len(command_bytes)):
        #     print i,'{:08b}'.format(command_bytes[i])

        pkt = bytearray(command_bytes)

        # print binascii.hexlify(pkt)

        return pkt

    def _bitset(self, number, bitlist):
        for bit in bitlist:
            number |= (1 << bit)

        return number

    def _bitclear(self, number, bitlist):
        for bit in bitlist:
            number &= ~(1 << bit)

        return number

    def _writeRead(self, packet):
        self.serial_port.write(packet)

        ack = struct.unpack('B',self.serial_port.read(1))[0]

        # print 'ack', '{0:08b}'.format(ack)

        ident = ack >> 6
        error_code = ack & 1
        # print 'ident', reverse_command_dict[ident]
        # print 'error_code', error_code

        return {0: False, 1: True}[error_code]

    def reconnect(self):
        self.serial_port.close()
        try:
            self.serial_port = serial.Serial(port=port, baudrate=115200, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_TWO, rtscts=True)
        except:
            traceback.print_exc()
            return False

        return True
