#!/usr/bin/env python

import rospy
import ema.modules.control as control

# import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
from ema_common_msgs.msg import Stimulator

# import utilities
from tf import transformations

# add when embedded
from std_msgs.msg import UInt16

# remove when embedded
import dynamic_reconfigure.client as reconfig
from math import pi
import numpy as np

# global variables
global on_off # system on/off
global angle # list of pedal angles
global speed # list of pedal angular speeds
global speed_ref # reference speed
global speed_err # speed error
global time # list of imu timestamps
global cycles # number of pedal turns
global new_cycle # a new pedal turn happened
global cycle_speed # list of current cycle speeds
global mean_cadence # mean rpm speed of last cycle
global distance_km # distance travelled in km
global stim_current # stim current amplitude for each channel
global stim_pw # stim pulse width for each channel
# global auto_on # automatic current adjustment - on/off
# global auto_max_current # automatic current adjustment - limit
# global auto_minvel # automatic current adjustment - trigger speed
# global auto_add_current # automatic current adjustment - add value

on_off = False
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0,0]
cycles = 0
new_cycle = False
cycle_speed = [0,0]
mean_cadence = 0
distance_km = 0
# auto_on = False
# auto_max_current = 0
# auto_minvel = 0
# auto_add_current = 0

# initial current amplitude
stim_current = {
    'Ch12': {'Odd': 0, 'Even': 0},
    'Ch34': {'Odd': 0, 'Even': 0},
    'Ch56': {'Odd': 0, 'Even': 0},
    'Ch78': {'Odd': 0, 'Even': 0}
}

# initial pulse width
stim_pw = {
    'Ch12': {'Odd': 500, 'Even': 500},
    'Ch34': {'Odd': 500, 'Even': 500},
    'Ch56': {'Odd': 500, 'Even': 500},
    'Ch78': {'Odd': 500, 'Even': 500}
}

# stim channel mapping
stim_order = ['Ch12_Odd','Ch12_Even', # CH1 & CH2
              'Ch34_Odd','Ch34_Even', # CH3 & CH4
              'Ch56_Odd','Ch56_Even', # CH5 & CH6
              'Ch78_Odd','Ch78_Even'] # CH5 & CH6

def server_callback(config):
    global stim_current
    global stim_pw
    global auto_on
    global auto_max_current
    global auto_minvel

    auto_on = config['AutoC_Enable']
    auto_max_current = config['AutoC_Shift']
    auto_minvel = config['AutoC_Velocity']

    # assign updated server parameters to global vars 
    # refer to the server node for constraints
    for x in stim_order:
        channel = x[:4] # Ch12, Ch34, Ch56, Ch78
        side = x[5:] # Odd or Even

        if config[channel[0]+'_Enable']:
            stim_current[channel][side] = config[channel[0]+'_Current_'+side]
            stim_pw[channel][side] = config[channel[0]+'_Pulse_Width_'+side]
        else:
            stim_current[channel][side] = 0
            stim_pw[channel][side] = 0


def pedal_callback(data):
    global angle
    global speed
    global speed_err
    global time
    # pi = 3.14159
    
    # get timestamp
    time.append(data.header.stamp)

    # get angle position
    qx,qy,qz,qw = data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w
    euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='rzyx')

    x = euler[2]
    y = euler[1]

    # correct issues with more than one axis rotating
    if y >= 0:
        y = (y/pi) * 180
        if abs(x) > (pi*0.5):
            y = 180-y            
    else:
        y = (y/pi) * 180
        if abs(x) > (pi*0.5):
            y = 180 - y
        else:
            y = 360 + y

    angle.append(y)

    # get angular speed
    speed.append(data.angular_velocity.y*(180/pi))

    # get error
    speed_err.append(speed_ref - speed[-1])


# def button_callback(data):
#     global on_off
#     global stim_current

#     if data == Int8(1):
#         if on_off == False:
#             on_off = True
        
#         for x in stim_order:
#             channel = x[:4] # Ch12, Ch34, Ch56, Ch78
#             side = x[5:] # Odd or Even

#             if stim_current[channel][side] < 100:
#                 stim_current[channel][side] += 2
#             rospy.loginfo(channel + '_' + side + " current is now %d", stim_current[channel][side])
#     elif data == Int8(2):
#         if on_off == True:

#             for x in stim_order:
#                 channel = x[:4] # Ch12, Ch34, Ch56, Ch78
#                 side = x[5:] # Odd or Even

#                 if stim_current[channel][side] > 2:
#                     stim_current[channel][side] -= 2
#                     rospy.loginfo(channel + '_' + side + " current is now %d", stim_current[channel][side])
#                 else:
#                     rospy.loginfo("Turned off " + channel + '_' + side)
#         else:
#             pass
#     elif data == Int8(3):
#         on_off = False

#         for x in stim_order:
#             channel = x[:4] # Ch12, Ch34, Ch56, Ch78
#             side = x[5:] # Odd or Even

#             if stim_current[channel][side] >= 3:
#                 stim_current[channel][side] = 2

#         rospy.loginfo("Turned off controller")


def main():
    global cycles # number of pedal turns
    global new_cycle # a new pedal turn happened
    global cycle_speed # list of current cycle speeds
    global mean_cadence # mean rpm speed of last cycle
    global distance_km # distance travelled in km
    global stim_current # stim current amplitude for each channel
    # global auto_add_current # automatic current adjustment - add value

    # init control node
    rospy.init_node('control', anonymous=False)

    # build basic stimulator message
    stimMsg = Stimulator()
    stimMsg.channel = list(range(1,6+1)) # all the 6 channels
    stimMsg.mode = 6*['single'] # no doublets/triplets
    stimMsg.pulse_width = 6*[0] # initialize w/ zeros
    stimMsg.pulse_current = 6*[0]

    # build basic angle/speed/signal message
    angleMsg = Float64()
    speedMsg = Float64()
    cadenceMsg = Float64()
    distanceMsg = Float64()
    signalMsg = Int32MultiArray() # visual stimulator signal
    signalMsg.data = 7*[0] # [index] is the actual channel number

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback = server_callback)
    
    # get control config
    controller = control.Control(rospy.get_param('/ema/control'))

    # list subscribed topics
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback = pedal_callback)
    # sub['button'] = rospy.Subscriber('button/value', Int8, callback = button_callback)
    
    # list published topics
    pub = {}
    pub['control'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['angle'] = rospy.Publisher('control/angle', Float64, queue_size=10)
    pub['signal'] = rospy.Publisher('control/stimsignal', Int32MultiArray, queue_size=10)
    pub['speed'] = rospy.Publisher('control/speed', Float64, queue_size=10)
    pub['cadence'] = rospy.Publisher('control/cadence', Float64, queue_size=10)
    pub['distance'] = rospy.Publisher('control/distance', Float64, queue_size=10)

    # define loop rate (in hz)
    rate = rospy.Rate(50)

    # node loop
    while not rospy.is_shutdown():
        # check for a new pedal turn
        if angle[-2]-angle[-1] > 350:
            new_cycle = True
            cycles += 1 # count turns
            mean_cadence = sum(cycle_speed)/len(cycle_speed) # simple mean
            cycle_speed = [] # reset list for new cycle
            distance_km = (cycles*3.14159*1.5*66)/100000  #1.5: volta da coroa em relacao ao pneu, 66cm(26pol): diametro do pneu, 100000: cm->km
            # print cycles, mean_cadence, distance_km

            # if auto_on:
            #    stim_current, auto_add_current = controller.automatic(stim_current, auto_add_current, mean_cadence, auto_minvel, auto_max_current)
            #    print auto_add_current
        else:
            new_cycle = False
            cycle_speed.append(speed[-1])

        # calculate control signal
        stimfactors = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)  

        # update current and pw values
        for i, x in enumerate(stim_order):
            channel = x[:4] # Ch12, Ch34, Ch56, Ch78
            side = x[5:] # Odd or Even

            stimMsg.pulse_current[i] = round(stimfactors[i]*stim_current[channel][side])
            stimMsg.pulse_width[i] = stim_pw[channel][side]
            signalMsg.data[i+1] = stimMsg.pulse_current[i]# [index] is the actual channel number

            # if new_cycle and auto_on:
            #     dyn_params.update_configuration({channel[0]+'_Current_'+side:stim_current[channel][side]})

        angleMsg.data = angle[-1]
        speedMsg.data = speed[-1]
        cadenceMsg.data = mean_cadence
        distanceMsg.data = distance_km

        # print signalMsg.data

        # send stimulator update
        pub['control'].publish(stimMsg)
        # send angle update
        pub['angle'].publish(angleMsg)
        # send signal update
        pub['signal'].publish(signalMsg)
        # send speed update
        pub['speed'].publish(speedMsg)
        # send cadence update
        pub['cadence'].publish(cadenceMsg)
        # send distance update
        pub['distance'].publish(distanceMsg)

        # wait for next loop
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
