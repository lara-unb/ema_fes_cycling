#!/usr/bin/env python

import rospy
import ema.modules.control as control

# import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import SetBool
from ema_common_msgs.msg import Stimulator

# import utilities
from tf import transformations
from math import pi

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
global main_current # reference current at the moment
global current_limit # maximum intensity
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
button_event = False
main_current = 0
current_limit = 0
# auto_on = False
# auto_max_current = 0
# auto_minvel = 0
# auto_add_current = 0

# initial current amplitude
stim_current = {
    'Ch1': 0,'Ch2': 0,
    'Ch3': 0,'Ch4': 0,
    'Ch5': 0,'Ch6': 0,
    'Ch7': 0,'Ch8': 0
}

# initial pulse width
stim_pw = { 
    'Ch1': 500,'Ch2': 500,
    'Ch3': 500,'Ch4': 500,
    'Ch5': 500,'Ch6': 500,
    'Ch7': 500,'Ch8': 500
}

# stim channel mapping
stim_order = [
    'Ch1','Ch2',
    'Ch3','Ch4',
    'Ch5','Ch6',
    'Ch7','Ch8'
]

# def server_callback(config):
#     global stim_current
#     global stim_pw
#     global auto_on
#     global auto_max_current
#     global auto_minvel

#     auto_on = config['AutoC_Enable']
#     auto_max_current = config['AutoC_Shift']
#     auto_minvel = config['AutoC_Velocity']

#     # assign updated server parameters to global vars 
#     # refer to the server node for constraints
#     for ch in stim_order:
#         stim_current[ch] = config[ch+'_Current']
#         stim_pw[ch] = config[ch+'_Pulse_Width']

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

def change_intensity_callback(req):
    global main_current
    global current_limit

    if req.data:  # Increase
        main_current += 2
        if main_current > current_limit: 
            main_current = current_limit
    else:  # Decrease
        main_current -= 2
        if main_current < 0:
            main_current = 0
    return {'success':True, 'message':str(main_current)}

# def button_callback(data):
#     global on_off
#     global button_event
#     global main_current
    
#     # raise a button_event flag
#     button_event = True
    
#     if data == UInt8(2): # turn on, increase
#         if on_off == False:
#             on_off = True
        
#         main_current += 2

#     elif data == UInt8(1): # decrease
#         if on_off == True:
#             main_current -= 2

#             if main_current < 0:
#                 on_off = False
#                 main_current = 0

#     elif data == UInt8(3): # zero
#         on_off = False
#         main_current = 0

def main():
    global cycles # number of pedal turns
    global new_cycle # a new pedal turn happened
    global cycle_speed # list of current cycle speeds
    global mean_cadence # mean rpm speed of last cycle
    global distance_km # distance travelled in km
    global stim_current # stim current amplitude for each channel
    global main_current # reference current at the moment 
    global current_limit # maximum intensity
    # global auto_add_current # automatic current adjustment - add value

    # init control node
    rospy.init_node('control', anonymous=False)

    # build basic stimulator message
    stimMsg = Stimulator()
    stimMsg.channel = list(range(1,8+1)) # all the 8 channels
    stimMsg.mode = 8*['single'] # no doublets/triplets
    stimMsg.pulse_width = 8*[0] # initialize w/ zeros
    stimMsg.pulse_current = 8*[0]

    # build general messages
    angleMsg = Float64()
    speedMsg = Float64()
    cadenceMsg = Float64()
    distanceMsg = Float64()
    intensityMsg = UInt8()
    signalMsg = Int32MultiArray() # visual stimulator signal
    signalMsg.data = 9*[0] # [index] is the actual channel number

    # communicate with the dynamic server
    # dyn_params = reconfig.Client('server', config_callback = server_callback)
    
    # get control config
    controller = control.Control(rospy.get_param('/ema/control'))

    # init current amplitude
    main_current, stim_current = controller.initialize(stim_current)

    # current maximum amplitude
    current_limit = controller.currentLimit()

    # list provided services
    services = {}
    services['change_intensity'] = rospy.Service('control/change_intensity',
        SetBool, change_intensity_callback)

    # list subscribed topics
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback = pedal_callback)
    
    # list published topics
    pub = {}
    pub['control'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['angle'] = rospy.Publisher('control/angle', Float64, queue_size=10)
    pub['signal'] = rospy.Publisher('control/stimsignal', Int32MultiArray, queue_size=10)
    pub['speed'] = rospy.Publisher('control/speed', Float64, queue_size=10)
    pub['cadence'] = rospy.Publisher('control/cadence', Float64, queue_size=10)
    pub['distance'] = rospy.Publisher('control/distance', Float64, queue_size=10)
    pub['intensity'] = rospy.Publisher('control/intensity', UInt8, queue_size=10)
    
    # define loop rate (in hz)
    rate = rospy.Rate(50)

    # node loop
    while not rospy.is_shutdown():
        # check for a new pedal turn
        if angle[-2]-angle[-1] > 350:
            try: # ignores ZeroDivisionError
                new_cycle = True
                cycles += 1 # count turns
                mean_cadence = sum(cycle_speed)/len(cycle_speed) # simple mean
                cycle_speed = [] # reset list for new cycle
                distance_km = (cycles*3.14159*1.5*66)/100000  #1.5: volta da coroa em relacao ao pneu, 66cm(26pol): diametro do pneu, 100000: cm->km
                # print cycles, mean_cadence, distance_km

                # if auto_on:
                #    stim_current, auto_add_current = controller.automatic(stim_current, auto_add_current, mean_cadence, auto_minvel, auto_max_current)
                #    print auto_add_current
            except:
                pass
        else:
            new_cycle = False
            cycle_speed.append(speed[-1])

        # calculate control signal
        stimfactors = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)  
        
        # get the proportion for each channel
        proportion = controller.multipliers()

        # update current and pw values
        for i, ch in enumerate(stim_order):
            stim_current[ch] = round(main_current*proportion[ch])
            stimMsg.pulse_current[i] = round(stimfactors[i]*stim_current[ch])
            stimMsg.pulse_width[i] = stim_pw[ch]
            signalMsg.data[i+1] = stimMsg.pulse_current[i]# [index] is the actual channel number

            # if new_cycle and auto_on:
            #     dyn_params.update_configuration({ch+'_Current':stim_current[ch]})

        angleMsg.data = angle[-1]
        speedMsg.data = speed[-1]
        cadenceMsg.data = mean_cadence
        distanceMsg.data = distance_km
        intensityMsg.data = main_current

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
        # send intensity update
        pub['intensity'].publish(intensityMsg)

        # wait for next loop
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
