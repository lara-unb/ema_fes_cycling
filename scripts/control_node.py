#!/usr/bin/env python

import rospy
import ema.modules.control as control
#import dynamic_reconfigure.client as reconfig

# import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
from ema_common_msgs.msg import Stimulator

##from math import pi
from tf import transformations

# global variables
global on_off
global angle
global speed
global speed_ref
global speed_err
global time
global stim_current
global stim_pw
global n_cycles
global cadence_cycle
global mean_cadence
global current_speed
global auto_add_current
global n_cycles
global new_cycle
global Q_enable
global H_enable
global G_enable

on_off = False
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0,0]

Q_enable = True
H_enable = False
G_enable = False

stim_current = {
    'Quad': {'Left': 0, 'Right': 0},
    'Hams': {'Left': 0, 'Right': 0},
    'Glut': {'Left': 0, 'Right': 0}
}

stim_pw = {
    'Quad': {'Left': 500, 'Right': 500},
    'Hams': {'Left': 0, 'Right': 0},
    'Glut': {'Left': 0, 'Right': 0}
}

stim_order = ['Quad_Left','Quad_Right', # CH1 & CH2
              'Hams_Left','Hams_Right', # CH3 & CH4
              'Glut_Left','Glut_Right'] # CH5 & CH6

def server_callback(config):
    global stim_current
    global stim_pw

    # assign updated server parameters to global vars 
    # refer to the server node for constraints
    for x in stim_order:
        muscle = x[:4] # Quad, Hams or Glut
        side = x[5:] # Left or Right

##        if Q_enable:
##            stim_current[muscle][side] = config[muscle[0]+'_Current_'+side]
##            stim_pw[muscle][side] = config[muscle[0]+'_Pulse_Width_'+side]
##        else:
##            stim_current[muscle][side] = 0
##            stim_pw[muscle][side] = 0
##
##        if H_enable:
##            stim_current[muscle][side] = config[muscle[0]+'_Current_'+side]
##            stim_pw[muscle][side] = config[muscle[0]+'_Pulse_Width_'+side]
##        else:
##            stim_current[muscle][side] = 0
##            stim_pw[muscle][side] = 0
##
##        if G_enable:
##            stim_current[muscle][side] = config[muscle[0]+'_Current_'+side]
##            stim_pw[muscle][side] = config[muscle[0]+'_Pulse_Width_'+side]
##        else:
##            stim_current[muscle][side] = 0
##            stim_pw[muscle][side] = 0
            
        if config[muscle[0]+'_Enable']:
            stim_current[muscle][side] = config[muscle[0]+'_Current_'+side]
            stim_pw[muscle][side] = config[muscle[0]+'_Pulse_Width_'+side]
        else:
            stim_current[muscle][side] = 0
            stim_pw[muscle][side] = 0


def pedal_callback(data):
    pi = 3.14
    
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


def remote_callback(data):
    global stimMsg
    global stim_current    
    global on_off
    
    if data == Int8(1):
        if on_off == False:
            on_off = True
        
        for x in stim_order:
            muscle = x[:4] # Quad, Hams or Glut
            side = x[5:] # Left or Right

            if stim_current[muscle][side] < 100:
                stim_current[muscle][side] += 2
            rospy.loginfo(muscle + '_' + side + " current is now %d", stim_current[muscle][side])
            
        #if stimMsg.pulse_current[0] < 80:
        #    stimMsg.pulse_current[0] += 1
        #    stimMsg.pulse_current[1] += 1

        #rospy.loginfo("Stimulator current is now %d", stimMsg.pulse_current[0])
    elif data == Int8(2):
        if on_off == True:
            #stimMsg.pulse_current[0] -= 1
            #stimMsg.pulse_current[1] -= 1

            for x in stim_order:
                muscle = x[:4] # Quad, Hams or Glut
                side = x[5:] # Left or Right

                if stim_current[muscle][side] > 2:
                    stim_current[muscle][side] -= 2
                    rospy.loginfo(muscle + '_' + side + " current is now %d", stim_current[muscle][side])
                else:
                    rospy.loginfo("Turned off " + muscle + '_' + side)

        
        #if stimMsg.pulse_current[0] < 6:
        #    on_off = False
        #    rospy.loginfo("Turned off controller")

        else:
            pass
            #rospy.loginfo("Stimulator current is now %d", stimMsg.pulse_current[0])
    elif data == Int8(3):
        on_off = False
        #stimMsg.pulse_current[0] = 5
        #stimMsg.pulse_current[1] = 5
        for x in stim_order:
            muscle = x[:4] # Quad, Hams or Glut
            side = x[5:] # Left or Right

            if stim_current[muscle][side] >= 3:
                stim_current[muscle][side] = 2

        rospy.loginfo("Turned off controller")
    
def main():
    global stimMsg
    global stim_current
    global stim_pw
    # global mean_cadence
    # global current_speed
    # global auto_on
    # global auto_max_current
    # global auto_minvel
    # global auto_add_current
    # global n_cycles
    # global new_cycle

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
    # speedMsg = Float64()
    # cycle_speedMsg = Float64()
    # n_cyclesMsg = Float64()
    signalMsg = Int32MultiArray() # visual stimulator signal
    signalMsg.data = 7*[0] # [index] is the actual channel number

    # communicate with the dynamic server
    #dyn_params = reconfig.Client('server', config_callback = server_callback)
    
    # get control config
    controller = control.Control(rospy.get_param('/ema/control'))

    # list subscribed topics
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback = pedal_callback)
    sub['remote'] = rospy.Subscriber('imu/remote_buttons', Int8, callback = remote_callback)
    
    # list published topics
    pub = {}
    pub['control'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['angle'] = rospy.Publisher('control/angle', Float64, queue_size=10)
    pub['signal'] = rospy.Publisher('control/stimsignal', Int32MultiArray, queue_size=10)
    # pub['speed'] = rospy.Publisher('control/speed', Float64, queue_size=10)
    # pub['cycle_speed'] = rospy.Publisher('control/cycle_speed', Float64, queue_size=10)
    # pub['n_cycles'] = rospy.Publisher('control/n_cycles', Float64, queue_size=10)

    # define node loop rate (in hz)
    rate = rospy.Rate(50)

    # node loop
    while not rospy.is_shutdown():
        # calculate control signal
        stimfactors = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)
        
        # if new_cycle:
        #     new_cycle = False
        #     if auto_on:
        #         stim_current, auto_add_current = controller.automatic(stim_current, auto_add_current, mean_cadence, auto_minvel, auto_max_current)
        #         print auto_add_current
        #     change = True
        

        # update current and pw values
        for i, x in enumerate(stim_order):
            muscle = x[:4] # Quad, Hams or Glut
            side = x[5:] # Left or Right

            # if stim_current[muscle][side] > 0:
            #
            #     stim_current[muscle][side] = stim_current[muscle][side] + auto_add_current
            #
            #     if stim_current[muscle][side] > 80:
            #         stim_current[muscle][side] = 80

            # if auto_on and change:
            #     dyn_params.update_configuration({muscle[0]+'_Current_'+side:stim_current[muscle][side]})

            stimMsg.pulse_current[i] = round(stimfactors[i]*stim_current[muscle][side])
            stimMsg.pulse_width[i] = stim_pw[muscle][side]
            signalMsg.data[i+1] = stimMsg.pulse_current[i]# [index] is the actual channel number

        change = False
        angleMsg.data = angle[-1]

        # speedMsg.data = current_speed
        # cycle_speedMsg.data = mean_cadence
        # n_cyclesMsg.data = n_cycles

        # print signalMsg.data
        # send stimulator update
        pub['control'].publish(stimMsg)
        # send angle update
        pub['angle'].publish(angleMsg)
        # send signal update
        pub['signal'].publish(signalMsg)
        # send speed update
        # pub['speed'].publish(speedMsg)
        # send cycle_speed update
        # pub['cycle_speed'].publish(cycle_speedMsg)
        # send n_cycles update
        # pub['n_cycles'].publish(n_cyclesMsg)

        # wait for next control loop
        
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
