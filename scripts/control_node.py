#!/usr/bin/env python

import rospy
import ema.modules.control as control
import dynamic_reconfigure.client as reconfig

# import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
from ema_common_msgs.msg import Stimulator

# import utilities
from math import pi
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

on_off = False
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0,0]

stim_current = {
    'Quad': {'Left': 0, 'Right': 0},
    'Hams': {'Left': 0, 'Right': 0},
    'Glut': {'Left': 0, 'Right': 0}
}

stim_pw = {
    'Quad': {'Left': 0, 'Right': 0},
    'Hams': {'Left': 0, 'Right': 0},
    'Glut': {'Left': 0, 'Right': 0}
}

# CH1 - Left Quad, CH2 - Right Quad
# CH3 - Left Hams, CH4 - Right Hams
# CH5 - Left Glut, CH6 - Right Glut
stim_order = ['Quad_Left','Quad_Right',
              'Hams_Left','Hams_Right',
              'Glut_Left','Glut_Right']

# Apply a progressive change to the stimulation current
def current_ramp():
    global stim_current
    progressive = [0,0]

    # conditions for current progressive increment
    if bool_left:
        if 0 <= progressive[0] <= 1:
            progressive[0] += 0.1
            if progressive[0] > 1:
                progressive[0] = 1
    else:
        if 0 <= progressive[0] <= 1:
            progressive[0] -= 0.1
            if progressive[0] < 0:
                progressive[0] = 0

    if bool_right:
        if 0 <= progressive[1] <= 1:
            progressive[1] += 0.1
            if progressive[1] > 1:
                progressive[1] = 1
    else:
        if 0 <= progressive[1] <= 1:
            progressive[1] -= 0.1
            if progressive[1] < 0:
                progressive[1] = 0
    # print(progressive)
    return progressive

def server_callback(config):
    global stim_current
    global stim_pw

    # assign updated server parameters to global vars 
    # refer to the server node for constraints
    for x in stim_order:
        muscle = x[:4] # Quad, Hams or Glut
        side = x[5:] # Left or Right

        if config[muscle[0]+'_Enable']:
            stim_current[muscle][side] = config[muscle[0]+'_Current_'+side]
            stim_pw[muscle][side] = config[muscle[0]+'_Pulse_Width_'+side]
        else:
            stim_current[muscle][side] = 0
            stim_pw[muscle][side] = 0

def pedal_callback(data):
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

    # print latest
    # print time[-1], angle[-1], speed[-1], speed_err[-1]

def remote_callback(data):
    global stimMsg
    
    global on_off
    
    if data == Int8(1):
        if on_off == False:
            on_off = True
        
        stimMsg.pulse_current[0] += 1
        stimMsg.pulse_current[1] += 1
        rospy.loginfo("Stimulator current is now %d", stimMsg.pulse_current[0])
    elif data == Int8(2):
        if on_off == True:
            stimMsg.pulse_current[0] -= 1
            stimMsg.pulse_current[1] -= 1
        
        if stimMsg.pulse_current[0] < 6:
            on_off = False
            rospy.loginfo("Turned off controller")
        else:
            rospy.loginfo("Stimulator current is now %d", stimMsg.pulse_current[0])
    elif data == Int8(3):
        on_off = False
        stimMsg.pulse_current[0] = 5
        stimMsg.pulse_current[1] = 5
        rospy.loginfo("Turned off controller")

def main():
    global stimMsg
    global stim_current
    global stim_pw

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
    signalMsg = Int32MultiArray() # visual stimulator signal
    signalMsg.data = 7*[0] # [index] is the actual channel number

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback = server_callback)
    
    # get control config
    controller = control.Control(rospy.get_param('/ema/control'))

    # list subscribed topics
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback = pedal_callback)
    # sub['remote'] = rospy.Subscriber('imu/remote_buttons', Int8, callback = remote_callback)
    
    # list published topics
    pub = {}
    pub['control'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['angle'] = rospy.Publisher('control/angle', Float64, queue_size=10)
    pub['signal'] = rospy.Publisher('control/stimsignal', Int32MultiArray, queue_size=10)
    pub['speed'] = rospy.Publisher('control/speed', Float64, queue_size=10)

    # define node loop rate (in hz)
    rate = rospy.Rate(50)

    # node loop
    while not rospy.is_shutdown():
        # calculate control signal
        stimfactors = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)
        
        # update current and pw values
        for i, x in enumerate(stim_order):
            muscle = x[:4] # Quad, Hams or Glut
            side = x[5:] # Left or Right
            stimMsg.pulse_current[i] = round(stimfactors[i]*stim_current[muscle][side])
            stimMsg.pulse_width[i] = stim_pw[muscle][side]
            signalMsg.data[i+1] = stimMsg.pulse_current[i] # [index] is the actual channel number

        angleMsg.data = angle[-1]

        current_speed = (angle[-1] - angle[-2])*50
        if current_speed < -1000:
            current_speed = 0
        speedMsg.data = current_speed

        # send stimulator update
        pub['control'].publish(stimMsg)
        # send angle update
        pub['angle'].publish(angleMsg)
        # send signal update
        pub['signal'].publish(signalMsg)
        # send speed update
        pub['speed'].publish(speedMsg)

        # wait for next control loop
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
