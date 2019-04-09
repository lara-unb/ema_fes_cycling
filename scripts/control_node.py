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

on_off = False
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0,0]


global stim_current
global stim_pulse

stim_current = {
    'quad':    {'left': 0, 'right': 0},
    'hams':    {'left': 0, 'right': 0},
    'glut':    {'left': 0, 'right': 0}
}
stim_pulse = {
    'quad':    {'left': 500, 'right': 500},
    'hams':    {'left': 500, 'right': 500},
    'glut':    {'left': 500, 'right': 500}
}

# Apply a progressive change to the stimulation current
def current_ramp(bool_left, bool_right):
    global left_current
    global right_current
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
    prefix = {'quad':'Q_', 'hams':'H_', 'glut':'G_'}

    # assign updated server parameters to global vars 
    # refer to the server node for constraints
    for m, p in prefix.items():

        if config[p+'Enable']:
            stim_current[m]['left'] = config[p + 'Current_Left']
            stim_current[m]['right'] = config[p + 'Current_Right']
        else:
            stim_current[m]['left'] = 0
            stim_current[m]['right'] = 0

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
    global stim_pulse

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback = server_callback)
    
    # init control node
    rospy.init_node('control', anonymous=False)
    
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
    pub['speed'] = rospy.Publisher('control/speed', Float64, queue_size=10)
    pub['signal'] = rospy.Publisher('control/stimsignal', Int32MultiArray, queue_size=10)
    
    # build basic angle message
    angleMsg = Float64()
    
    # build basic speed message
    speedMsg = Float64()

    # build basic stimulator signal message for plotting purposes
    signalMsg = Int32MultiArray()
    # signalMsg.data = []

    # build basic stimulator message
    stimMsg = Stimulator()
    stimMsg.channel = [1,2,3,4,5,6]
    stimMsg.mode = 6*['single']
    stimMsg.pulse_width = 6*[500]
    stimMsg.pulse_current = 6*[0]

    # define loop rate (in hz)
    rate = rospy.Rate(50)

    # node loop
    while not rospy.is_shutdown():

        # calculate control signal based on sensor angle
        bool_left, bool_right = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)

        left_current, right_current = current_ramp(bool_left, bool_right)

        stimMsg.pulse_current = [left_current, right_current]
        # print(stimMsg.pulse_current)

        # send stimulator update
        pub['control'].publish(stimMsg)
        
        # send angle update
        angleMsg.data = angle[-1]
        pub['angle'].publish(angleMsg)
        
        # send speed update
        current_speed = (angle[-1] - angle[-2])*50
        if current_speed < -1000:
            current_speed = 0
        speedMsg.data = current_speed
        pub['speed'].publish(speedMsg)

        # send signal update
        signalMsg.data = [left_current, right_current]
        pub['signal'].publish(signalMsg)

        # wait for next control loop
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
