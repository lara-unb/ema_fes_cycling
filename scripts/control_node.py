#!/usr/bin/env python

import rospy
import ema.modules.control as control
import dynamic_reconfigure.client as reconfig

# import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int8
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
global pw_left
global pw_right

global left_current
global right_current
global update_values

left_current = 0
right_current = 0
update_values = False

on_off = False
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0,0]
pw_left = [0,0]
pw_right = [0,0]

def server_callback(config):
    global left_current
    global right_current
    global update_values
    update_values = True

    # print('Server callback', config['current_left'], config['current_right'], left_current, right_current)

    if config['current_left'] - left_current > 2:
        left_current += 2
    else:
        left_current = config['current_left']


    if config['current_right'] - right_current > 2:
        right_current += 2
    else:
        right_current = config['current_right']

    # print('End server callback', config['current_left'], config['current_right'], left_current, right_current)

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
    #print time[-1], angle[-1], speed[-1], speed_err[-1]

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
    global update_values

    # init control node
    controller = control.Control(rospy.init_node('control', anonymous=False))

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback = server_callback)

    # get control config
    config_dict = rospy.get_param('/ema/control')

    # list subscribed topics
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback = pedal_callback)
    # sub['remote'] = rospy.Subscriber('imu/remote_buttons', Int8, callback = remote_callback)
    
    # list published topics
    pub = {}
    pub['control'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['angle'] = rospy.Publisher('control/angle', Float64, queue_size=10)
    pub['speed'] = rospy.Publisher('control/speed', Float64, queue_size=10)
    
    # define loop rate (in hz)
    rate = rospy.Rate(50)
    
    # build basic stimulator message
    stimMsg = Stimulator()
    stimMsg.channel = [1, 2]
    stimMsg.mode = ['single', 'single']
    stimMsg.pulse_width = [500, 500]
    
    # build basic angle message
    angleMsg = Float64()
    
    # build basic speed message
    speedMsg = Float64()

    # node loop
    while not rospy.is_shutdown():
        # calculate control signal
        # theta = rospy.get_param('/ema_fes_cycling/')      
        # rospy.loginfo("min %d %s",theta["angle_"+id+"_min"],id)
        # rospy.loginfo("max %d %s",theta["angle_"+id+"_max"],id)

     #    if on_off == True:
     #        	pwl, pwr = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)
    	# else:
     #        pwl, pwr = [0, 0]

        # parameters update
        if update_values is True:
            params = { 'current_left' : left_current, 'current_right' : right_current }
            dyn_params.update_configuration(params)
            update_values = False

        bool_left, bool_right = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)
        stimMsg.pulse_current = [bool_left*left_current, bool_right*right_current]

        # send stimulator update
        # stimMsg.pulse_width = [pwl, pwr]
        pub['control'].publish(stimMsg)
        

        # send angle update
        angleMsg.data = angle[-1]
        pub['angle'].publish(angleMsg)
        
        # send speed update
        speedMsg.data = speed[-1]
        pub['speed'].publish(speedMsg)
        
        # store control signal for plotting
        # pw_left.append(pwl)
        # pw_right.append(pwr)

        # wait for next control loop
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
