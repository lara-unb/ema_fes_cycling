#!/usr/bin/env python

import rospy
import ema.modules.control as control

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

on_off = False
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0,0]
pw_left = [0,0]
pw_right = [0,0]

def pedal_callback(data):
    # get timestamp
    time.append(data.header.stamp)

    # get angle position
    qx,qy,qz,qw = data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w
    euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')
    if euler[0] >= 0:
        angle.append(euler[0]*(180/pi))
    else:
        angle.append(360+euler[0]*(180/pi))
        
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
    
    # init control node
    controller = control.Control(rospy.init_node('control', anonymous=False))
    
    # get control config
    config_dict = rospy.get_param('/ema_trike/control')
    
    # list subscribed topics
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback = pedal_callback)
    sub['remote'] = rospy.Subscriber('imu/remote_buttons', Int8, callback = remote_callback)
    
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
    stimMsg.pulse_current = [18, 18]
    
    # build basic angle message
    angleMsg = Float64()
    
    # build basic speed message
    speedMsg = Float64()
    
    # node loop
    while not rospy.is_shutdown():
        # calculate control signal
        if on_off == True:
            pwl, pwr = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)
        else:
            pwl, pwr = [0, 0]
        
        # send stimulator update
        stimMsg.pulse_width = [pwl, pwr]
        pub['control'].publish(stimMsg)
        
        # send angle update
        angleMsg.data = angle[-1]
        pub['angle'].publish(angleMsg)
        
        # send speed update
        speedMsg.data = speed[-1]
        pub['speed'].publish(speedMsg)
        
        # store control signal for plotting
        pw_left.append(pwl)
        pw_right.append(pwr)
        
        # wait for next control loop
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
