#!/usr/bin/env python

"""

Particularly, this code is the main routine for the FES cycling application.
It receives info from other nodes and acts accordingly upon the system.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

"""

# Python 2 and 3 compatibility
# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function
# from builtins import *

import rospy
import modules.trike as trike

# Import ROS msgs
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Duration
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from sensor_msgs.msg import Imu
from ema_common_msgs.msg import Stimulator
from ema_common_msgs.srv import SetUInt16

# Import utilities
from tf import transformations
from math import pi
import os
import yaml
import rospkg
import rosnode

# Other imports
import dynamic_reconfigure.client

# Global variables
global status              # 'off', 'training' or 'racing'
global angle               # List of pedal angles
global speed               # List of pedal angular speeds
global speed_ref           # Reference speed
global speed_err           # Speed error
global time                # List of imu timestamps
global start_time          # Instant when control is turned on
global cycles              # Number of pedal turns
global new_cycle           # Flag for every new pedal turn
global cycle_speed         # List of current cycle speeds
global mean_cadence        # Mean RPM speed of last cycle
global distance_km         # Distance travelled in km
global stim_current        # Stim current for each channel
global stim_pw             # Stim pulse width for each channel
global main_current        # Reference current at the moment
global current_limit       # Maximum intensity
global controller          # Auxiliary object
# global auto_on           # Auto current adjust - on/off
# global auto_max_current  # Auto current adjust - limit
# global auto_minvel       # Auto current adjust - trigger speed
# global auto_add_current  # Auto current adjust - add value

# Set initial param values
status = 'off'
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0, 0]
start_time = None
cycles = 0
new_cycle = False
cycle_speed = [0,0]
mean_cadence = 0
distance_km = 0
main_current = 0
current_limit = 0
controller = None
# auto_on = False
# auto_max_current = 0
# auto_minvel = 0
# auto_add_current = 0

# Current amplitude container
stim_current = {
    'ch1': 0, 'ch2': 0,
    'ch3': 0, 'ch4': 0,
    'ch5': 0, 'ch6': 0,
    'ch7': 0, 'ch8': 0
}

# Pulse width container
stim_pw = {
    'ch1': 500, 'ch2': 500,
    'ch3': 500, 'ch4': 500,
    'ch5': 500, 'ch6': 500,
    'ch7': 500, 'ch8': 500
}

# Stim channel mapping
stim_order = [
    'ch1', 'ch2',
    'ch3', 'ch4',
    'ch5', 'ch6',
    'ch7', 'ch8'
]


def reboot_callback(data):
    """ROS Service handler to reboot the machine.

    Attributes:
        req (Empty): empty input
    """
    rospy.loginfo('Reboot: service request')
    # Attempt to shutdown all nodes except this
    nodes = rosnode.get_node_names('ema')
    nodes.remove(rospy.get_name())
    success_list, fail_list = rosnode.kill_nodes(nodes)
    if fail_list:
        rospy.logerr('Reboot: failed on %s shutdown', fail_list)
    rospy.loginfo('Rebooting machine...')
    os.system('sudo reboot')
    return {}

def kill_all_callback(req):
    """ROS Service handler to shutdown all nodes.

    Attributes:
        req (Empty): empty input
    """
    rospy.loginfo('Shutdown all nodes: service request')
    nodes = rosnode.get_node_names('ema')  # List all nodes running
    nodes.remove(rospy.get_name())  # All nodes except this
    # Shutdown the nodes and rely on roslaunch respawn to restart
    success_list, fail_list = rosnode.kill_nodes(nodes)
    if fail_list:
        rospy.logerr('Shutdown all nodes: failed on %s', fail_list)
    rospy.loginfo('Node shutdown: service request')
    rospy.Timer(rospy.Duration(3), rospy.signal_shutdown, oneshot=True)
    return {}

def kill_node_callback(req):
    """ROS Service handler to shutdown this node.

    Attributes:
        req (Empty): empty input
    """
    # Shutdown this node and rely on roslaunch respawn to restart
    rospy.loginfo('Node shutdown: service request')
    rospy.Timer(rospy.Duration(3), rospy.signal_shutdown, oneshot=True)
    return {}

def set_status_callback(req):
    """ROS Service handler to change the cycling mode.

    Attributes:
        req (UInt16): 0 is off, 1 is training, 2 is racing
    """
    global status
    global start_time
    global main_current

    rospy.logdebug('Set status: service request')
    enum = ['off','training','racing']
    try:
        status = enum[req.data]
    except IndexError as e:
        status = 'off'
        rospy.logerr('Set status: failed, received %s', req.data)
        return {'success':False, 'message':'off'}
    # Change main current based on status
    if status == 'off':
        main_current = 0
        return {'success':True, 'message':'off'}
    elif status == 'training':
        start_time = rospy.Time.now()
        main_current = rospy.get_param('trike/training_current')
        return {'success':True, 'message':'training'}
    elif status == 'racing':
        start_time = rospy.Time.now()+rospy.Duration(32)
        main_current = rospy.get_param('trike/racing_current')
        return {'success':True, 'message':'racing'}

def set_pulse_width_callback(req):
    """ROS Service handler to set the stim pulse width.

    Attributes:
        req (int): new pulse width
    """
    global stim_pw

    rospy.loginfo('Set pulse width: service request')
    pw_now = rospy.get_param('trike/pulse_width')
    msg = str(pw_now)
    if pw_now != req.data:
        if req.data >= 0:
            rospy.set_param('trike/pulse_width', req.data)  # Change the param server
            rospack = rospkg.RosPack()
            control_cfg_path = rospack.get_path('ema_fes_cycling')+'/config/trike.yaml'
            # Change the config yaml file
            with open(control_cfg_path, 'r') as f:
                control_file = yaml.safe_load(f)
                control_file['pulse_width'] = req.data
            with open(control_cfg_path, 'w') as f:
                yaml.safe_dump(control_file, f)
            # Shutdown this node and rely on roslaunch respawn to restart
            msg = str(req.data)
            rospy.loginfo('Node shutdown: new pulse width')
            rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
            return {'success':True, 'message':msg}
    return {'success':False, 'message':msg}

def set_init_intensity_callback(req):
    """ROS Service handler to set the initial stim intensity.

    Attributes:
        req (int): new initial intensity
    """
    rospy.loginfo('Set initial intensity: service request')
    init_current = rospy.get_param('trike/training_current')
    msg = str(init_current)
    if init_current != req.data:
        if req.data >= 0:
            rospy.set_param('trike/training_current', req.data)  # Change the param server
            rospack = rospkg.RosPack()
            control_cfg_path = rospack.get_path('ema_fes_cycling')+'/config/trike.yaml'
            # Change the config yaml file
            with open(control_cfg_path, 'r') as f:
                control_file = yaml.safe_load(f)
                control_file['training_current'] = req.data
            with open(control_cfg_path, 'w') as f:
                yaml.safe_dump(control_file, f)
            # Shutdown this node and rely on roslaunch respawn to restart
            msg = str(req.data)
            rospy.loginfo('Node shutdown: new initial intensity')
            rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
            return {'success':True, 'message':msg}
    return {'success':False, 'message':msg}

def change_intensity_callback(req):
    """ROS Service handler to request a change in intensity.

    Attributes:
        req (bool): 0 to decrease and 1 to increase
    """
    global status
    global main_current
    global current_limit

    rospy.logdebug('Change intensity: service request')
    if status == 'off':
        main_current = 0
    else:
        if req.data:  # Increase
            main_current += 2
            if main_current > current_limit: 
                main_current = current_limit
        else:  # Decrease
            main_current -= 2
            if main_current < 0:
                main_current = 0
    return {'success':True, 'message':str(main_current)}

def server_callback(config):
    """Assign the server parameters to the equivalent variables.

    Attributes:
        config (dict): server dictionary with its parameters
    """
    global stim_current
    global stim_pw
    global controller

    controller.update_param(config)
    for ch in stim_order:
        stim_current[ch] = config[ch+'_current']
        stim_pw[ch] = config[ch+'_pulse_width']

def pedal_callback(data):
    """Process measurements from the pedal IMU sensor.

    Attributes:
        data (Imu): msg from the pedal IMU sensor
    """
    global angle
    global speed
    global speed_err
    global time

    # Get timestamp
    time.append(data.header.stamp)
    # Get angle position
    qx,qy,qz,qw = data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w
    # rzxy - return (pitch, roll, yaw)
    euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='rzxy')
    roll = euler[1]
    yaw = euler[2]
    # Correct issues with range and more than one axis rotating
    if yaw >= 0:
        yaw = (yaw/pi)*180
        if abs(roll) > (pi*0.5):
            yaw = 180-yaw
    else:
        yaw = (yaw/pi)*180
        if abs(roll) > (pi*0.5):
            yaw = 180-yaw
        else:
            yaw = 360+yaw
    # Get yaw angle in degrees
    angle.append(yaw)
    # Get angular speed in degrees/s
    speed.append(data.angular_velocity.y*(180/pi))
    # Get angular speed error
    speed_err.append(speed_ref-speed[-1])

def main():
    global status              # 'off', 'training' or 'racing'
    global start_time          # Instant when control is turned on
    global cycles              # Number of pedal turns
    global new_cycle           # Flag for every new pedal turn
    global cycle_speed         # List of current cycle speeds
    global mean_cadence        # Mean RPM speed of last cycle
    global distance_km         # Distance travelled in km
    global stim_current        # Stim current for each channel
    global stim_pw             # Stim pulse width for each channel
    global main_current        # Reference current at the moment
    global current_limit       # Maximum intensity
    global controller          # Auxiliary object

    # Init control node
    rospy.loginfo('Initializing node')
    rospy.init_node('trike')  # Overwritten by launch file name

    # Build basic stimulator msg
    stimMsg = Stimulator()
    stimMsg.channel = list(range(1,8+1))  # All the 8 channels
    stimMsg.mode = 8*['single']  # No doublets/triplets
    stimMsg.pulse_width = 8*[0] # Initialize w/ zeros
    stimMsg.pulse_current = 8*[0]

    # Build general messages
    statusMsg = String()
    angleMsg = Float64()
    speedMsg = Float64()
    cadenceMsg = Float64()
    distanceMsg = Float64()
    intensityMsg = UInt8()
    elapsedMsg = Duration()
    signalMsg = Int32MultiArray()
    signalMsg.data = 9*[0] # [index] is the actual channel number

    # Get control config
    rospy.loginfo('Building manager class')
    controller = trike.Control(rospy.get_param('trike'))

    # List subscribed topics
    rospy.loginfo('Setting up topics')
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback=pedal_callback)
    # List published topics
    pub = {}
    pub['control'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['status'] = rospy.Publisher('trike/status', String, queue_size=10)
    pub['angle'] = rospy.Publisher('trike/angle', Float64, queue_size=10)
    pub['signal'] = rospy.Publisher('trike/stimsignal', Int32MultiArray, queue_size=10)
    pub['speed'] = rospy.Publisher('trike/speed', Float64, queue_size=10)
    pub['cadence'] = rospy.Publisher('trike/cadence', Float64, queue_size=10)
    pub['distance'] = rospy.Publisher('trike/distance', Float64, queue_size=10)
    pub['elapsed'] = rospy.Publisher('trike/elapsed', Duration, queue_size=10)

    # Retrieve where the code is currently running (change in .launch)
    platform = rospy.get_param('platform')
    # Embedded system exclusive initialization
    if platform == 'rasp':
        # List provided services
        rospy.loginfo('Setting up services')
        services = {}
        services['reboot'] = rospy.Service('trike/reboot',
            Empty, reboot_callback)
        services['kill_all'] = rospy.Service('trike/kill_all',
            Empty, kill_all_callback)
        services['kill_node'] = rospy.Service('trike/kill_node',
            Empty, kill_node_callback)
        services['set_status'] = rospy.Service('trike/set_status',
            SetUInt16, set_status_callback)
        services['set_pulse_width'] = rospy.Service('trike/set_pulse_width',
            SetUInt16, set_pulse_width_callback)
        services['set_init_intensity'] = rospy.Service('trike/set_init_intensity',
            SetUInt16, set_init_intensity_callback)
        services['change_intensity'] = rospy.Service('trike/change_intensity',
            SetBool, change_intensity_callback)
        # Current maximum amplitude
        current_limit = controller.current_limit()
        # Init controller setup and stimulation parameters
        main_current, stim_current, stim_pw = controller.initialize(stim_current, stim_pw)
        # Additional published topics
        pub['intensity'] = rospy.Publisher('trike/intensity', UInt8, queue_size=10)
    # PC system exclusive initialization
    elif platform == 'pc':
        status = 'training'
        start_time = rospy.Time.now()
        # Communicate with the dynamic server
        dyn_params = dynamic_reconfigure.client.Client('trike_config',
                        config_callback=server_callback)  # 'server_node_name'

    # Define loop rate (in hz)
    rate = rospy.Rate(50)
    # Define auxiliary loop variables
    check_half_turn = False
    elapsed_time = rospy.Duration(0)
    # Node loop
    while not rospy.is_shutdown():
        # Check cycling mode
        if status == 'off':
            main_current = 0
        else:
            # Get duration since control was turned on
            elapsed_time = rospy.Time.now()-start_time
            # Check for a new pedal turn
            if (angle[-1] > 160) and (angle[-1] < 200):  # Flag half-turn
                check_half_turn = True
            if check_half_turn:
                # Get greater absolute difference between five consecutive angles
                pack = angle[-5:]
                pack_shifted = angle[-6:-1]
                pack_diff = [item-pack_shifted[i] for i, item in enumerate(pack)]
                greater_diff = max(pack_diff, key=abs)  # Greater absolute difference
                # Account for angles past 360 deg
                if abs(greater_diff) > 300:
                    check_half_turn = False  # Reset half-turn flag
                    try:
                        # Simple mean and 6 for deg/s to rpm
                        mean_cadence = sum(cycle_speed)/(6*len(cycle_speed))
                        # One crankset turn is equivalent to 1.5 wheel turn, the
                        # tire diameter is 66cm(26in) and 100k for cm to km, so...
                        # pi*1.5*66[rpm] = 1[cm/min] and 60/100k[cm/min] = 1[km/h]
                        mean_cadence = pi*0.0594*mean_cadence  # rpm to km/h
                        cycle_speed = []  # Reset list for new cycle
                        # Moving forward and passed warm up period
                        if (greater_diff < 0) and (elapsed_time.to_sec() > 0):
                            cycles += 1  # Count turns
                            distance_km = (cycles*pi*1.5*66)/100000
                    except ZeroDivisionError as e:
                        rospy.logerr(e)
            # Gather speed while waiting for a new pedal turn
            cycle_speed.append(speed[-1])
            rospy.logdebug('Cycle: %d, Cadence: %.2f, Distance: %.2f',
                cycles, mean_cadence, distance_km)
            if status == 'racing':
                # Stop after 8 min or 1.2 km
                if (elapsed_time.to_sec() > 8.0*60) or (distance_km > 1.2):
                    status = 'off'
                    main_current = 0

        # Calculate control signal
        stimfactors = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)
        rospy.logdebug('Stimfactors: %s', stimfactors)
        # Embedded system exclusive
        if platform == 'rasp':
            # Get the proportion for each channel
            proportion = controller.multipliers()
            intensityMsg.data = main_current
            # Send intensity update
            pub['intensity'].publish(intensityMsg)
        # Update current and pw values
        for i, ch in enumerate(stim_order):
            # Embedded system exclusive
            if platform == 'rasp':
                stim_current[ch] = round(main_current*proportion[ch])
            stimMsg.pulse_current[i] = round(stimfactors[i]*stim_current[ch])
            stimMsg.pulse_width[i] = stim_pw[ch]
            signalMsg.data[i+1] = stimMsg.pulse_current[i]  # [index] is the actual channel number

        statusMsg.data = status
        angleMsg.data = angle[-1]
        speedMsg.data = speed[-1]
        cadenceMsg.data = mean_cadence
        distanceMsg.data = distance_km
        intensityMsg.data = main_current
        elapsedMsg.data = elapsed_time
        # Send updates
        pub['control'].publish(stimMsg)
        pub['status'].publish(statusMsg)
        pub['angle'].publish(angleMsg)
        pub['signal'].publish(signalMsg)
        pub['speed'].publish(speedMsg)
        pub['cadence'].publish(cadenceMsg)
        pub['distance'].publish(distanceMsg)
        pub['elapsed'].publish(elapsedMsg)

        # Wait for next loop
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
