#!/usr/bin/env python

import rospy
import ema.modules.control as control

# Import ROS msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Duration
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from ema_common_msgs.msg import Stimulator
from ema_common_msgs.srv import SetUInt16

# Import utilities
from tf import transformations
from math import pi
import os
import yaml
import rospkg
import rosnode

# Global variables
global status # 'off', 'training' or 'racing'
global angle # List of pedal angles
global speed # List of pedal angular speeds
global speed_ref # Reference speed
global speed_err # Speed error
global time # List of imu timestamps
global start_time  # Instant when control is turned on
global cycles # Number of pedal turns
global new_cycle # Flag when a new pedal turn happens
global cycle_speed # List of current cycle speeds
global mean_cadence # Mean RPM speed of last cycle
global distance_km # Distance travelled in km
global stim_current # Stim current amplitude for each channel
global stim_pw # Stim pulse width for each channel
global main_current # Reference current at the moment
global current_limit # Maximum intensity

status = 'off'
angle = [0,0]
speed = [0,0]
speed_ref = 300
speed_err = [0,0]
time = [0,0]
start_time = 0
cycles = 0
new_cycle = False
cycle_speed = [0,0]
mean_cadence = 0
distance_km = 0
button_event = False
main_current = 0
current_limit = 0

# Initial current amplitude
stim_current = {
    'Ch1': 0,'Ch2': 0,
    'Ch3': 0,'Ch4': 0,
    'Ch5': 0,'Ch6': 0,
    'Ch7': 0,'Ch8': 0
}

# Initial pulse width
stim_pw = { 
    'Ch1': 500,'Ch2': 500,
    'Ch3': 500,'Ch4': 500,
    'Ch5': 500,'Ch6': 500,
    'Ch7': 500,'Ch8': 500
}

# Stim channel mapping
stim_order = [
    'Ch1','Ch2',
    'Ch3','Ch4',
    'Ch5','Ch6',
    'Ch7','Ch8'
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

    rospy.loginfo('Set status: service request')
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
        main_current = rospy.get_param('control/training_current')
        return {'success':True, 'message':'training'}
    elif status == 'racing':
        start_time = rospy.Time.now()+rospy.Duration(32)
        main_current = rospy.get_param('control/racing_current')
        return {'success':True, 'message':'racing'}

def set_pulse_width_callback(req):
    """ROS Service handler to set the stim pulse width.

    Attributes:
        req (int): new pulse width
    """
    global stim_pw

    rospy.loginfo('Set pulse width: service request')
    pw_now = rospy.get_param('control/pulse_width')
    msg = str(pw_now)
    if pw_now != req.data:
        if req.data >= 0:
            rospy.set_param('control/pulse_width', req.data)  # Change the param server
            rospack = rospkg.RosPack()
            control_cfg_path = rospack.get_path('ema_fes_cycling')+'/config/control.yaml'
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
    init_current = rospy.get_param('control/training_current')
    msg = str(init_current)
    if init_current != req.data:
        if req.data >= 0:
            rospy.set_param('control/training_current', req.data)  # Change the param server
            rospack = rospkg.RosPack()
            control_cfg_path = rospack.get_path('ema_fes_cycling')+'/config/control.yaml'
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
    global main_current
    global current_limit

    rospy.loginfo('Change intensity: service request')
    if req.data:  # Increase
        main_current += 2
        if main_current > current_limit: 
            main_current = current_limit
    else:  # Decrease
        main_current -= 2
        if main_current < 0:
            main_current = 0
    return {'success':True, 'message':str(main_current)}

def pedal_callback(data):
    global angle
    global speed
    global speed_err
    global time
    # pi = 3.14159
    
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
        yaw = (yaw/pi) * 180
        if abs(roll) > (pi*0.5):
            yaw = 180-yaw
    else:
        yaw = (yaw/pi) * 180
        if abs(roll) > (pi*0.5):
            yaw = 180 - yaw
        else:
            yaw = 360 + yaw
    # Get yaw angle in degrees
    angle.append(yaw)
    # Get angular speed in degrees/s
    speed.append(data.angular_velocity.y*(180/pi))
    # Get angular speed error
    speed_err.append(speed_ref - speed[-1])

def main():
    global status # 'off', 'training' or 'racing'
    global start_time  # Instant when control is turned on
    global cycles # Number of pedal turns
    global new_cycle # Flag when a new pedal turn happens
    global cycle_speed # List of current cycle speeds
    global mean_cadence # Mean RPM speed of last cycle
    global distance_km # Distance travelled in km
    global stim_current # Stim current amplitude for each channel
    global stim_pw # Stim pulse width for each channel
    global main_current # Reference current at the moment
    global current_limit # Maximum intensity

    # Init control node
    rospy.loginfo('Initializing node')
    rospy.init_node('control')

    # Build basic stimulator message
    stimMsg = Stimulator()
    stimMsg.channel = list(range(1,8+1)) # All the 8 channels
    stimMsg.mode = 8*['single'] # No doublets/triplets
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
    signalMsg = Int32MultiArray() # Visual stimulator signal
    signalMsg.data = 9*[0] # [index] is the actual channel number
    
    # Get control config
    rospy.loginfo('Building manager class')
    controller = control.Control(rospy.get_param('/ema/control'))

    # Init controller setup and stimulation parameters
    main_current, stim_current, stim_pw = controller.initialize(stim_current, stim_pw)

    # Current maximum amplitude
    current_limit = controller.currentLimit()

    # List provided services
    rospy.loginfo('Setting up services')
    services = {}
    services['reboot'] = rospy.Service('control/reboot',
        Empty, reboot_callback)
    services['kill_all'] = rospy.Service('control/kill_all',
        Empty, kill_all_callback)
    services['kill_node'] = rospy.Service('control/kill_node',
        Empty, kill_node_callback)
    services['set_status'] = rospy.Service('control/set_status',
        SetUInt16, set_status_callback)
    services['set_pulse_width'] = rospy.Service('control/set_pulse_width',
        SetUInt16, set_pulse_width_callback)
    services['set_init_intensity'] = rospy.Service('control/set_init_intensity',
        SetUInt16, set_init_intensity_callback)
    services['change_intensity'] = rospy.Service('control/change_intensity',
        SetBool, change_intensity_callback)

    # List subscribed topics
    rospy.loginfo('Setting up topics')
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback=pedal_callback)
    
    # List published topics
    pub = {}
    pub['control'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['status'] = rospy.Publisher('control/status', String, queue_size=10)
    pub['angle'] = rospy.Publisher('control/angle', Float64, queue_size=10)
    pub['signal'] = rospy.Publisher('control/stimsignal', Int32MultiArray, queue_size=10)
    pub['speed'] = rospy.Publisher('control/speed', Float64, queue_size=10)
    pub['cadence'] = rospy.Publisher('control/cadence', Float64, queue_size=10)
    pub['distance'] = rospy.Publisher('control/distance', Float64, queue_size=10)
    pub['intensity'] = rospy.Publisher('control/intensity', UInt8, queue_size=10)
    pub['elapsed'] = rospy.Publisher('control/elapsed', Duration, queue_size=10)
    
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

        # Calculate control signal
        stimfactors = controller.calculate(angle[-1], speed[-1], speed_ref, speed_err)
        rospy.logdebug('Stimfactors: %s', stimfactors)
        
        # Get the proportion for each channel
        proportion = controller.multipliers()

        # Update current and pw values
        for i, ch in enumerate(stim_order):
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

        # Send stimulator update
        pub['control'].publish(stimMsg)
        # Send status update
        pub['status'].publish(statusMsg)
        # Send angle update
        pub['angle'].publish(angleMsg)
        # Send signal update
        pub['signal'].publish(signalMsg)
        # Send speed update
        pub['speed'].publish(speedMsg)
        # Send cadence update
        pub['cadence'].publish(cadenceMsg)
        # Send distance update
        pub['distance'].publish(distanceMsg)
        # Send intensity update
        pub['intensity'].publish(intensityMsg)
        # Send elapsed time update
        pub['elapsed'].publish(elapsedMsg)

        # Wait for next loop
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
