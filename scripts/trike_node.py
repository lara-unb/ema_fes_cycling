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

import rospy
import modules.trike as trike

# Import ROS msgs:
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from ema_common_msgs.msg import Stimulator

# Import utilities:
from tf import transformations

# Other imports:
import dynamic_reconfigure.client
from math import pi

# Global variables:
global on_off              # System on/off
global angle               # List of pedal angles
global speed               # List of pedal angular speeds
global speed_ref           # Reference speed
global speed_err           # Speed error
global time                # List of imu timestamps
global cycles              # Number of pedal turns
global new_cycle           # Flag for every new pedal turn
global cycle_speed         # List of current cycle speeds
global mean_cadence        # Mean RPM speed of last cycle
global distance_km         # Distance travelled in km
global stim_current        # Stim current for each channel
global stim_pw             # Stim pulse width for each channel
global button_event        # Flag for button press
global main_current        # Reference current at the moment
global controller
# global auto_on           # Auto current adjust - on/off
# global auto_max_current  # Auto current adjust - limit
# global auto_minvel       # Auto current adjust - trigger speed
# global auto_add_current  # Auto current adjust - add value

# Set initial param values:
on_off = False
angle = [0, 0]
speed = [0, 0]
speed_ref = 300
speed_err = [0, 0]
time = [0, 0]
cycles = 0
new_cycle = False
cycle_speed = [0, 0]
mean_cadence = 0
distance_km = 0
button_event = False
main_current = 0
controller = None
# auto_on = False
# auto_max_current = 0
# auto_minvel = 0
# auto_add_current = 0

stim_current = {
    'Ch1': 0, 'Ch2': 0,
    'Ch3': 0, 'Ch4': 0,
    'Ch5': 0, 'Ch6': 0,
    'Ch7': 0, 'Ch8': 0
}

stim_pw = {
    'Ch1': 500, 'Ch2': 500,
    'Ch3': 500, 'Ch4': 500,
    'Ch5': 500, 'Ch6': 500,
    'Ch7': 500, 'Ch8': 500
}

# Stim channel mapping:
stim_order = [
    'Ch1', 'Ch2',
    'Ch3', 'Ch4',
    'Ch5', 'Ch6',
    'Ch7', 'Ch8'
]


def server_callback(config):
    """Assign the server parameters to the equivalent variables.

    Attributes:
        config (dict): server dictionary with its parameters
    """
    global stim_current
    global stim_pw
    global controller

    controller.updateParam(config)

    for ch in stim_order:
        stim_current[ch] = config[ch+'Current']
        stim_pw[ch] = config[ch+'PulseWidth']


def pedal_callback(data):
    """Process measurements from the pedal IMU sensor.

    Attributes:
        data (Imu): msg from the pedal IMU sensor
    """
    global angle
    global speed
    global speed_err
    global time
    # pi = 3.14159

    # Get timestamp:
    time.append(data.header.stamp)

    # Get pedal IMU angles:
    qx = data.orientation.x
    qy = data.orientation.y
    qz = data.orientation.z
    qw = data.orientation.w
    euler = transformations.euler_from_quaternion(
                [qx, qy, qz, qw], axes='rzyx')
    x = euler[2]
    y = euler[1]

    # Correct issues with more than one axis rotating:
    if y >= 0:
        y = (y/pi)*180
        if abs(x) > (pi*0.5):
            y = 180-y
    else:
        y = (y/pi)*180
        if abs(x) > (pi*0.5):
            y = 180-y
        else:
            y = 360+y

    angle.append(y)

    # Get angular speed:
    speed.append(data.angular_velocity.y*(180/pi))

    # Get speed error for controller:
    speed_err.append(speed_ref - speed[-1])


def button_callback(data):
    """Process the button commands from the user.

    Attributes:
        data (UInt8): latest msg from the buttons
    """
    global on_off
    global button_event
    global main_current

    # Raise a button pressed flag:
    button_event = True

    # Turn on or increase intensity:
    if data == UInt8(2):
        if not on_off:
            on_off = True
        main_current += 2

    # Decrease intensity:
    elif data == UInt8(1):
        if on_off:
            main_current -= 2

            if main_current < 0:
                on_off = False
                main_current = 0

    # Zero intensity:
    elif data == UInt8(3):
        on_off = False
        main_current = 0


def main():
    global cycles
    global new_cycle
    global cycle_speed
    global mean_cadence
    global distance_km
    global stim_current
    global button_event
    global main_current
    global controller
    # global auto_add_current

    # Init control node:
    rospy.init_node('trike')  # Overwritten by launch file name

    # Build basic stimulator msg:
    stimMsg = Stimulator()
    stimMsg.channel = list(range(1, 8+1))  # All the 8 channels
    stimMsg.mode = 8*['single']  # No doublets/triplets
    stimMsg.pulse_width = 8*[0]  # Init w/ zeros
    stimMsg.pulse_current = 8*[0]

    # Build auxiliary msg to help visualize stimulator signal:
    signalMsg = Int32MultiArray()
    signalMsg.data = 9*[0]  # 1-8 channels, 0 index isnt used

    # Build other msgs:
    angleMsg = Float64()
    speedMsg = Float64()
    cadenceMsg = Float64()
    distanceMsg = Float64()

    # Get control config:
    controller = trike.Control(rospy.get_param('trike'))

    # List subscribed topics:
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback=pedal_callback)

    # List published topics:
    pub = {}
    pub['control'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
    pub['angle'] = rospy.Publisher('trike/angle', Float64, queue_size=10)
    pub['signal'] = rospy.Publisher('trike/stimsignal', Int32MultiArray, queue_size=10)
    pub['speed'] = rospy.Publisher('trike/speed', Float64, queue_size=10)
    pub['cadence'] = rospy.Publisher('trike/cadence', Float64, queue_size=10)
    pub['distance'] = rospy.Publisher('trike/distance', Float64, queue_size=10)

    # Retrieve where the code is currently running (change in .launch):
    platform = rospy.get_param('platform')

    # Embedded system exclusive initialization:
    if platform == 'rasp':
        # Build other msgs:
        displayMsg = UInt8()

        # Initialize control:
        main_current, stim_current = controller.initialize(stim_current)

        # Additional subscribed topics:
        sub['button'] = rospy.Subscriber('button/action', UInt8, callback=button_callback)

        # Additional published topics:
        pub['intensity'] = rospy.Publisher('display/update', UInt8, queue_size=10)

    # PC system exclusive initialization:
    elif platform == 'pc':
        # Communicate with the dynamic server:
        dyn_params = dynamic_reconfigure.client.Client('trike_config',
                        config_callback=server_callback)  # 'server_node_name'

    # Define loop rate (in hz):
    rate = rospy.Rate(50)

    # Node loop:
    while not rospy.is_shutdown():
        # Check for a new pedal turn:
        if angle[-2]-angle[-1] > 350:
            try:  # Ignores ZeroDivisionError
                new_cycle = True
                cycles += 1  # Count turns
                mean_cadence = sum(cycle_speed)/len(cycle_speed)  # Simple mean
                cycle_speed = []  # Reset list for new cycle
                # 1.5: volta da coroa em relacao ao pneu, 66cm(26in): diametro
                # do pneu, 100000: cm para km
                distance_km = (cycles*3.14159*1.5*66)/100000
                # print cycles, mean_cadence, distance_km

                # if auto_on:
                #    stim_current, auto_add_current = controller.automatic(
                #             stim_current, auto_add_current, mean_cadence,
                #             auto_minvel, auto_max_current)
                #    print auto_add_current
            except:
                pass
        else:
            new_cycle = False
            cycle_speed.append(speed[-1])

        # Calculate control factor:
        stimfactors = controller.calculate(angle[-1], speed[-1], 
                        speed_ref, speed_err)

        # Embedded system exclusive:
        if platform == 'rasp':
            # Get the proportion for each channel:
            proportion = controller.multipliers()

            # Limit the stimulation intensity:
            if main_current > controller.currentLimit():
                main_current = controller.currentLimit()

            if button_event:
                button_event = False
                displayMsg.data = main_current
                # Send display update:
                pub['intensity'].publish(displayMsg)

        # Update current and pw values:
        for i, ch in enumerate(stim_order):
            # Embedded system exclusive:
            if platform == 'rasp':
                stim_current[ch] = round(main_current*proportion[ch])

            stimMsg.pulse_current[i] = round(stimfactors[i]*stim_current[ch])
            stimMsg.pulse_width[i] = stim_pw[ch]
            signalMsg.data[i+1] = stimMsg.pulse_current[i]  # 1-8 channels, 0 index isnt used

            # if new_cycle and auto_on:
            #     dyn_params.update_configuration({ch+'_Current':stim_current[ch]})

        angleMsg.data = angle[-1]
        speedMsg.data = speed[-1]
        cadenceMsg.data = mean_cadence
        distanceMsg.data = distance_km

        # print signalMsg.data

        # Send updates:
        pub['control'].publish(stimMsg)
        pub['angle'].publish(angleMsg)
        pub['signal'].publish(signalMsg)
        pub['speed'].publish(speedMsg)
        pub['cadence'].publish(cadenceMsg)
        pub['distance'].publish(distanceMsg)

        # Wait for next loop:
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
