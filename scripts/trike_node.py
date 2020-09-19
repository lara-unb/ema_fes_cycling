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

# # Python 2 and 3 compatibility
# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function
# from builtins import *

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

# Dictionary used for stimulation parameters
template_dict = {
    'Ch1': 0, 'Ch2': 0,
    'Ch3': 0, 'Ch4': 0,
    'Ch5': 0, 'Ch6': 0,
    'Ch7': 0, 'Ch8': 0
}


class TrikeWrapper(object):
    """A class used to wrap the trike functionalities and establish a ROS
    interface for its module.
    """
    def __init__(self):
        # Declare internal components
        self.trike = trike.Trike(rospy.get_param('trike'))  # Init from lower level class
        self.platform = rospy.get_param('platform')  # Where it's running - desktop/embedded
        self.paramserver = {}  # Interface with ROS parameters
        self.topics = {'pub': {},'sub': {}}  # ROS Topics - published/subscribed
        self.services = {'prov': {},'req': {}}  # ROS Services - provided(server)/requested(client)
        self.msgs = {}  # Exchanged msgs through the topics
        self.stim_pw = {}  # Stimulation pulse width for each channel
        self.stim_current = {}  # Maximum stimulation current for each channel at the moment
        self.stim_current_max = 0  # Maximum current from all channels at the moment
        self.stim_current_now = 9*[0]  # Instant stimulation current for each channel

        # Other components
        self.on_off = False  # System on/off
        self.angle = [0]  # List of pedal angles
        self.speed = [0]  # List of pedal angular speeds
        self.speed_ref = 300  # Reference speed
        self.speed_err = [0]  # Speed error
        self.time = [0]  # List of imu timestamps

        # Initial setup
        self.build_msgs()
        self.set_topics()
        self.initialize()

    def initialize(self):
        """Configure the proper setup according to the present platform."""
        # Initialize current and pw dicts
        self.stim_current.update(template_dict)
        self.stim_pw.update(template_dict)
        # Exclusive initialization for embedded
        if self.platform == 'rasp':
            # Initialize control
            self.stim_current_max, self.stim_current, self.stim_pw = self.trike.initialize(
                self.stim_current, self.stim_pw)
        # Exclusive initialization for desktop
        elif self.platform == 'pc':
            # Communicate with the dynamic server
            self.paramserver = dynamic_reconfigure.client.Client('trike_config',
                config_callback=self.server_callback)  # 'server_node_name'

    def build_msgs(self):
        """Prepare and build msgs according to their ROS Msg type."""
        # Build stimulator msg
        stim_msg = Stimulator()
        stim_msg.channel = list(range(1, 8+1))  # All the 8 channels
        stim_msg.mode = 8*['single']  # No doublets/triplets
        stim_msg.pulse_width = 8*[0]  # Init w/ zeros
        stim_msg.pulse_current = 8*[0]
        # Build signal msg to publish instant current
        signal_msg = Int32MultiArray()
        signal_msg.data = 9*[0]  # Index corresponds to stimulation channel 1-8
        # Assign designated internal variables
        self.msgs['stim'] = stim_msg
        self.msgs['signal'] = signal_msg
        self.msgs['angle'] = Float64()
        self.msgs['speed'] = Float64()
        self.msgs['intensity'] = UInt8()

    def set_topics(self):
        """Declare the subscribed and published ROS Topics."""
        # List subscribed topics:
        self.topics['sub']['pedal'] = rospy.Subscriber('imu/pedal', Imu, self.pedal_callback)
        # List published topics:
        self.topics['pub']['stim'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
        self.topics['pub']['signal'] = rospy.Publisher('trike/signal', Int32MultiArray, queue_size=10)
        self.topics['pub']['angle'] = rospy.Publisher('trike/angle', Float64, queue_size=10)
        self.topics['pub']['speed'] = rospy.Publisher('trike/speed', Float64, queue_size=10)
        self.topics['pub']['intensity'] = rospy.Publisher('trike/intensity', UInt8, queue_size=10)

    def recalculate(self):
        """Reconsider the control action based on present state."""
        # Calculate control factors for each channel
        stimfactors = self.trike.calculate(self.angle[-1], self.speed[-1],
            self.speed_ref, self.speed_err)
        # Update instant current
        for k, v in self.stim_current.items():
            channel = int(''.join(num for num in k if num.isdigit()))  # int in str
            self.stim_current_now[channel] = round(stimfactors[channel-1]*v)

    def update_msgs(self):
        """Modify ROS Msg variables according to their present value."""
        # Update stimulation pulse width and current values
        for k, v in self.stim_pw.items():
            channel = int(''.join(num for num in k if num.isdigit()))  # int in str
            self.msgs['stim'].pulse_width[channel-1] = v
        self.msgs['stim'].pulse_current = self.stim_current_now[1:]
        self.msgs['signal'].data = self.stim_current_now
        # Update other msgs
        self.msgs['angle'].data = self.angle[-1]
        self.msgs['speed'].data = self.speed[-1]
        self.msgs['intensity'].data = self.stim_current_max

    def publish_msgs(self):
        """Publish on all ROS Topics."""
        for name, tp in self.topics['pub'].items():
            tp.publish(self.msgs[name])

    def server_callback(self, config):
        """ROS dynamic reconfigure callback to assign the modified server
        parameters to the equivalent variables.

        Attributes:
            config (dict): server dictionary with all parameters
        """
        # Update the angles and other parameters
        self.trike.updateParam(config)
        # Update internal variables
        maxvalue = self.stim_current_max
        for k in self.stim_current.keys():
            # Find greater value
            if config[k+'Current'] > maxvalue:
                maxvalue = config[k+'Current']
            self.stim_current[k] = config[k+'Current']
            self.stim_pw[k] = config[k+'PulseWidth']
        # Update maximum value
        self.stim_current_max = maxvalue

    def pedal_callback(self, data):
        """ROS Topic callback to process measurements from the pedal IMU.

        Attributes:
            data (Imu): ROS Msg from the pedal IMU sensor
        """
        # Get timestamp:
        self.time.append(data.header.stamp)
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

        #  Update angle and speed data
        self.angle.append(y)
        self.speed.append(data.angular_velocity.y*(180/pi))
        self.speed_err.append(self.speed_ref - self.speed[-1])


def main():
    # Init control node
    rospy.init_node('trike')
    # Create trike auxiliary class
    aux = TrikeWrapper()
    # Define loop rate (in hz)
    rate = rospy.Rate(50)
    # Node loop
    while not rospy.is_shutdown():
        # Control action to update applied stimulation 
        aux.recalculate()
        # Redefine published msgs
        aux.update_msgs()
        # Send the new msgs
        aux.publish_msgs()
        # Wait for next loop
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
