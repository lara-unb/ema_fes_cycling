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

# Import ROS msgs
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from ema_common_msgs.msg import Stimulator

# Import utilities
from tf.transformations import euler_from_quaternion

# Other imports
import dynamic_reconfigure.client
from math import pi

# Stim channel mapping
stim_order = [
    'Ch1', 'Ch2',
    'Ch3', 'Ch4',
    'Ch5', 'Ch6',
    'Ch7', 'Ch8'
]


class TrikeWrapper(object):
    """A class used to wrap the trike functionalities and establish a ROS
    interface for its module.
    """
    def __init__(self):
        # Declare internal components
        self.trike = trike.Trike(rospy.get_param('trike'))  # Init from lower level class
        self.platform = rospy.get_param('platform')  # Where it's running - embedded/desktop
        self.paramserver = {}  # Interface with ROS parameters
        self.topics = {'pub': {},'sub': {}}  # ROS Topics - published/subscribed
        self.services = {'prov': {},'req': {}}  # ROS Services - provided(server)/requested(client)
        self.msgs = {}  # Exchanged msgs through the topics
        self.build_msgs()
        self.set_topics()
        # Configure based on platform - embedded/desktop
        if self.platform == 'rasp':
            # Configure control with initial values
            self.trike.apply_initial_config()
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
        # List subscribed topics
        self.topics['sub']['pedal'] = rospy.Subscriber('imu/pedal', Imu, self.pedal_callback)
        # List published topics
        self.topics['pub']['stim'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
        self.topics['pub']['signal'] = rospy.Publisher('trike/signal', Int32MultiArray, queue_size=10)
        self.topics['pub']['angle'] = rospy.Publisher('trike/angle', Float64, queue_size=10)
        self.topics['pub']['speed'] = rospy.Publisher('trike/speed', Float64, queue_size=10)
        self.topics['pub']['intensity'] = rospy.Publisher('trike/intensity', UInt8, queue_size=10)

    def process(self):
        """Update based on present state."""
        self.trike.calculate()

    def update_msgs(self):
        """Modify ROS Msg variables according to their present value."""
        # Get data and setup variables
        _, angle, speed, _ = self.trike.get_latest_measurements()
        pw_dict = self.trike.pulse_width_now
        current_dict = self.trike.stim_current_now
        current_max = self.trike.stim_current_max
        pw_list = 8*[0]
        current_list = 8*[0]
        # Convert stimulation data from dict to list based on stim_order
        for i, channel in enumerate(stim_order):
            pw_list[i] = pw_dict[channel]
            current_list[i] = current_dict[channel]
        # Update msgs
        self.msgs['stim'].pulse_width = pw_list
        self.msgs['stim'].pulse_current = current_list
        self.msgs['signal'].data = instant_current
        self.msgs['angle'].data = angle
        self.msgs['speed'].data = speed
        self.msgs['intensity'].data = current_max

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
        # Dictionary used for stimulation parameters
        template_dict = {
            'Ch1': 0, 'Ch2': 0,
            'Ch3': 0, 'Ch4': 0,
            'Ch5': 0, 'Ch6': 0,
            'Ch7': 0, 'Ch8': 0
        }
        new_pw = template_dict.copy()
        new_current = template_dict.copy()
        for k in template_dict.keys():
            new_pw[k] = config[k+'PulseWidth']
            new_current[k] = config[k+'Current']
        # Update stimulation
        self.trike.update_stim_pw(new_pw)
        self.trike.update_stim_current(new_current)
        # Update the angles and other parameters
        self.trike.update_config(config)

    def pedal_callback(self, data):
        """ROS Topic callback to process measurements from the pedal IMU.

        Attributes:
            data (Imu): ROS Msg from the pedal IMU sensor
        """
        # Get pedal IMU angles
        qx = data.orientation.x
        qy = data.orientation.y
        qz = data.orientation.z
        qw = data.orientation.w
        euler = euler_from_quaternion([qx, qy, qz, qw], axes='rzyx')
        x = euler[2]
        y = euler[1]
        # Correct issues with more than one axis rotating
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
        # Store received data
        latest_time = data.header.stamp
        latest_angle = y
        latest_speed = data.angular_velocity.y*(180/pi)
        self.trike.update_measurements(latest_time, latest_angle, latest_speed)


def main():
    # Init control node
    rospy.init_node('trike')
    # Create auxiliary class
    aux = TrikeWrapper()
    # Define loop rate (in hz)
    rate = rospy.Rate(50)
    # Node loop
    while not rospy.is_shutdown():
        # New interaction
        aux.process()
        # Redefine publisher msgs
        aux.update_msgs()
        # Send the msgs
        aux.publish_msgs()
        # Wait for next loop
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
