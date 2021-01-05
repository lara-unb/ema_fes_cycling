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

# Stimulation channel mapping
stim_order = [
    'ch1', 'ch2',
    'ch3', 'ch4',
    'ch5', 'ch6',
    'ch7', 'ch8'
]


class TrikeWrapper(object):
    """A class used to wrap the trike functionalities and establish a ROS
    interface for its module.
    """
    def __init__(self):
        rospy.loginfo('Initializing trike')
        self.trike = trike.Trike(rospy.get_param('trike'))  # Lower level class
        self.platform = rospy.get_param('platform')  # Embedded/stationary platform running the code
        self.paramserver = {}  # Interface with ROS parameters
        self.services = {'prov': {},'req': {}}  # Dict with all ROS services - provided/requested
        self.topics = {'pub': {},'sub': {}}  # Dict with all ROS topics - published/subscribed
        self.msgs = {}  # Dict with exchanged msgs
        self.time_start = None  # Instant when control is turned on
        self.time_elapsed = None  # Elapsed time since control was turned on
        # Perform initial build
        rospy.loginfo('Setting up messages and topics')
        self.build_msgs()
        self.set_topics()
        # Configure based on embedded/stationary platform
        rospy.loginfo('Adapting to platform')
        if self.platform == 'rasp':
            self.platform_rasp_adapt()
        elif self.platform == 'pc':
            self.platform_pc_adapt()

    def build_msgs(self):
        """Prepare and build msgs according to their ROS Msg type."""
        # Build stimulator msg
        stim_msg = Stimulator()
        stim_msg.channel = list(range(1,8+1))  # All the 8 channels
        stim_msg.mode = 8*['single']  # No doublets/triplets
        stim_msg.pulse_width = 8*[0]  # Initialize with zeros
        stim_msg.pulse_current = 8*[0]
        # Build intensity msg to publish instant stimulation current
        intensity_msg = Int32MultiArray()
        intensity_msg.data = 9*[0]  # [index] is the actual channel number
        # Assign designated internal variables
        self.msgs['stim'] = stim_msg
        self.msgs['status'] = String()
        self.msgs['angle'] = Float64()
        self.msgs['intensity'] = intensity_msg
        self.msgs['speed'] = Float64()
        self.msgs['cadence'] = Float64()
        self.msgs['distance'] = Float64()
        self.msgs['elapsed'] = Duration()

    def set_topics(self):
        """Declare the subscribed and published ROS Topics."""
        # List subscribed topics
        self.topics['sub']['pedal'] = rospy.Subscriber('imu/pedal', Imu, self.pedal_callback)
        # List published topics
        self.topics['pub']['stim'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
        self.topics['pub']['status'] = rospy.Publisher('trike/status', String, queue_size=10)
        self.topics['pub']['angle'] = rospy.Publisher('trike/angle', Float64, queue_size=10)
        self.topics['pub']['intensity'] = rospy.Publisher('trike/intensity', Int32MultiArray, queue_size=10)
        self.topics['pub']['speed'] = rospy.Publisher('trike/speed', Float64, queue_size=10)
        self.topics['pub']['cadence'] = rospy.Publisher('trike/cadence', Float64, queue_size=10)
        self.topics['pub']['distance'] = rospy.Publisher('trike/distance', Float64, queue_size=10)
        self.topics['pub']['elapsed'] = rospy.Publisher('trike/elapsed', Duration, queue_size=10)

    def platform_rasp_adapt(self):
        """Adjust and add what's specific of embedded platform."""
        # List provided services
        self.services['prov']['reboot'] = rospy.Service('trike/reboot',
            Empty, self.reboot_callback)
        self.services['prov']['kill_all'] = rospy.Service('trike/kill_all',
            Empty, self.kill_all_callback)
        self.services['prov']['set_status'] = rospy.Service('trike/set_status',
            SetUInt16, self.set_status_callback)
        self.services['prov']['set_pulse_width'] = rospy.Service('trike/set_pulse_width',
            SetUInt16, self.set_pulse_width_callback)
        self.services['prov']['set_init_intensity'] = rospy.Service('trike/set_init_intensity',
            SetUInt16, self.set_init_intensity_callback)
        self.services['prov']['change_intensity'] = rospy.Service('trike/change_intensity',
            SetBool, self.change_intensity_callback)
        # Apply default pulse width for all channels
        self.trike.update_stim_pw(rospy.get_param('trike/pulse_width'))

    def platform_pc_adapt(self):
        """Adjust and add what's specific of stationary platform."""
        # Communicate with the dynamic server
        self.paramserver = dynamic_reconfigure.client.Client('trike_config',
            config_callback=self.server_callback)  # 'server_node_name'
        # Update config parameters with default values
        config = self.paramserver.get_configuration()
        self.trike.update_config(config)
        # Update pulse width with default values
        pw_dict = {k[:3]:config[k] for k in config if k[4:] == 'pulse_width'}
        self.trike.update_stim_pw(pw_dict)
        # Update other terms
        self.trike.set_status('training')
        self.time_start = rospy.Time.now()

    def pedal_callback(self, data):
        """ROS Topic callback to process measurements from the pedal IMU.

        Attributes:
            data (Imu): ROS Msg from the pedal IMU sensor
        """
        # Get quaternion components
        qx = data.orientation.x
        qy = data.orientation.y
        qz = data.orientation.z
        qw = data.orientation.w
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
        # Store received data
        latest_time = data.header.stamp
        latest_angle = yaw
        latest_speed = data.angular_velocity.y*(180/pi)
        self.trike.update_measurements(latest_time, latest_angle, latest_speed)

    def server_callback(self, config):
        """ROS dynamic reconfigure callback to assign the modified server
        parameters to the equivalent variables.

        Attributes:
            config (dict): server dictionary with its parameters
        """
        # Focus on the changes and ignore the rest
        del config['groups']  # Remove unused sub dict
        prev = self.trike.config_dict
        try:
            modified = {k:config[k] for k in config if config[k] != prev[k]}
        except KeyError as e:
            return
        for param, update in modified.items():
            channel = ''
            self.trike.update_config(param, update)
            if param[4:] == 'current':
                channel = int(param[2])
                self.trike.update_stim_current(value=update, ch=channel)
            elif param[4:] == 'pulse_width':
                channel = int(param[2])
                self.trike.update_stim_pw(value=update, ch=channel)

    def refresh(self):
        """Update based on present state."""
        if self.trike.status == 'off':
            self.trike.check_new_cycle()
        else:
            self.time_elapsed = rospy.Time.now()-self.time_start
            if self.trike.status == 'racing':
                # Stop after 8 min or 1.2 km
                if (self.time_elapsed.to_sec() > 8.0*60) or (self.trike.distance > 1.2):
                    self.trike.set_status('off')
                    return
            flag = ''
            if self.time_elapsed.to_sec() <= 0:
                # Don't count distance yet
                flag = 'distance'
            self.trike.check_new_cycle(ignored=flag)
            self.trike.calculate()

    def update_msgs(self):
        """Modify ROS Msg variables according to their present value."""
        # Debug data
        cycles = self.trike.cycles
        stim_pw = self.trike.stim_pw
        stim_current = self.trike.stim_current
        msg = 'CYCLES:{}\nPW:{}\nCURRENT:{}\n'.format(cycles, stim_pw, stim_current)
        rospy.logdebug(msg)
        # Get data and setup variables
        _, angle, speed, _ = self.trike.get_latest_measurements()
        status = self.trike.status
        cadence = self.trike.cadence
        distance = self.trike.distance
        pw_dict = self.trike.stim_pw_now
        current_dict = self.trike.stim_current_now
        # Convert stimulation data from dict to list based on stim_order
        pw_list = 8*[0]
        current_list = 8*[0]
        for i, channel in enumerate(stim_order):
            pw_list[i] = pw_dict[channel]
            current_list[i] = current_dict[channel]
        # Update msgs
        self.msgs['stim'].pulse_width = pw_list
        self.msgs['stim'].pulse_current = current_list
        self.msgs['status'] = status
        self.msgs['angle'].data = angle
        self.msgs['intensity'].data = [0]+current_list
        self.msgs['speed'].data = speed
        self.msgs['cadence'] = cadence
        self.msgs['distance'] = distance
        self.msgs['elapsed'] = self.time_elapsed

    def publish_msgs(self):
        """Publish on all ROS Topics."""
        for name, tp in self.topics['pub'].items():
            tp.publish(self.msgs[name])

    def reboot_callback(self, data):
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

    def kill_all_callback(self, req):
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

    def set_status_callback(self, req):
        """ROS Service handler to change the cycling mode.

        Attributes:
            req (UInt16): 0 is off, 1 is training, 2 is racing
        """
        rospy.logdebug('Set status: service request')
        enum = ['off','training','racing']
        try:
            self.trike.set_status(enum[req.data])
        except IndexError as e:
            self.trike.set_status('off')
            rospy.logerr('Set status: failed, received %s', req.data)
            return {'success':False, 'message':self.trike.status}
        # Change time and current based on status
        stat = self.trike.status
        if stat == 'off':
            pass
        elif stat == 'training':
            self.time_elapsed = 0
            self.time_start = rospy.Time.now()
            self.trike.update_stim_current(value=rospy.get_param('trike/training_current'),
                proportion=rospy.get_param('trike/stim_proportion'))
        elif stat == 'racing':
            self.time_elapsed = 0
            self.time_start = rospy.Time.now()+rospy.Duration(30)
            self.trike.update_stim_current(value=rospy.get_param('trike/racing_current'),
                proportion=rospy.get_param('trike/stim_proportion'))
        return {'success':True, 'message':stat}

    def set_pulse_width_callback(self, req):
        """ROS Service handler to set the stimulation pulse width.

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
                file_path = rospack.get_path('ema_fes_cycling')+'/config/trike.yaml'
                # Change the config yaml file
                with open(file_path, 'r') as f:
                    file_handler = yaml.safe_load(f)
                    file_handler['pulse_width'] = req.data
                with open(file_path, 'w') as f:
                    yaml.safe_dump(file_handler, f)
                # Shutdown this node and rely on roslaunch respawn to restart
                msg = str(req.data)
                rospy.loginfo('Node shutdown: new pulse width')
                rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
                return {'success':True, 'message':msg}
        return {'success':False, 'message':msg}

    def set_init_intensity_callback(self, req):
        """ROS Service handler to set the initial stimulation intensity.

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
                file_path = rospack.get_path('ema_fes_cycling')+'/config/trike.yaml'
                # Change the config yaml file
                with open(file_path, 'r') as f:
                    file_handler = yaml.safe_load(f)
                    file_handler['training_current'] = req.data
                with open(file_path, 'w') as f:
                    yaml.safe_dump(file_handler, f)
                # Shutdown this node and rely on roslaunch respawn to restart
                msg = str(req.data)
                rospy.loginfo('Node shutdown: new initial intensity')
                rospy.Timer(rospy.Duration(1), rospy.signal_shutdown, oneshot=True)
                return {'success':True, 'message':msg}
        return {'success':False, 'message':msg}

    def change_intensity_callback(self, req):
        """ROS Service handler to request a change in intensity.

        Attributes:
            req (bool): 0 to decrease and 1 to increase
        """
        rospy.logdebug('Change intensity: service request')
        if req.data:  # Increase
            update = self.trike.stim_current_max+2
            self.trike.update_stim_current(value=update,
                proportion=rospy.get_param('trike/stim_proportion'))
        else:  # Decrease
            update = self.trike.stim_current_max-2
            self.trike.update_stim_current(value=update,
                proportion=rospy.get_param('trike/stim_proportion'))
        return {'success':True, 'message':str(self.trike.stim_current_max)}


def main():
    # Init control node
    rospy.loginfo('Initializing node')
    rospy.init_node('trike')
    # Create auxiliary class
    rospy.loginfo('Creating auxiliary class')
    aux = TrikeWrapper()
    # Define loop rate (in hz)
    rate = rospy.Rate(50)
    # Node loop
    while not rospy.is_shutdown():
        # New interaction
        aux.refresh()
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
