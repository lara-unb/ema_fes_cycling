#!/usr/bin/env python3

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
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from builtins import *

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
from tf.transformations import euler_from_quaternion
from math import pi
import os
import yaml
import rospkg
import rosnode

# Other imports
import dynamic_reconfigure.client


class TrikeWrapper(object):
    """A class used to wrap the trike functionalities and establish a ROS
    interface for its module.

    Attributes:
        self.trike (object): lower level class
        self.platform (string): embedded/stationary platform running the code
        self.matrix_on (int): loop rate in hz if using matrix
        self.paramserver (object): interface with ROS parameters
        self.services (dict): ROS services - provided/requested
        self.topics (dict): ROS topics - published/subscribed
        self.msgs (dict): exchanged msgs
        self.time_start (object): instant when control is turned on
        self.time_elapsed (object): elapsed time since control was turned on
    """
    def __init__(self):
        rospy.loginfo('Initializing trike')
        self.trike = trike.Trike(rospy.get_param('trike'))
        self.platform = rospy.get_param('platform')
        self.matrix_on = rospy.get_param('stimulator/matrix', False)
        self.paramserver = {}
        self.services = {'prov': {},'req': {}}
        self.topics = {'pub': {},'sub': {}}
        self.msgs = {}
        self.time_start = None
        self.time_elapsed = None
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
        if not self.matrix_on:  # When using conventional electrode
            # Build stimulator msg
            stim_msg = Stimulator()
            stim_msg.channel = list(range(1,8+1))  # All the 8 channels
            stim_msg.mode = 8*['single']  # No doublets/triplets
            stim_msg.pulse_current = 8*[0]
            stim_msg.pulse_width = 8*[0]  # Initialize with zeros
            self.msgs['stim'] = stim_msg
        # Build intensity msg to publish instant stimulation signal
        current_msg = Int32MultiArray()
        current_msg.data = 9*[0]  # [index] is the actual channel number
        pw_msg = Int32MultiArray()
        pw_msg.data = 9*[0]  # [index] is the actual channel number
        # Assign designated internal variables
        self.msgs['status'] = String()
        self.msgs['angle'] = Float64()
        self.msgs['current'] = current_msg
        self.msgs['pw'] = pw_msg
        self.msgs['speed'] = Float64()
        self.msgs['cadence'] = Float64()
        self.msgs['distance'] = Float64()
        self.msgs['elapsed'] = Duration()

    def set_topics(self):
        """Declare the subscribed and published ROS Topics."""
        # List subscribed topics
        self.topics['sub']['pedal'] = rospy.Subscriber('imu/pedal', Imu, self.pedal_callback)
        # List published topics
        if not self.matrix_on:  # When using conventional electrode
            self.topics['pub']['stim'] = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
        self.topics['pub']['status'] = rospy.Publisher('trike/status', String, queue_size=10)
        self.topics['pub']['angle'] = rospy.Publisher('trike/angle', Float64, queue_size=10)
        self.topics['pub']['current'] = rospy.Publisher('stimulator/current', Int32MultiArray, queue_size=10)
        self.topics['pub']['pw'] = rospy.Publisher('stimulator/pulse_width', Int32MultiArray, queue_size=10)
        self.topics['pub']['speed'] = rospy.Publisher('trike/speed', Float64, queue_size=10)
        self.topics['pub']['cadence'] = rospy.Publisher('trike/cadence', Float64, queue_size=10)
        self.topics['pub']['distance'] = rospy.Publisher('trike/distance', Float64, queue_size=10)
        self.topics['pub']['elapsed'] = rospy.Publisher('trike/elapsed', Duration, queue_size=10)

    def platform_rasp_adapt(self):
        """Adjust and add what's specific of embedded platform."""
        # List provided services
        self.services['prov']['reboot'] = rospy.Service('trike/reboot',
            Empty, self.reboot_callback)
        self.services['prov']['shutdown'] = rospy.Service('trike/shutdown',
            Empty, self.shutdown_callback)
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
        self.services['prov']['auto_pulse_width'] = rospy.Service('trike/auto_pulse_width',
            SetBool, self.auto_pulse_width_callback) # not used yet

    def platform_pc_adapt(self):
        """Adjust and add what's specific of stationary platform."""
        # Communicate with the dynamic server
        self.paramserver = dynamic_reconfigure.client.Client('trike_config',
            config_callback=self.server_callback)  # 'server_node_name'
        # Update config parameters with default values
        config = self.paramserver.get_configuration()
        self.trike.set_config(config)
        # Update pulse width with default values
        pw_dict = {k[:3]:config[k] for k in config if k[4:] == 'pulse_width'}
        self.trike.set_stim_pw(pw_dict)
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
        euler = euler_from_quaternion([qx, qy, qz, qw], axes='rzxy')
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
        self.trike.set_latest_measurements(latest_time, latest_angle, latest_speed)

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
            self.trike.set_config(param, update)
            if param[4:] == 'current':
                channel = int(param[2])
                self.trike.set_stim_current(value=update, ch=channel)
            elif param[4:] == 'pulse_width':
                channel = int(param[2])
                self.trike.set_stim_pw(value=update, ch=channel)
            elif param == 'autoPW_on':
                if update:  # Automatic pulse width sequence enabled
                    self.trike.set_status('autopw-PC') # PC autopw status
                    self.time_elapsed = 0
                    self.time_start = rospy.Time.now()
                else:
                    # Update the pulse width on the server
                    for k, v in self.trike.stim_pw.items():
                        self.paramserver.update_configuration({k+'_pulse_width':v})
                    # Back to training status keeping the distance stored
                    self.trike.set_status('training', zero_distance=False)

    def refresh(self):
        """Update based on present state."""
        if self.trike.status == 'off':
            self.trike.check_new_cycle()
        else:
            # if self.time_elapsed == 0:
            #     rospy.loginfo('No callback delay - set_status_callback started at the right time')
            if (self.time_elapsed or self.time_start) == None:
                rospy.logwarn('Callback delay detected - set_status_callback - Variables time_start and time_elapsed = NoneType')
                rospy.loginfo('Fixing delay - setting time_elapsed = 0 and time_start = rospy.Time.now()')
                self.time_elapsed = 0
                self.time_elapsed = rospy.Time.now()
            self.time_elapsed = rospy.Time.now()-self.time_start
            if self.trike.status == 'racing':
                # Stop after 8 min or 1.2 km
                if (self.time_elapsed.to_sec() > 8.0*60) or (self.trike.distance > 1.2):
                    self.trike.set_status('off')
                    return
            elif 'autopw' in self.trike.status:
                # Adapt autopw sequence to rasp and pc configs
                if self.platform == 'rasp':
                    # Automatic pulse width sequence with proportions on rasp
                    self.trike.update_autopw_sequence(self.time_elapsed.to_sec(), 
                        proportion=rospy.get_param('trike/pw_proportion'))
                elif self.platform == 'pc':
                    # Automatic pulse width sequence on pc
                    self.trike.update_autopw_sequence(self.time_elapsed.to_sec())
            flag = ''
            if self.time_elapsed.to_sec() <= 0:
                rospy.loginfo('Set Status Function Initialized at Correct Time: '+str(self.trike.status))
                # Don't count distance yet
                flag = 'distance'
            self.trike.check_new_cycle(ignored=flag)
            self.trike.update_stim_output()

    def update_msgs(self):
        """Modify ROS Msg variables according to their present value."""
        # Debug data
        cycles = self.trike.cycles
        stim_current = self.trike.stim_current
        stim_pw = self.trike.stim_pw
        msg = 'CYCLES:{}\nCURRENT:{}\nPW:{}\n'.format(cycles, stim_current, stim_pw)
        rospy.logdebug(msg)
        # Get data and setup variables
        _, angle, speed, _ = self.trike.get_latest_measurements()
        status = self.trike.status
        cadence = self.trike.cadence
        distance = self.trike.distance
        current_list, pw_list = self.trike.get_stim_list()
        # Update msgs
        if not self.matrix_on:  # When using conventional electrode
            self.msgs['stim'].pulse_current = current_list
            self.msgs['stim'].pulse_width = pw_list
        self.msgs['status'] = status
        self.msgs['angle'].data = angle
        self.msgs['current'].data = [0]+current_list
        self.msgs['pw'].data = [0]+pw_list
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

    def shutdown_callback(self, data):
        """ROS Service handler to shutdown the machine.

        Attributes:
            req (Empty): empty input
        """
        rospy.loginfo('Shutdown: service request')
        # Attempt to shutdown all nodes except this
        nodes = rosnode.get_node_names('ema')
        nodes.remove(rospy.get_name())
        success_list, fail_list = rosnode.kill_nodes(nodes)
        if fail_list:
            rospy.logerr('Shutdown: failed on %s shutdown', fail_list)
        rospy.loginfo('Shutting down machine...')
        os.system('sudo halt')
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
            req (UInt16): 0 is off, 1 is training, 2 is racing and 3,4,5,6,7 are automated
        """
        rospy.logdebug('Set status: service request')
        enum = ['off','training','racing','autopw-PC','autopw-CC','autopw-CM','autopw-TC','autopw-TM']
        try:
            # Using proportion to check activated stim channels to all automated status
            self.trike.set_status(enum[req.data], proportion=rospy.get_param('trike/stim_proportion'))
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
            # Apply default current with proportions for all channels
            self.trike.set_stim_current(value=rospy.get_param('trike/training_current'),
                proportion=rospy.get_param('trike/stim_proportion'))
            # Apply default pulse width for all channels
            self.trike.set_stim_pw(rospy.get_param('trike/pulse_width'), 
                proportion=rospy.get_param('trike/pw_proportion'))
        elif stat == 'racing':
            self.time_elapsed = 0
            self.time_start = rospy.Time.now()+rospy.Duration(30)
            # Apply default current with proportions for all channels
            self.trike.set_stim_current(value=rospy.get_param('trike/racing_current'),
                proportion=rospy.get_param('trike/stim_proportion'))
            # Apply default pulse width for all channels
            self.trike.set_stim_pw(rospy.get_param('trike/pulse_width'), 
                proportion=rospy.get_param('trike/pw_proportion'))
        elif 'autopw' in stat:
            self.time_elapsed = 0
            self.time_start = rospy.Time.now()
            # rospy.loginfo('set_status_callback: '+str(stat))
            # rospy.loginfo('Time start before: '+str(self.time_start))
            # Apply default current with proportions for all channels
            self.trike.set_stim_current(value=rospy.get_param('trike/autoPW_current'),
                proportion=rospy.get_param('trike/stim_proportion'))
            # Apply initial default pulse width with proportions for all channels
            self.trike.set_stim_pw(rospy.get_param('trike/autoPW_init'), 
                proportion=rospy.get_param('trike/pw_proportion'))
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
            self.trike.set_stim_current(value=update,
                proportion=rospy.get_param('trike/stim_proportion'))
        else:  # Decrease
            update = self.trike.stim_current_max-2
            self.trike.set_stim_current(value=update,
                proportion=rospy.get_param('trike/stim_proportion'))
        return {'success':True, 'message':str(self.trike.stim_current_max)}

    def auto_pulse_width_callback(self, req):
        """Call a ROS Service to request new pulse width value (automated cycling)

        Attributes:
            req (int): updated pulse width
        """
        rospy.logdebug('Auto Pulse Width: service request')
        current_list, pw_list = self.trike.get_stim_list()
        return {'success':True, 'message':str(self.trike.stim_pw_max)}


def main():
    # Init control node
    rospy.loginfo('Initializing node')
    rospy.init_node('trike')
    # Create auxiliary class
    rospy.loginfo('Creating auxiliary class')
    aux = TrikeWrapper()
    # Define loop rate (in hz)
    rate = rospy.Rate(48)
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
