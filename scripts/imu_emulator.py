#!/usr/bin/env python3

"""

Particularly, this code takes recorded data from a bag file and publishes
the pedal angle as a ROS message.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

"""

# Python 2 and 3 compatibility
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from builtins import *

import rospy
import rosbag
import rospkg

# Import ROS msgs:
from sensor_msgs.msg import Imu

# Global variables:
global file

file = "Recorded.bag"


def main():
    # Init button node:
    rospy.init_node('imu_emulator', anonymous=False)

    # List published topics:
    pub = rospy.Publisher('imu/pedal', Imu, queue_size=10)

    # Define loop rate (in hz):
    rate = rospy.Rate(164)  # Equivalent to the data sample rate

    rospack = rospkg.RosPack()
    path = rospack.get_path("ema_fes_cycling")+"/resources/"+file
    bag = rosbag.Bag(path)
    messages = {}

    for topic, msg, timestamp in bag.read_messages(topics=['/ema/imu/pedal']):
        if topic not in messages:
            messages[topic] = [msg]
        else:
            messages[topic].append(msg)

    msg = list(messages['/ema/imu/pedal'])

    # Node loop:
    while not rospy.is_shutdown():
        if len(msg) < 1:
            msg = list(messages['/ema/imu/pedal'])

        pub.publish(msg.pop(0))

        # Wait for next loop:
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
