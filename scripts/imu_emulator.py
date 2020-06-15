#!/usr/bin/env python

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

import rospy
import rosbag
import rospkg

# Import ROS msgs
from sensor_msgs.msg import Imu

# Global variables
global file

file = "Recorded.bag"


def main():
    # init button node
    rospy.init_node('imu_emulator', anonymous=False)
    
    # list published topics
    pub = rospy.Publisher('imu/pedal', Imu, queue_size=10)

    # define loop rate (in hz) 'Define data sample rate '
    rate = rospy.Rate(164)

    rospack = rospkg.RosPack()
    path = rospack.get_path("ema_fes_cycling")+"/resources/"+ file
    
    bag = rosbag.Bag(path)
    messages = {}

    for topic, msg, timestamp in bag.read_messages(topics=['/ema/imu/pedal']):
        if topic not in messages:
            messages[topic] = [msg]
        else:
            messages[topic].append(msg)
    
    msg = list(messages['/ema/imu/pedal'])
    # node loop
    while not rospy.is_shutdown():
        if len(msg) < 1:   
            msg = list(messages['/ema/imu/pedal'])
        
        pub.publish(msg.pop(0)) 
        
        # wait for next loop
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass