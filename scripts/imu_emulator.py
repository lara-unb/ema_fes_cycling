#!/usr/bin/env python

import rospy
import rosbag
import rospkg
# import ros msgs
from sensor_msgs.msg import Imu

global file

file = "pedal15.bag"


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