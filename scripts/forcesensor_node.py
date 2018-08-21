#!/usr/bin/env python

import rospy

# import ros msgs
import ema.modules.forcesensor as forcesensor
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int8

def main():
    # init forcesensor node
    rospy.init_node('forcesensor', anonymous=False)

    # get forcesensor config
    forcesensor_manager = forcesensor.ForceSensor(rospy.get_param('/ema_trike/forcesensor'))

    # list published topics
    pub = {}
    for name in forcesensor_manager.forcesensors:
        pub[name] = rospy.Publisher('forcesensor/' + name, WrenchStamped, queue_size=10)

    # define loop rate (in hz)
    rate = rospy.Rate(150)

    # node loop
    while not rospy.is_shutdown():

        try:
            timestamp = rospy.Time.now()
            frame_id = 'base_link'

            ## messages are shared by all force sensors?
            forceMsg = WrenchStamped()
            forceMsg.header.stamp = timestamp
            forceMsg.header.frame_id = frame_id
            
            for name in forcesensor_manager.forcesensors:
                force_vector = forcesensor_manager.getForce(name)

                forceMsg.wrench.force.x = force_vector[0]
                forceMsg.wrench.force.y = force_vector[1]
                forceMsg.wrench.force.z = force_vector[2]
                forceMsg.wrench.torque.x = 0
                forceMsg.wrench.torque.y = 0
                forceMsg.wrench.torque.z = 0
                
                pub[name].publish(forceMsg)
        
        except TypeError:
            print 'TypeError occured!'
        
        # sleep until it's time to work again
        rate.sleep()
        
    # cleanup
    forcesensor_manager.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
