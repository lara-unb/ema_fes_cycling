#!/usr/bin/env python

import rospy

# import ros msgs
import ema.modules.imu as imu
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8

def main():
    # init imu node
    rospy.init_node('imu', anonymous=False)

    # get imu config
    imu_manager = imu.IMU(rospy.get_param('/ema_trike/imu'))

    # list published topics
    pub = {}
    for name in imu_manager.imus:
        pub[name] = rospy.Publisher('imu/' + name, Imu, queue_size=10)
        pub[name + '_buttons'] = rospy.Publisher('imu/' + name + '_buttons', Int8, queue_size=10)

    # define loop rate (in hz)
    rate = rospy.Rate(200)

    # node loop
    while not rospy.is_shutdown():

        try:
            timestamp = rospy.Time.now()
            frame_id = 'base_link'

            if imu_manager.streaming == False:
                ## messages are shared by all imus
                imuMsg = Imu()
                imuMsg.header.stamp = timestamp
                imuMsg.header.frame_id = frame_id
                buttons = Int8()
                
                for name in imu_manager.imus:
                    orientation = imu_manager.getQuaternion(name)

                    imuMsg.orientation.x = orientation[0]
                    imuMsg.orientation.y = orientation[1]
                    imuMsg.orientation.z = orientation[2]
                    imuMsg.orientation.w = orientation[3]

                    angular_velocity = imu_manager.getGyroData(name)

                    imuMsg.angular_velocity.x = angular_velocity[0]
                    imuMsg.angular_velocity.y = angular_velocity[1]
                    imuMsg.angular_velocity.z = angular_velocity[2]

                    linear_acceleration = imu_manager.getAccelData(name)

                    imuMsg.linear_acceleration.x = -linear_acceleration[0]
                    imuMsg.linear_acceleration.y = -linear_acceleration[1]
                    imuMsg.linear_acceleration.z = -linear_acceleration[2]

                    pub[name].publish(imuMsg)
                    
                    buttons = imu_manager.getButtonState(name)
                    
                    pub[name + '_buttons'].publish(buttons)
            else:
                if imu_manager.broadcast == False:
                    for name in imu_manager.imus:
                        ## one message per imu
                        imuMsg = Imu()
                        imuMsg.header.stamp = timestamp
                        imuMsg.header.frame_id = frame_id
                        buttons = Int8()
                        
                        streaming_data = imu_manager.getStreamingData(name)
                        idx = 0
                        
                        for slot in imu_manager.streaming_slots[name]:
                            #print name, slot
                            
                            if slot == 'getTaredOrientationAsQuaternion':
                                                            
                                imuMsg.orientation.x = streaming_data[idx]
                                imuMsg.orientation.y = streaming_data[idx+1]
                                imuMsg.orientation.z = streaming_data[idx+2]
                                imuMsg.orientation.w = streaming_data[idx+3]
                                
                                idx = idx + 4
                                
                            elif slot == 'getNormalizedGyroRate':
                        
                                imuMsg.angular_velocity.x = streaming_data[idx]
                                imuMsg.angular_velocity.y = streaming_data[idx+1]
                                imuMsg.angular_velocity.z = streaming_data[idx+2]
                                
                                idx = idx + 3
                                
                            elif slot == 'getCorrectedAccelerometerVector':
                                
                                imuMsg.linear_acceleration.x = -streaming_data[idx]
                                imuMsg.linear_acceleration.y = -streaming_data[idx+1]
                                imuMsg.linear_acceleration.z = -streaming_data[idx+2]
                                
                                idx = idx + 3
                                
                                print type(streaming_data)
                                
                            elif slot == 'getButtonState':

                                if type(streaming_data) == 'tuple':
                                    buttons = streaming_data[idx]
                                
                                    idx = idx + 1
                                else:
                                    # imu is only streaming button state, result is not a tuple
                                    buttons = streaming_data

                        # publish streamed data
                        pub[name].publish(imuMsg)
                        pub[name + '_buttons'].publish(buttons)
                else:
                    for name in imu_manager.imus:
                        ## one message per imu
                        imuMsg = Imu()
                        imuMsg.header.stamp = timestamp
                        imuMsg.header.frame_id = frame_id
                        buttons = Int8()
                        
                        streaming_data = imu_manager.devices[name].getStreamingBatch()
                        
                        idx = 0
                        
                        imuMsg.orientation.x = streaming_data[idx]
                        imuMsg.orientation.y = streaming_data[idx+1]
                        imuMsg.orientation.z = streaming_data[idx+2]
                        imuMsg.orientation.w = streaming_data[idx+3]
                            
                        idx = idx + 4
                            
                        imuMsg.angular_velocity.x = streaming_data[idx]
                        imuMsg.angular_velocity.y = streaming_data[idx+1]
                        imuMsg.angular_velocity.z = streaming_data[idx+2]
                            
                        idx = idx + 3
                            
                        imuMsg.linear_acceleration.x = -streaming_data[idx]
                        imuMsg.linear_acceleration.y = -streaming_data[idx+1]
                        imuMsg.linear_acceleration.z = -streaming_data[idx+2]
                        
                        idx = idx + 3
                            
                        buttons = streaming_data[idx]

                        # publish streamed data
                        pub[name].publish(imuMsg)
                        pub[name + '_buttons'].publish(buttons)
        except TypeError:
            print 'TypeError occured!'

        # sleep until it's time to work again
        rate.sleep()
        
    # cleanup
    imu_manager.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
