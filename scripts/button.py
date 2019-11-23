#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int8

def button():
    button1=11
    button2=12

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(button1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(button2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    pub = rospy.Publisher('imu/remote_buttons', Int8, queue_size=10)
    rospy.init_node('button', anonymous=True)
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        button_value = 0b00000000
        if GPIO.input(button1):
            button_value=button_value|0b00000001
        if GPIO.input(button2):
            button_value=button_value|0b00000010
        pub.publish(Int8(button_value))
        rate.sleep()
if __name__ == '__main__':
    try:
        button()
    except rospy.ROSInterruptException:
        pass

