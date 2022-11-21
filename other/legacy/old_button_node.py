#!/usr/bin/env python

import rospy

# import ros msgs
from std_msgs.msg import UInt8

# import utilities
import RPi.GPIO as GPIO

def main():
    # init button node
    rospy.init_node('button', anonymous=False)

    # init I/O config
    button1=11
    button2=13

    # set GPIO mode reference to board pin order
    GPIO.setmode(GPIO.BOARD)

    # turn on GPIO pull down onboard resistors 
    GPIO.setup(button1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(button2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # list published topics
    pub = rospy.Publisher('button/action', UInt8, queue_size=10)

    # define loop rate (in hz)
    rate = rospy.Rate(4)

    # node loop
    while not rospy.is_shutdown():

        # set button value to a byte 
        button_value = 0b00000000

        # apply OR operation on byte if button pressed
        if GPIO.input(button1):
            button_value=button_value|0b00000001

        if GPIO.input(button2):
            button_value=button_value|0b00000010

        # at least one button pressed
        if button_value:
            # send button update
            pub.publish(UInt8(button_value)) # 1, 2 or 3

        # wait for next loop
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
