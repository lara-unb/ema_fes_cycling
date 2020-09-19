#!/usr/bin/env python

import rospy
import time
# import ros msgs
from std_msgs.msg import UInt8

# import utilities
import RPi.GPIO as GPIO

global pressed_time
global current_button_value

def button1_callback(channel):
    global pressed_time
    global current_button_value
    current = time.time()
    if current_button_value == 0:
        current_button_value = 1

    elif current_button_value == 1:
        current_button_value = 4 

    elif current_button_value == 2:       
        current_button_value = 3
    pressed_time = current

def button2_callback(channel):
    global pressed_time
    global current_button_value
    current = time.time()
    if current_button_value == 0:
        current_button_value = 2

    elif current_button_value == 2:
        current_button_value = 5 

    elif current_button_value == 1:       
        current_button_value = 3
    pressed_time = current

def main():
    global current_button_value
    global pressed_time
    # init button node
    rospy.init_node('button', anonymous=False)

    # init I/O config
    button1=11
    button2=13
    bouncetime = 20
    # set GPIO mode reference to board pin order
    GPIO.setmode(GPIO.BOARD)

    # turn on GPIO pull down onboard resistors 
    GPIO.setup(button1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(button2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    #Set on interrupt event in button ports
    GPIO.add_event_detect(button1, GPIO.RISING, callback=button1_callback, bouncetime=bouncetime)
    GPIO.add_event_detect(button2, GPIO.RISING, callback=button2_callback, bouncetime=bouncetime)
    # list published topics
    pub = rospy.Publisher('button/action', UInt8, queue_size=10)

    # define loop rate (in hz)
    rate = rospy.Rate(20)

    #Time to wait to publish when a button is pressed (Miliseconds)

    wait_time = 200 
    # node loop
    while not rospy.is_shutdown():

        if current_button_value > 0:
            if (time.time()-pressed_time) * 1000 > wait_time:     
                pub.publish(UInt8(current_value))
                current_value = 0
        rate.sleep()
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
