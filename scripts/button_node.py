#!/usr/bin/env python3

"""

Particularly, this code initializes the button interface, check for interaction
with polling and asynchronously publishes a ROS message.

The ROS node runs this code. It should make all the necessary
communication/interaction with ROS and it shouldn't deal with minor details.
For example, it would be used to publish a filtered sensor measurement as
a ROS message to other ROS nodes instead of establishing the serial comm
and treating that raw measurement. For more info, check:
http://wiki.ros.org/Nodes

"""

import rospy

# Import ROS msgs:
from std_msgs.msg import UInt8

# Import utilities:
import RPi.GPIO as GPIO


def main():
    # Init button node:
    rospy.init_node('button', anonymous=False)

    # Init I/O config:
    button1 = 11
    button2 = 13

    # Set GPIO mode reference to board pin order:
    GPIO.setmode(GPIO.BOARD)

    # Turn on GPIO pull down onboard resistors:
    GPIO.setup(button1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(button2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # List published topics:
    pub = rospy.Publisher('button/action', UInt8, queue_size=10)

    # Define loop rate (in hz):
    rate = rospy.Rate(4)

    # Node loop:
    while not rospy.is_shutdown():
        # Set button value to a byte:
        button_value = 0b00000000

        # Apply OR operation on byte if button is pressed.
        if GPIO.input(button1):
            button_value = button_value | 0b00000001
        if GPIO.input(button2):
            button_value = button_value | 0b00000010

        # At least one button was pressed.
        if button_value:
            # Send button update:
            pub.publish(UInt8(button_value))  # 1, 2 or 3

        # Wait for next loop:
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
