#!/usr/bin/env python

import rospy
import roslib
import sys
import RPi.GPIO as GPIO

# Initialize the LED to not be lit.
lit = False

def run():

        # Initialize to use BCM numbering.
	GPIO.setmode(GPIO.BCM)

        # We want pin 18 to be an output pin, and it should start out with a low value.
	GPIO.setup(24, GPIO.OUT, initial=GPIO.LOW)

	rospy.init_node("led_blink", anonymous=False)

        # Set up a timer to toggle the LED state once per second.
	rospy.Timer(rospy.Duration(1.0), timer_callback)
        rospy.spin()
	GPIO.cleanup()
	
	
def timer_callback(event):
	global lit

        # Toggle the LED state.
	lit = not lit
	if lit:
		GPIO.output(24, 1)
	else:
		GPIO.output(24, 0)


def main(args):
	run()


if __name__ == "__main__":
	main(sys.argv)
