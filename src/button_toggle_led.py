#!/usr/bin/env python

import rospy
import roslib
import sys
import RPi.GPIO as GPIO

lit = False
def run():

        # Initialize to use BCM numbering.
	GPIO.setmode(GPIO.BCM)

        # We want pin 18 to be an output pin, and it should start out with a low value.
	GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)

        # We are going to take in the button press on pin 23.
	GPIO.setup(23, GPIO.IN)

	rospy.init_node("button_toggle_led", anonymous=False)

	# Now setup a callback for detecting a rising edge from the button.
	GPIO.add_event_detect(23, GPIO.RISING, callback=rising_edge_callback, bouncetime=600)
	rospy.spin()
	GPIO.cleanup()


def rising_edge_callback(channel):
	global lit

	lit = not lit

        # Toggle the state of the LED.
	if lit:
		GPIO.output(18, 1)
	else:
		GPIO.output(18, 0)
	

def main(args):
	run()


if __name__ == "__main__":
	main(sys.argv)
