#!/usr/bin/env python

import rospy
import roslib
import sys
from std_msgs.msg import String
import RPi.GPIO as GPIO

lit = False
def gpio_start():

	GPIO.setmode(GPIO.BCM)
	GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(23, GPIO.IN)

	rospy.init_node("button_toggle_led", anonymous=False)

	# Now setup a callback for detecting a rising edge
	GPIO.add_event_detect(23, GPIO.RISING, callback=rising_edge_callback, bouncetime=400)
	rospy.spin()
	GPIO.cleanup()


def rising_edge_callback(channel):
	global lit

	lit = not lit

	if lit:
		GPIO.output(18, 1)
	else:
		GPIO.output(18, 0)
	

def main(args):
	gpio_start()


if __name__ == "__main__":
	main(sys.argv)
