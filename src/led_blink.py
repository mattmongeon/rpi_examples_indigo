#!/usr/bin/env python

import rospy
import roslib
import sys
import RPi.GPIO as GPIO


lit = False

def run():

	GPIO.setmode(GPIO.BCM)
	GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)

	rospy.init_node("led_blink", anonymous=False)
	rospy.Timer(rospy.Duration(1.0), timer_callback)
        rospy.spin()
	GPIO.cleanup()
	
	
def timer_callback(event):
	global lit

        # Toggle the light state.
	lit = not lit
	if lit:
		GPIO.output(18, 1)
	else:
		GPIO.output(18, 0)


def main(args):
	run()


if __name__ == "__main__":
	main(sys.argv)
