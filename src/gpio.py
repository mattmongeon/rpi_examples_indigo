#!/usr/bin/env python

import rospy
import roslib
import sys
from std_msgs.msg import String
import RPi.GPIO as GPIO


#try:
#        import RPi.GPIO as GPIO
#except RuntimeError:
#                print{"Not Working"}


lit = False
def gpio_start():

	global lit	
	
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(23, GPIO.IN)

	rospy.init_node("gpio", anonymous=False)

	rospy.Timer(rospy.Duration(1.0), timer_callback)

	# Now setup a callback for detecting a rising edge
	#GPIO.add_event_detect(23, GPIO.RISING, callback=rising_edge_callback, bouncetime=200)
	#rospy.spin()
	r = rospy.Rate(50) # 50 Hz
	while not rospy.is_shutdown():
		
		if GPIO.input(23):
			lit = not lit
			
			GPIO.output(18, lit)

		r.sleep()
	
	
def timer_callback(event):
	global lit

	lit = not lit
	if lit:
		GPIO.output(18, 1)
	else:
		GPIO.output(18, 0)


def rising_edge_callback(channel):
	print "Rising edge on channel " + str(channel)


def falling_edge_callback(channel):
	print "Falling edge on channel " + str(channel)	


def main(args):
	gpio_start()


if __name__ == "__main__":
	main(sys.argv)
