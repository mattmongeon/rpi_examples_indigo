#!/usr/bin/env python

import rospy
import roslib
import sys
from std_msgs.msg import String

pub = rospy.Publisher("alive", String, queue_size=10)

def run():
	rospy.init_node("running_node", anonymous=True)

	rospy.Timer(rospy.Duration(0.1), callback)

	rospy.spin()

def callback(event):
	now = rospy.get_time()
	s = "I'm alive at time:  " + str(now)
	str_msg = String(s)
	
	pub.publish(str_msg)

def main(args):
	run()

if __name__ == '__main__':
	main(sys.argv)

