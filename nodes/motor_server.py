#!/usr/bin/env python
#testing custom roboclaw ROS node
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist

DEFAULT_TWIST_CMD_TOPIC = "~twist_command"
#Windows comport name
#rc = Roboclaw("COM9",115200)
#Linux comport name

def handle_add_two_inits(req):
	print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
	return AddTwoIntsResponse(req.a + req.b)

def cmd_vel_callback(twist):
	# some code to implement a life span of twist a command
	# dont pass twist message that are too old (> 1 sec) or DEADMAN seconds paramter
	# essentially twist messages must expire
	return

def add_two_ints_server():
	rospy.init_node('add_two_ints_server')
	#s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
	cmd_sub = rospy.Subscriber('ds4/cmd_vel', Twist, cmd_vel_callback, queue_size = 1)
	print "Ready to add two ints."
	rospy.spin()

if __name__ == "__main__":
	print "running add_two_ints_server"
	add_two_ints_server()
