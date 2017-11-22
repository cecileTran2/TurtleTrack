#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import geometry_msgs.msg
from turtlesim.msg import Pose
import sys
import time
import numpy
import math
import sys

def message():

	global t_ini

	msg = geometry_msgs.msg.Twist()

	if time.time()-t_ini<5:
		msg.linear.x = 0.5
	else:
		msg.linear.x = 0.0

	return msg


if __name__ == '__main__':

    #linear_velocity = float(sys.argv[1])

	t_ini = time.time()

	rospy.init_node('test_nav', anonymous=True)
	pub_cmd = rospy.Publisher('/teleop_velocity_smoother/raw_cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
	#pub_cmd = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		msg = message()
		pub_cmd.publish(msg)
		r.sleep()

	print("coucou")

   

    #rospy.Subscriber("/turtle1/pose", Pose, callback)

 

rospy.spin()    # keeps your node from exiting until the node has been shutdown.
