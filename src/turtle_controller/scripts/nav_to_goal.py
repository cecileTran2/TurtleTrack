#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sys

def nav_to_goal(x_goal,y_goal,theta_goal):
    #pub_left = rospy.Publisher('/pioneer_p3dx/leftWheelCommand',std_msgs.msg.Float64, queue_size=10)

   # pub_right = rospy.Publisher('/pioneer_p3dx/rightWheelCommand', std_msgs.msg.Float64, queue_size=10)

    pub_cmd = rospy.Publisher('/turtle1/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

    def callback(data):
        
        x_init = data.x
        y_init = data.y
        theta_init = data.theta

        msg = geometry_msgs.msg.Twist()

        msg.linear.x = x_goal - x_init
        msg.linear.y = y_goal - y_init
        msg.angular.z = theta_goal - theta_init
        
        pub_cmd.publish(msg)
        
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
    rospy.Subscriber("/turtle1/pose", geometry_msgs.msg.Twist, callback)
    rospy.init_node('nav_to_goal', anonymous=True)
       
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    

if __name__ == '__main__':
    x_goal = sys.argv[1]
    y_goal = sys.argv[2]
    theta_goal = sys.argv[3]
    nav_to_goal(x_goal,y_goal,theta_goal)
    
    while not rospy.is_shutdown():
        pass
