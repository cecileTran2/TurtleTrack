#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import geometry_msgs.msg
from turtlesim.msg import Pose
import sys
import time
import numpy

def callback(data):
       
    global pub_cmd, x_init, y_init, theta_init

    x_init = data.x
    y_init = data.y
    theta_init = data.theta

    msg = geometry_msgs.msg.Twist()

    # msg.linear.x = x_goal - x_init
    # msg.linear.y = y_goal - y_init
    # msg.angular.z = theta_goal - theta_init
    t1 = time.time()
    t2 = time.time()
    #while t2 - t1 < 5:

    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0

    global arrived_theta_1

    arrived_theta = False
    arrived_x = False
    arrived_y = False
    arrive_final = False


    #Definition de l'angle d'orientation initial
    if (y_goal>y_init and x_goal>x_init):
        theta_objectif = numpy.arctan((y_init-y_goal)/((x_init - x_goal)))
    elif (y_goal<y_init and x_goal>x_init):
        theta_objectif = (2*3.1415) + numpy.arctan((y_init-y_goal)/((x_init - x_goal)))
    elif (y_goal>y_init and x_goal<x_init):
        theta_objectif = 3.1415 + numpy.arctan((y_init-y_goal)/((x_init - x_goal)))
    else:
        theta_objectif = 3.1415 + numpy.arctan((y_init-y_goal)/((x_init - x_goal)))

    # Orientation vers la destination
    if (abs(theta_objectif - theta_init) > 0.01) and not arrived_theta_1:
        msg.angular.z = 0.5
    else:
        if not arrived_theta_1:
            arrived_theta_1 = True

    if arrived_theta_1:
        arrived_theta = True
        print("arrived_theta!")

    # Deplacement vers la destination
    if abs(x_goal - x_init) > 0.01 and arrived_theta:
        msg.linear.x = 1 #* numpy.cos(theta_objectif)
        msg.angular.z = 0
    elif not arrived_theta:
        pass
    else:
        arrived_x = True
        #print("arrived_x! :D")

    if abs(y_goal - y_init) > 0.03 and arrived_theta:
        # msg.linear.y = 1 * numpy.sin(theta_objectif)
        msg.angular.z = 0
    elif not arrived_theta:
        pass
    else:
        arrived_y = True


    print('#'*20)

    print("abs(y_goal - y_init) : ", abs(y_goal - y_init))
        
    print("Theta goal", theta_goal)
    print("Theta init", theta_init)
    print("soust", abs(theta_goal - theta_init))

    print("Arrived x:", arrived_x)
    print("Arrived y:", arrived_y)
    print("Arrived theta:", arrived_theta)


    #Orientation finale
    if (abs(theta_goal - theta_init) > 0.01) and arrived_x and arrived_y:
        msg.angular.z = 0.5
        print("coucou")
    elif (abs(theta_goal - theta_init) < 0.01) and arrived_theta:
        msg.angular.z = 0
        print("arrived_autre_theta!")
    else:
        pass
        
    pub_cmd.publish(msg)
    
# def nav_to_goal(x_goal,y_goal,theta_goal):
    #pub_left = rospy.Publisher('/pioneer_p3dx/leftWheelCommand',std_msgs.msg.Float64, queue_size=10)

   # pub_right = rospy.Publisher('/pioneer_p3dx/rightWheelCommand', std_msgs.msg.Float64, queue_size=10)

       
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    

if __name__ == '__main__':
    x_goal = float(sys.argv[1])
    y_goal = float(sys.argv[2])
    theta_goal = float(sys.argv[3])

    arrived_theta_1 = False

    rospy.init_node('nav_to_goal', anonymous=True)

    pub_cmd = rospy.Publisher('/turtle1/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, callback)

    rospy.spin()

    #while not rospy.is_shutdown():
    #    pass
