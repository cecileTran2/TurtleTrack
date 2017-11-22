#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import geometry_msgs.msg
from turtlesim.msg import Pose
import sys
import time
import numpy

LINEAR_VELOCITY = 3
ANGULAR_VELOCITY = 1.5
LINEAR_ERROR = 0.03
ANGULAR_ERROR = 0.01


def move_from_to(x_init, y_init, theta_init, x_goal, y_goal, theta_goal):

    global  arrived_theta_1, arrived_theta, arrived_x, arrived_y, msg
    
    #Definition de l'angle d'orientation initial
    if (y_goal>y_init and x_goal>x_init):
        theta_objectif = numpy.arctan((y_init-y_goal)/((x_init - x_goal)))
        
    elif (y_goal<y_init and x_goal>x_init):
        theta_objectif = (2*numpy.pi) + numpy.arctan((y_init-y_goal)/((x_init - x_goal)))
        
    elif (y_goal>y_init and x_goal<x_init):
        theta_objectif = numpy.pi + numpy.arctan((y_init-y_goal)/((x_init - x_goal)))
        
    else:
        theta_objectif = numpy.pi + numpy.arctan((y_init-y_goal)/((x_init - x_goal)))

        
    # Orientation vers la destination
    if (abs(theta_objectif - theta_init) > ANGULAR_ERROR) and not arrived_theta_1:
        msg.angular.z = ANGULAR_VELOCITY
    else:
        if not arrived_theta_1:
            arrived_theta_1 = True

    if arrived_theta_1:
        arrived_theta = True
        print("arrived_theta!")

        
    # Deplacement vers la destination
    if abs(x_goal - x_init) > LINEAR_ERROR and arrived_theta:
        msg.linear.x = LINEAR_VELOCITY #* numpy.cos(theta_objectif)
        msg.angular.z = 0
    elif not arrived_theta:
        pass
    else:
        arrived_x = True
        #print("arrived_x! :D")

    if abs(y_goal - y_init) > LINEAR_ERROR and arrived_theta:
        #msg.linear.x = LINEAR_VELOCITY #* numpy.sin(theta_objectif)
        msg.angular.z = 0
    elif not arrived_theta:
        pass
    else:
        arrived_y = True

    # Debug print
    print('#'*20)

    print("abs(y_goal - y_init) : ", abs(y_goal - y_init))
        
    print("Theta goal", theta_goal)
    print("Theta init", theta_init)
    print("soust", abs(theta_goal - theta_init))

    print("Arrived x:", arrived_x)
    print("Arrived y:", arrived_y)
    print("Arrived theta:", arrived_theta)

    
def callback(data):
       
    global pub_cmd, x_goal, y_goal, theta_goal, arrived_theta_1, arrived_theta, arrived_x, arrived_y, msg

    x_init = data.x
    y_init = data.y
    theta_init = data.theta
    
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    
    move_from_to(x_init, y_init, theta_init, x_goal, y_goal, theta_goal)

    #if((arrived_x and not arrived_y) or (arrived_y and not arrived_x)):
     #   move_from_to(x_init, y_init, theta_init, x_goal, y_goal, theta_goal)
    
    # #Definition de l'angle d'orientation initial
    # if (y_goal>y_init and x_goal>x_init):
    #     theta_objectif = numpy.arctan((y_init-y_goal)/((x_init - x_goal)))
    # elif (y_goal<y_init and x_goal>x_init):
    #     theta_objectif = (2*numpy.pi) + numpy.arctan((y_init-y_goal)/((x_init - x_goal))) 
    # elif (y_goal>y_init and x_goal<x_init):
    #     theta_objectif = numpy.pi + numpy.arctan((y_init-y_goal)/((x_init - x_goal)))
    # else:
    #     theta_objectif = numpy.pi + numpy.arctan((y_init-y_goal)/((x_init - x_goal)))

        
    # # Orientation vers la destination
    # if (abs(theta_objectif - theta_init) > ANGULAR_ERROR) and not arrived_theta_1:
    #     msg.angular.z = ANGULAR_VELOCITY

    # else:
    #     if not arrived_theta_1:
    #         arrived_theta_1 = True
            
    # if arrived_theta_1:
    #     arrived_theta = True
    #     print("arrived_theta!")

        
    # # Deplacement vers la destination
    # if abs(x_goal - x_init) > LINEAR_ERROR and arrived_theta:
    #     msg.linear.x = LINEAR_VELOCITY #* numpy.cos(theta_objectif)
    #     msg.angular.z = 0
    # elif not arrived_theta:
    #     pass
    # else:
    #     arrived_x = True
    #     #print("arrived_x! :D")

    # if abs(y_goal - y_init) > LINEAR_ERROR and arrived_theta:
    #     msg.linear.x = LINEAR_VELOCITY #* numpy.sin(theta_objectif)
    #     msg.angular.z = 0
    # elif not arrived_theta:
    #     pass
    # else:
    #     arrived_y = True

    # # Debug print
    # print('#'*20)

    # print("abs(y_goal - y_init) : ", abs(y_goal - y_init))
        
    # print("Theta goal", theta_goal)
    # print("Theta init", theta_init)
    # print("soust", abs(theta_goal - theta_init))

    # print("Arrived x:", arrived_x)
    # print("Arrived y:", arrived_y)
    # print("Arrived theta:", arrived_theta)

    
    #Orientation finale
    if (abs(theta_goal - theta_init) > ANGULAR_ERROR) and arrived_x and arrived_y:
        msg.angular.z = ANGULAR_VELOCITY
        print("coucou")
        
    elif (abs(theta_goal - theta_init) < ANGULAR_ERROR) and arrived_theta:
        msg.angular.z = 0
        print("arrived_autre_theta!")
    else:
        pass
        
    pub_cmd.publish(msg)

    

if __name__ == '__main__':
    x_goal = float(sys.argv[1])
    y_goal = float(sys.argv[2])
    theta_goal = float(sys.argv[3])

    arrived_theta_1 = False
    arrived_theta = False
    arrived_x = False
    arrived_y = False

    msg = geometry_msgs.msg.Twist()

    rospy.init_node('nav_to_goal', anonymous=True)

    pub_cmd = rospy.Publisher('/turtle1/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, callback)

    rospy.spin()
