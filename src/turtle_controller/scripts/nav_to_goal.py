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
import nav_msgs.msg


LINEAR_ERROR = 0.3
ANGULAR_ERROR = 0.02


ANGULAR_VELOCITY = 0.3
LINEAR_VELOCITY = 0.2


coef_prop = 3.46

def equal_signe(x,y):
    if x>=0 and y>=0 or x<=0 and y<=0:
        return True
    else:
        return False

def quater2yaw(q):

    #il lui faut l'objet pose.pose.orientation

    yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    pitch = math.asin(-2.0*(q.x*q.z - q.w*q.y))
    roll = math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)

    return yaw, pitch, roll

def vitesse_rotation(theta_carrelage,
                     theta_obj,
                     vitesse_angulaire = 1.0,
                     linear_error = None):
    global msg

    if linear_error is None:
        V = vitesse_angulaire
    else:
        V = 10 * linear_error

    if theta_obj<(theta_carrelage+math.pi) and theta_obj > theta_carrelage:
        return V
    else:
        return -V


def normalise_angle(angle):

    # met l'angle dans [0, 2 pi]

    #return angle 

    if angle >= 0.0 and angle <= 2*numpy.pi:
        return angle
    elif angle > 2*numpy.pi:
        output_angle = angle
        while output_angle > 2*numpy.pi:
            output_angle -= 2*numpy.pi
        return output_angle
    else:
        output_angle = angle
        while output_angle < 0:
            output_angle += 2 * numpy.pi
        return output_angle


def get_theta_obj(x_i, x_f, y_i, y_f, marge_erreur = 0.0):


    # rend theta_obj, le robot doit etre dans l'angle theta_obj pour se diriger en ligne droite vers sa cible

    if x_i == x_f:
        return math.pi/2 #sinon division par 0 si xi = xf

    tan_theta_obj = (y_i - y_f)/(x_i - x_f)

    if y_f > y_i and x_f > x_i:
        theta_obj = numpy.arctan(tan_theta_obj)
    elif y_f < y_i and x_f > x_i:
        theta_obj = (2*math.pi) + numpy.arctan(tan_theta_obj)
    elif y_f > y_i and x_f < x_i:
        theta_obj = math.pi + numpy.arctan(tan_theta_obj)
    else:
        theta_obj = math.pi + numpy.arctan(tan_theta_obj)

    return theta_obj


def get_angular_error(x_init, x_goal, y_init, y_goal):
    theta_obj = get_theta_obj(x_init, x_goal, y_init, y_goal)
    theta_obj_error = get_theta_obj(x_init, x_goal, y_init, y_goal)
    return abs(theta_obj_error-theta_obj)


def get_linear_error(x_i, x_f, y_i, y_f,
                     angular_error):

    # donne l'erreur de position permise

    distance_to_go = math.sqrt((x_f-x_i)**2+(y_f-y_i)**2)

    return distance_to_go * math.tan(angular_error)/math.sqrt(2)


def rotate_franklin(theta_carrelage,
                    angular_velocity,
                    angular_error,
                    theta_obj):
    global pub_cmd, msg

    # effectue une rotation tant qu'il n'a pas atteint l'angle voulu
    # TO DO: metre une boucle de retour s'il depasse l'angle par accident (avec une vitesse negative de module plus
    # faible pour qu'il fasse la rotation dans l'autre sens avec plus de precision

    # debug_print:
    # print (20*'#')
    #
    # print "erreur angulaire: ", abs(angle_final - angle_initial)
    # print "angle final", angle_final
    # print "angle initial", angle_initial
    # print "seuil", angular_error
    # print "doit tourner", abs(angle_final - angle_initial) > angular_error
    #
    # print (20 * '#')


    if abs(normalise_angle(theta_carrelage) -normalise_angle(theta_obj)) > angular_error:
        msg.angular.z = angular_velocity
    else:
        msg.angular.z = 0.0


def walk_franklin(x_i, y_i, x_f, y_f, linear_velocity, linear_error):

    # avance vers la position voulue tant qu'il ne l'a pas atteinte
    # TO DO: mettre une boucle de retour s'il depasse la position par accident

    global pub_cmd, msg

    arrived = (abs(x_f - x_i) < linear_error and abs(y_f - y_i) < linear_error)
    if not arrived:
        msg.linear.x = linear_velocity
    else:
        msg.linear.x = 0.0


def odom_callback(data):

    global pub_cmd, msg, arrived_position, angular_velocity, arrived_theta, linear_error,\
    x_odom_init, y_odom_init, theta_odom_init, current_error_on_x, current_error_on_y


     #on recupere les positions courrantes du robot:
    x_odom = data.pose.pose.position.x
    y_odom = data.pose.pose.position.y

    yaw, pitch, roll = quater2yaw(data.pose.pose.orientation)
    print "yaw pitch roll: ", yaw, pitch, roll
    theta_odom = normalise_angle(roll)

    #on recupere le repere odom initial
    if x_odom_init is None:
        x_odom_init, y_odom_init, theta_odom_init = x_odom, y_odom, theta_odom
        # print('x_robot_init : ', x_robot_init)
        # print('y_robot_init : ', y_robot_init)
        print('theta_robot_init : ', theta_robot_init)

    #on a les coordonnes du but dans le repere du carrelage, on veut les mettre dans le repere odom init:
    x_goal_odom_init = x_goal_carrelage/coef_prop - x_odom_init
    y_goal_odom_init = y_goal_carrelage/coef_prop - y_odom_init
    theta_goal_odom_init = theta_goal_carrelage - theta_odom_init

    

    # print('x_goal : ', x_goal)
    # print('y_goal : ', y_goal)
    # print('theta_goal : ', theta_goal)

    #print('data.pose.pose.orientation : ', data.pose.pose.orientation)
    

    # print('x_robot : ', x_robot)
    # print('y_robot : ', y_robot)
    # print('theta_robot : ', theta_robot)
    # print('theta_robot_init : ', theta_robot_init)

    #on recupere la position courante du robot dans le repere carrelage:

    x_current_odom_init = x_odom - x_odom_init # on commence par les exprimer dans le repere odom_init
    y_current_odom_init = y_odom - y_odom_init
    theta_current_odom_init = theta_odom - theta_odom_init

    x_carrelage = x_current_odom_init*coef_prop + x_carrelage_init 
    y_carrelage = y_current_odom_init*coef_prop + y_carrelage_init
    theta_carrelage_init= 0.0   #on suppose ici que la tortue est placee initialement avec un angle theta = 0.0 :D
    theta_carrelage = theta_current_odom_init + theta_carrelage_init

    msg = geometry_msgs.msg.Twist()

    if not arrived_position:

        # si la tortue n'a pas encore atteint sa position spatiale, sa position angulaire voulue est l'orientation
        # vers la position finale

        theta_obj = get_theta_obj(x_i=x_carrelage_init, x_f=x_goal_carrelage, y_i=y_carrelage_init, y_f=y_goal_carrelage)
        print('theta_obj : ', theta_obj)

    else:

        # sinon c'est l'angle final voulu

        theta_obj = theta_goal_carrelage
        print('theta_obj : ', theta_obj)

    if not arrived_theta and not arrived_position:

        angular_velocity = vitesse_rotation(theta_carrelage=theta_carrelage,
                                            theta_obj=theta_obj,
                                            vitesse_angulaire=ANGULAR_VELOCITY)

        rotate_franklin(theta_carrelage=theta_carrelage,
                        theta_obj=theta_obj,
                        angular_velocity=angular_velocity,
                        angular_error=ANGULAR_ERROR)

        arrived_theta = abs(normalise_angle(theta_obj) - normalise_angle(theta_carrelage)) < ANGULAR_ERROR

        #real_angular_error = abs(normalise_angle(theta_obj) - normalise_angle(theta_carrelage))
        linear_error = get_linear_error(x_i=x_carrelage, y_i=y_carrelage, x_f=x_goal_carrelage, y_f=y_goal_carrelage,
                                        angular_error=ANGULAR_ERROR)


    elif arrived_theta and not arrived_position:

        walk_franklin(x_i=x_carrelage, y_i=y_carrelage, x_f=x_goal_carrelage, y_f=y_goal_carrelage,
                      linear_velocity=LINEAR_VELOCITY,
                      linear_error=linear_error)

        distance_to_go = math.sqrt((x_carrelage-x_goal_carrelage)**2+(y_carrelage-y_goal_carrelage)**2)

        #current_angular_error_permitted = numpy.arctan(LINEAR_ERROR/distance_to_go)
        #error_angle = abs(theta_obj - theta_carrelage)
        arrived_position = (abs(x_goal_carrelage - x_carrelage) < LINEAR_ERROR and abs(y_goal_carrelage - y_carrelage) < LINEAR_ERROR)
        arrived_only_one_position = (abs(x_goal_carrelage - x_carrelage) < LINEAR_ERROR or abs(y_goal_carrelage - y_carrelage) < LINEAR_ERROR)

        #tentative de debut de correction de quand il part dans la mauvaise direction
        #il regarde si son erreur en x et en y augmentent, si c'est le cas, c'est qu'il n'atteindra
        #jamais la destination

        new_error_on_x = abs(x_goal_carrelage - x_carrelage)
        new_error_on_y = abs(y_goal_carrelage - y_carrelage)

        if new_error_on_x < current_error_on_x or new_error_on_y < current_error_on_y:
            current_error_on_x = new_error_on_x
            current_error_on_y = new_error_on_y

        if new_error_on_x > current_error_on_x and new_error_on_y > current_error_on_y:
            print "je pars dans les choux" #faire une boucle de retour ici 
            #pour qu'il retente d'atteindre sa position en esperant qu'il s en 
            #soit approche 





        if arrived_position:
            arrived_theta = False

    elif arrived_position and not arrived_theta:

            angular_velocity = vitesse_rotation(theta_carrelage=theta_carrelage,
                                                theta_obj=theta_obj,
                                                vitesse_angulaire=ANGULAR_VELOCITY)

            rotate_franklin(theta_carrelage=theta_carrelage,
                            theta_obj=theta_goal_carrelage,
                            angular_velocity=angular_velocity,
                            angular_error=ANGULAR_ERROR)

            arrived_theta = abs(theta_obj - theta_carrelage) < ANGULAR_ERROR

    else:
        print ("Franklin au rapport, destination atteinte mon colonel!")

    print 40 * '#'
    print 'theta carrelage: ', normalise_angle(theta_carrelage)
    print 'theta obj: ', normalise_angle(theta_obj)
    print 'arrived theta/position: ', arrived_theta, arrived_position
    print "x_carrelage: ", x_carrelage 
    print "y_carrelage: ", y_carrelage 


    pub_cmd.publish(msg)


if __name__ == '__main__':

    print('***********************************')

    x_goal_carrelage = float(sys.argv[1])   #la position but dans le ref carrelage absolu_
    y_goal_carrelage = float(sys.argv[2])
    theta_goal_carrelage = float(sys.argv[3])
    x_carrelage_init = float(sys.argv[4])
    y_carrelage_init = float(sys.argv[5])

    current_error_on_x = abs(x_carrelage_init - x_goal_carrelage)
    current_error_on_y = abs(y_carrelage_init - y_goal_carrelage)

    theta_goal_carrelage = normalise_angle(theta_goal_carrelage)

    x_odom_init, y_odom_init, theta_odom_init = None, None, None

    arrived_position = False
    arrived_theta = False

    angular_velocity = ANGULAR_VELOCITY
    linear_error = LINEAR_ERROR
    # rajouter une variable distance to go globale pour ne plus se deplacer avec des conditions if

    rospy.init_node('nav_to_goal', anonymous=True)
    msg = geometry_msgs.msg.Twist()

    #pub_cmd = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    pub_cmd = rospy.Publisher('/cmd_vel_mux/input/teleop', geometry_msgs.msg.Twist, queue_size=10)
    #mettre cmd_vel et remaper dans le fichier launch
    #rospy.Subscriber("/turtle1/pose", Pose, callback)
    rospy.Subscriber("/odom", nav_msgs.msg.Odometry, odom_callback)

rospy.spin()    # keeps your node from exiting until the node has been shutdown.
