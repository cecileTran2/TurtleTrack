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


LINEAR_ERROR = 0.03
ANGULAR_ERROR = 0.01


ANGULAR_VELOCITY = 0.5
LINEAR_VELOCITY = 1


def equal_signe(x,y):
    if x>=0 and y>=0 or x<=0 and y<=0:
        return True
    else:
        return False


def vitesse_rotation(angle_initial, angle_final, vitesse_angulaire = 1.0, linear_error = None):
    global msg

    if linear_error is None:
        V = vitesse_angulaire
    else:
        V = 10 * linear_error

    if angle_final<(angle_initial+math.pi) and angle_final>angle_initial:
        return V
    else:
        return -V


def normalise_angle(angle):

    # met l'angle dans [0, 2 pi]

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


def rotate_franklin(x_i, y_i, x_f, y_f,
                    angle_initial,
                    angular_velocity,
                    angular_error,
                    angle_final=None):
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

    if angle_final is None:
        angle_final = get_theta_obj(x_i, x_f, y_i, y_f)

    if abs(angle_final - angle_initial) > angular_error:
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


def callback(data):

    global pub_cmd, msg, arrived_position, angular_velocity, arrived_theta, linear_error

    x_init = data.x
    y_init = data.y
    theta_init = normalise_angle(data.theta)

    # debug print:
    print("position_x", x_init)
    print("position_y", y_init)
    print("position_theta", theta_init)

    print("arrived_theta", arrived_theta)
    print("arrived_position", arrived_position)

    msg = geometry_msgs.msg.Twist()

    if not arrived_position:

        # si la tortue n'a pas encore atteint sa position spatiale, sa position angulaire voulue est l'orientation
        # vers la position finale

        theta_obj = get_theta_obj(x_i=x_init, y_i=y_init, x_f=x_goal, y_f=y_goal)

    else:

        # sinon c'est l'angle final voulu

        theta_obj = theta_goal

    if not arrived_theta and not arrived_position:

        angular_velocity = vitesse_rotation(angle_initial=theta_init,
                                            angle_final=theta_obj,
                                            vitesse_angulaire=ANGULAR_VELOCITY)

        rotate_franklin(x_i=x_init, y_i=y_init, x_f=x_goal, y_f=y_goal,
                        angle_initial=theta_init,
                        angle_final=theta_obj,
                        angular_velocity=angular_velocity,
                        angular_error=ANGULAR_ERROR)

        arrived_theta = abs(theta_obj - theta_init) < ANGULAR_ERROR
        real_angular_error = abs(theta_obj - theta_init)
        linear_error = get_linear_error(x_i=x_init, y_i=y_init, x_f=x_goal, y_f=y_goal,
                                        angular_error=ANGULAR_ERROR)

    elif arrived_theta and not arrived_position:

        walk_franklin(x_i=x_init, y_i=y_init, x_f=x_goal, y_f=y_goal,
                      linear_velocity=LINEAR_VELOCITY,
                      linear_error=linear_error)

        arrived_position = (abs(x_goal - x_init) < linear_error and abs(y_goal - y_init) < linear_error)

        if arrived_position:
            arrived_theta = False

    elif arrived_position and not arrived_theta:

            angular_velocity = vitesse_rotation(angle_initial=theta_init,
                                                angle_final=theta_obj,
                                                vitesse_angulaire=ANGULAR_VELOCITY)

            rotate_franklin(x_i=x_init, y_i=y_init, x_f=x_goal, y_f=y_goal,
                            angle_initial=theta_init,
                            angle_final=theta_goal,
                            angular_velocity=angular_velocity,
                            angular_error=ANGULAR_ERROR)

            arrived_theta = abs(theta_obj - theta_init) < ANGULAR_ERROR

    else:
        print ("Franklin au rapport, destination atteinte mon colonel!")

    pub_cmd.publish(msg)


if __name__ == '__main__':

    x_goal = float(sys.argv[1])
    y_goal = float(sys.argv[2])
    theta_goal = float(sys.argv[3])

    theta_goal = normalise_angle(theta_goal)

    arrived_position = False
    arrived_theta = False
    angular_velocity = ANGULAR_VELOCITY
    linear_error = LINEAR_ERROR
    # rajouter une variable distance to go globale pour ne plus se deplacer avec des conditions if

    rospy.init_node('nav_to_goal', anonymous=True)
    msg = geometry_msgs.msg.Twist()

    pub_cmd = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, callback)

rospy.spin()    # keeps your node from exiting until the node has been shutdown.
