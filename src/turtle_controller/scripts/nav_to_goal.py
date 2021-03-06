#!/usr/bin/env python

import os
import sys
import time
import math
import random

import numpy as np

import rospy
import geometry_msgs.msg
import nav_msgs.msg

from sklearn.svm import NuSVR
from turtlesim.msg import Pose

# Errors
LINEAR_ERROR = 0.4
ANGULAR_ERROR = 0.05

# Velocities
ANGULAR_VELOCITY = 0.3
LINEAR_VELOCITY = 0.2

init_with_svm = True
coef_prop = 3.46    # coefficient de proportionnalite permettant de passer de deplacement

def train_svms():

    with open(os.path.join('/usr/users/promo2018/tran_cec/projet_track/src/turtle_controller/scripts/data/', 'data_pan_tilt.csv'), 'r') as f:
        lines = [l.rstrip('\n').split(',') for l in f][1:]

    pans = [float(p[0]) for p in lines]
    tilts = [float(t[1]) for t in lines]
    x_carr = [float(x[2]) for x in lines]
    y_carr = [float(y[3]) for y in lines]

    c = list(zip(pans, tilts, x_carr, y_carr))
    random.shuffle(c)
    pans, tilts, x_carr, y_carr = zip(*c)

    train_percentage = 0.8
    train_len = int(train_percentage * len(pans))

    pans_train, pans_test = pans[:train_len], pans[train_len:]
    tilts_train, tilts_test = tilts[:train_len], tilts[train_len:]
    x_carr_train, x_carr_test = x_carr[:train_len], x_carr[train_len:]
    y_carr_train, y_carr_test = y_carr[:train_len], y_carr[train_len:]

    # X
    # Train
    X_train = np.asarray([[i, j] for i, j in zip(pans_train, tilts_train)])
    y_train = np.asarray([i for i in x_carr_train])

    regr_x = NuSVR(nu=0.1, kernel='linear')
    regr_x.fit(X_train, y_train)

    print(regr_x.score(X_train, y_train))

    # Test
    X_test = np.asarray([[i, j] for i, j in zip(pans_test, tilts_test)])
    y_test = np.asarray([i for i in x_carr_test])

    print(regr_x.score(X_test, y_test))

    # Y
    # Train
    X_train = np.asarray([[i, j] for i, j in zip(pans_train, tilts_train)])
    y_train = np.asarray([i for i in y_carr_train])

    regr_y = NuSVR(nu=0.1, kernel='linear')
    regr_y.fit(X_train, y_train)

    print(regr_y.score(X_train, y_train))

    # Test
    X_test = np.asarray([[i, j] for i, j in zip(pans_test, tilts_test)])
    y_test = np.asarray([i for i in y_carr_test])

    print(regr_y.score(X_test, y_test))

    return regr_x, regr_y


def pantilt2xy(pan, tilt):

    global regr_x, regr_y

    pt = np.asarray([[pan, tilt]])
    x = regr_x.predict(pt)[0]
    y = regr_y.predict(pt)[0]

    return x, y


def quater2yaw(q):

    # il lui faut l'objet pose.pose.orientation
    yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    pitch = math.asin(-2.0*(q.x*q.z - q.w*q.y))
    roll = math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    return yaw, pitch, roll


def vitesse_rotation(theta_carrelage,
                     theta_obj,
                     vitesse_angulaire = 1.0,
                     linear_error = None):
    global msg

    return ANGULAR_VELOCITY


def normalise_angle(angle):

    # met l'angle dans [0, 2 pi]

    if angle >= 0.0 and angle <= 2*np.pi:
        return angle
    elif angle > 2*np.pi:
        output_angle = angle
        while output_angle > 2*np.pi:
            output_angle -= 2*np.pi
        return output_angle
    else:
        output_angle = angle
        while output_angle < 0:
            output_angle += 2 * np.pi
        return output_angle


def get_theta_obj(x_i, x_f, y_i, y_f, marge_erreur = 0.0):
    # rend theta_obj, le robot doit etre dans l'angle theta_obj pour se diriger en ligne droite vers sa cible

    if x_i == x_f:
        return math.pi/2 #sinon division par 0 si xi = xf

    tan_theta_obj = (y_i - y_f)/(x_i - x_f)

    if y_f > y_i and x_f > x_i:
        theta_obj = np.arctan(tan_theta_obj)
    elif y_f < y_i and x_f > x_i:
        theta_obj = (2*math.pi) + np.arctan(tan_theta_obj)
    elif y_f > y_i and x_f < x_i:
        theta_obj = math.pi + np.arctan(tan_theta_obj)
    else:
        theta_obj = math.pi + np.arctan(tan_theta_obj)

    return theta_obj


def get_angular_error(x_init, x_goal, y_init, y_goal):
    theta_obj = get_theta_obj(x_init, x_goal, y_init, y_goal)
    theta_obj_error = get_theta_obj(x_init, x_goal, y_init, y_goal)
    return abs(theta_obj_error-theta_obj)


def get_linear_error(x_i, x_f, y_i, y_f, angular_error):
    # donne l'erreur de position permise
    distance_to_go = math.sqrt((x_f-x_i)**2+(y_f-y_i)**2)
    return distance_to_go * math.tan(angular_error)/math.sqrt(2)


def rotate_franklin(theta_carrelage, angular_velocity, angular_error, theta_obj):
    global pub_cmd, msg

    # effectue une rotation tant qu'il n'a pas atteint l'angle voulu
    # TO DO: metre une boucle de retour s'il depasse l'angle par accident (avec une vitesse negative de module plus
    # faible pour qu'il fasse la rotation dans l'autre sens avec plus de precision)

    if abs(normalise_angle(theta_carrelage) - normalise_angle(theta_obj)) > angular_error:
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


def svm_position_callback(data):
    global x_svm, y_svm
    x_svm = data.x
    y_svm = data.y
    print("x, y :", x_svm, y_svm)


def odom_callback(data):

    global pub_cmd, msg, arrived_position, angular_velocity, arrived_theta, linear_error,\
    x_odom_init, y_odom_init, theta_odom_init, previous_error_on_x, previous_error_on_y,\
    X_poses, Y_poses, THETA_poses, index_pose, x_goal_carrelage, y_goal_carrelage, theta_goal_carrelage,\
    x_carrelage_init, y_carrelage_init, x_svm, y_svm, already_corrected, count, LINEAR_ERROR

    x_carrelage_init = x_svm
    y_carrelage_init = y_svm

    # on recupere les positions courrantes du robot:
    x_odom = data.pose.pose.position.x
    y_odom = data.pose.pose.position.y

    yaw, pitch, roll = quater2yaw(data.pose.pose.orientation)
    theta_odom = normalise_angle(roll)

    # on recupere le repere odom initial
    if x_odom_init is None:
        x_odom_init, y_odom_init, theta_odom_init = x_odom, y_odom, theta_odom
        print('theta_odom_init : ', theta_odom_init)

    # on a les coordonnes du but dans le repere du carrelage, on veut les mettre dans le repere odom init:
    x_goal_odom_init = x_goal_carrelage/coef_prop - x_odom_init
    y_goal_odom_init = y_goal_carrelage/coef_prop - y_odom_init
    theta_goal_odom_init = theta_goal_carrelage - theta_odom_init

    # on recupere la position courante du robot dans le repere carrelage:
    x_current_odom_init = x_odom - x_odom_init # on commence par les exprimer dans le repere odom_init
    y_current_odom_init = y_odom - y_odom_init
    theta_current_odom_init = theta_odom - theta_odom_init


    if x_carrelage_init is not None:
        x_carrelage = abs(x_current_odom_init*coef_prop + x_carrelage_init)
        y_carrelage = abs(y_current_odom_init*coef_prop + y_carrelage_init)

    theta_carrelage_init= 0.0   # on suppose ici que la tortue est placee initialement avec un angle theta = 0.0 :D
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

        linear_error = get_linear_error(x_i=x_carrelage, y_i=y_carrelage, x_f=x_goal_carrelage, y_f=y_goal_carrelage,
                                        angular_error=ANGULAR_ERROR)

    elif arrived_theta and not arrived_position:

        walk_franklin(x_i=x_carrelage, y_i=y_carrelage, x_f=x_goal_carrelage, y_f=y_goal_carrelage,
                      linear_velocity=LINEAR_VELOCITY,
                      linear_error=linear_error)

        distance_to_go = math.sqrt((x_carrelage-x_goal_carrelage)**2+(y_carrelage-y_goal_carrelage)**2)

        arrived_position_carrelage = (abs(x_goal_carrelage - x_carrelage) < LINEAR_ERROR and abs(y_goal_carrelage - y_carrelage) < LINEAR_ERROR)
        arrived_position_svm = (abs(x_goal_carrelage - x_svm) < LINEAR_ERROR and abs(y_goal_carrelage - y_svm) < LINEAR_ERROR)

        arrived_position = arrived_position_svm or arrived_position_carrelage


        arrived_only_one_position = (abs(x_goal_carrelage - x_carrelage) < LINEAR_ERROR or abs(y_goal_carrelage - y_carrelage) < LINEAR_ERROR)

        # tentative de debut de correction de quand il part dans la mauvaise direction
        # il regarde si son erreur en x et en y augmentent, si c'est le cas, c'est qu'il n'atteindra
        # jamais la destination

        new_error_on_x = abs(x_goal_carrelage - x_carrelage)
        new_error_on_y = abs(y_goal_carrelage - y_carrelage)

        if new_error_on_x > previous_error_on_x + 0.1 and new_error_on_y > previous_error_on_y + 0.1:
            print("je pars dans les choux") #faire une boucle de retour ici

            # cette solution marche pas parce qu il se remet dans les choux tout seul
            if not already_corrected:

                print(60* "*")

                arrived_position = False
                arrived_theta = False
                x_carrelage = x_svm
                y_carrelage = y_svm
                x_carrelage_init = x_carrelage
                y_carrelage_init = y_carrelage

                theta_carrelage_init = theta_carrelage #????

                print(x_carrelage_init, y_carrelage_init, theta_carrelage_init)
                time.sleep(8)

            else:
                LINEAR_ERROR += 0.3

                arrived_position = False
                arrived_theta = False
                x_carrelage = x_svm
                y_carrelage = y_svm
                x_carrelage_init = x_carrelage
                y_carrelage_init = y_carrelage

                theta_carrelage_init = theta_carrelage

                print(x_carrelage_init, y_carrelage_init, theta_carrelage_init)
                time.sleep(8)

                x_carrelage_init = x_carrelage
                y_carrelage_init = y_carrelage

            already_corrected = True

            # pour qu'il retente d'atteindre sa position en esperant qu'il s en
            # soit approche

        previous_error_on_x = new_error_on_x
        previous_error_on_y = new_error_on_y

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
        print("Franklin au rapport, destination atteinte mon colonel!")
        already_corrected = False

        index_pose += 1
        if index_pose < len(X_poses):
            arrived_position = False
            arrived_theta = False
            x_goal_carrelage = X_poses[index_pose]
            y_goal_carrelage = Y_poses[index_pose]
            x_carrelage_init = X_poses[index_pose-1]
            y_carrelage_init = Y_poses[index_pose-1]

            while True:
                print('while true')
                try:
                    with open(os.path.join('src/turtle_controller/scripts/', 'pt.txt'), 'r') as f:
                        lines = [l.rstrip('\n') for l in f]
                        if len(lines):
                            l = lines[0].split('\t')
                            x_carrelage = l[0]
                            y_carrelage = l[1]
                            break
                except Exception as e:
                    pass

            theta_goal_carrelage = normalise_angle(THETA_poses[index_pose])


    print(40 * '#')
    print(count); count += 1
    print(40 * '#')
    print('theta carrelage: ', normalise_angle(theta_carrelage))
    print('theta obj: ', normalise_angle(theta_obj))
    print('arrived theta/position: ', arrived_theta, arrived_position))
    if arrived_theta:
        print 60 * '_'
    print("x_carrelage: ", x_carrelage)
    print("y_carrelage: ", y_carrelage)
    print("x_goal_carrelage: ", x_goal_carrelage)
    print("y_goal_carrelage: ", y_goal_carrelage)

    pub_cmd.publish(msg)


if __name__ == '__main__':
    print('***********************************')

    x_goal_carrelage = float(sys.argv[1])   # la position but dans le ref carrelage absolu
    y_goal_carrelage = float(sys.argv[2])
    theta_goal_carrelage = float(sys.argv[3])

    try:
        file_poses = sys.argv[4]
        print('Poses')
        print(file_poses)
        file_poses = os.path.join('src/turtle_controller/scripts/', file_poses)
        with open(file_poses, 'r') as f:
            lines = [l.rstrip('\n').split('\t') for l in f]
            X_poses = [float(l[0]) for l in lines]
            Y_poses = [float(l[1]) for l in lines]
            THETA_poses = [float(l[2]) for l in lines]
        print('Poses fini.')

    except Exception as e:
        print(e)
        print('No poses.txt')
        X_poses = [x_goal_carrelage]
        Y_poses = [y_goal_carrelage]
        THETA_poses = [theta_goal_carrelage]

    print(X_poses)
    print(Y_poses)
    print(THETA_poses)

    regr_x, regr_y = train_svms()

    x_svm = None
    y_svm = None

    previous_error_on_y = 1000
    previous_error_on_x = 1000

    already_corrected = False

    count = 0

    index_pose = 0

    x_goal_carrelage = X_poses[index_pose]
    y_goal_carrelage = Y_poses[index_pose]
    theta_goal_carrelage = normalise_angle(THETA_poses[index_pose])

    x_odom_init, y_odom_init, theta_odom_init = None, None, None

    arrived_position = False
    arrived_theta = False

    angular_velocity = ANGULAR_VELOCITY
    linear_error = LINEAR_ERROR
    # rajouter une variable distance to go globale pour ne plus se deplacer avec des conditions if

    rospy.init_node('nav_to_goal', anonymous=True)
    msg = geometry_msgs.msg.Twist()

    pub_cmd = rospy.Publisher('/cmd_vel_mux/input/teleop', geometry_msgs.msg.Twist, queue_size=10)
    rospy.Subscriber("/odom", nav_msgs.msg.Odometry, odom_callback)
    rospy.Subscriber("/ArenaPosition", geometry_msgs.msg.Point, svm_position_callback)

rospy.spin()    # keeps your node from exiting until the node has been shutdown.
