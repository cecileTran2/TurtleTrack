#!/usr/bin/env python

import time
import math
import random

import numpy as np

import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg

from turtlesim.msg import Pose
from axis_camera.msg import Axis

IRIS_VALUE = 500

liste_pan_tilt_search = [[-76,-27], [-65,-50], [-44,-50], [-34,-34], [-55, -34], [-55,-27]]

class PanTiltController:

    def __init__(self):

        self.camera_info = None
        self.coordinates = None
        self.mode = "search"
        self.sens = 1
        self.track_file = 'track_file.txt'
        self.x_robot_init, self.y_robot_init, self.theta_robot_init = None, None, None
        self.x_carrelage = None
        self.y_carrelage = None
        self.theta_carrelage = None

        with open(self.track_file, 'w') as f:
            pass

        # Subscribers
        self.camera_info_sub = rospy.Subscriber('/state',
            Axis, self.camera_callback, queue_size = 1)
        self.coordinates_turtle_sub = rospy.Subscriber('/coordinates',
            geometry_msgs.msg.Point, self.coordinates_callback, queue_size = 1)
        self.odom_sub = rospy.Subscriber("/odom", nav_msgs.msg.Odometry, self.odom_callback)

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd', Axis, queue_size = 1)


    def odom_callback(self, data):

        print("odom_callback")

        def quater2yaw(q):
            yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
            pitch = math.asin(-2.0*(q.x*q.z - q.w*q.y))
            roll = math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
            return yaw, pitch, roll

        def normalise_angle(angle):
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


        x_robot = data.pose.pose.position.x
        y_robot = data.pose.pose.position.y

        yaw, pitch, roll = quater2yaw(data.pose.pose.orientation)
        theta_robot = normalise_angle(roll)

        if self.x_robot_init is None:
            self.x_robot_init, self.y_robot_init, self.theta_robot_init = x_robot, y_robot, theta_robot

        self.x_carrelage = x_robot - self.x_robot_init
        self.y_carrelage = y_robot - self.y_robot_init
        self.theta_carrelage = normalise_angle(theta_robot - self.theta_robot_init)


    def save_new_track(self):
        print('{}\t{}\t{}\t{}'.format(self.camera_info.pan, self.camera_info.tilt, self.x_carrelage, self.y_carrelage))

        if (self.camera_info.pan is not None and self.camera_info.tilt is not None
            and self.x_carrelage is not None and self.y_carrelage is not None):

            print("I'm saving !")

            with open(self.track_file, 'a') as f:
                s = '{},{},{},{}\n'.format(self.camera_info.pan,
                    self.camera_info.tilt, self.x_carrelage, self.y_carrelage)
                f.write(s)


    def camera_callback(self, msg):

        print("camera_callback")

        self.camera_info = msg
        self.convert()

        time.sleep(0.2)
        self.save_new_track()
        self.cmd_pub.publish(self.camera_info)


    def coordinates_callback(self, msg):
        print("coordinates_callback")
        self.coordinates = msg


    def convert(self):

        zoom = self.camera_info.zoom
        tilt = self.camera_info.tilt
        pan = self.camera_info.pan
        brightness = self.camera_info.brightness
        iris = IRIS_VALUE
        autofocus = self.camera_info.autofocus

        if self.coordinates.x or self.coordinates.y:
            self.mode = 'track'
        else:
            self.mode = 'search'

        if self.mode == 'track':

            print('Mode Track')

            PI = math.pi
            xc, yc = 355, 275

            theta = 4.189301e+001-6.436043e-003*zoom+2.404497e-007*zoom*zoom

            focale = xc/math.tan((PI*theta/180.0)/2)

            x = self.coordinates.x - xc
            y = self.coordinates.y - yc
            z = focale

            norme = math.sqrt(x*x + y*y + z*z)
            x /= norme
            y /= norme
            z /= norme

            beta0 = -(PI*pan/180.0)
            alpha0 = -(PI*tilt/180.0)

            X = math.cos(beta0)*x + math.sin(alpha0) * math.sin(beta0)*y - math.cos(alpha0)* math.sin(beta0)*z
            Y = math.cos(alpha0)*y + math.sin(alpha0)*z
            Z = math.sin(beta0)*x - math.sin(alpha0)* math.cos(beta0)*y + math.cos(alpha0)* math.cos(beta0)*z

            alpha = math.atan2(Y, math.sqrt(X*X + Z*Z))
            beta = -math.atan2(X, Z);

            pan = -(180.0*beta/PI)
            tilt = -(180.0*alpha/PI)

            self.camera_info.pan = pan
            self.camera_info.tilt = tilt
            self.camera_info.zoom = zoom
            self.camera_info.iris = IRIS_VALUE

        elif self.mode == 'search':

            index_pan_tilt = random.randint(0, len(liste_pan_tilt_search)-1)
            pan_search,tilt_search = liste_pan_tilt_search[index_pan_tilt][0], liste_pan_tilt_search[index_pan_tilt][1]

            self.camera_info.tilt = tilt_search
            self.camera_info.pan = pan_search
            time.sleep(1)

            print('Mode Search')

        if self.camera_info.pan >= 180.0 or self.camera_info.pan <= -180.0:
            self.sens = - self.sens
            self.camera_info.pan += self.sens * 10.0

        print(self.camera_info.pan)


if __name__ == '__main__':

    rospy.init_node('pan_tilt', anonymous=True)
    pantilt_controller = PanTiltController()

    rospy.spin()
