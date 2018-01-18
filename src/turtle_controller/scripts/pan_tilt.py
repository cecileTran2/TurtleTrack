#!/usr/bin/env python
# license removed for brevity

import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from turtlesim.msg import Pose
import sys
import time
import numpy as np
import math
import cv2
from dynamic_reconfigure.server import Server
from turtle_controller.cfg import HSVParamsConfig
import copy
from sensor_msgs.msg import Image, CameraInfo
from axis_camera.msg import Axis

IRIS_VALUE = 500

class PanTiltController:

    def __init__(self):

        self.camera_info = None
        self.coordinates = None
        self.mode = "search"
        self.sens = 1
        self.track_file = 'track_file.txt'

        # Subscribers
        self.camera_info_sub = rospy.Subscriber('/state', 
            Axis, self.camera_callback, queue_size = 1)
        self.coordinates_turtle_sub = rospy.Subscriber('/coordinates', 
            geometry_msgs.msg.Point, self.coordinates_callback, queue_size = 1)

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd', Axis, queue_size = 1)


    def camera_callback(self, msg):

        def save_new_track():
            with open(self.track_file, 'a') as f:
                s = '{},{},{},{}\n'.format(self.camera_info.pan, 
                    self.camera_info.tilt, self.coordinates.x, self.coordinates.y)
                f.write(s)

        self.camera_info = msg
        self.convert()

        time.sleep(0.2)
        save_new_track()
        self.cmd_pub.publish(self.camera_info)

    def coordinates_callback(self, msg):
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
            xc, yc = 352, 263

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
            self.camera_info.tilt = -30.0
            self.camera_info.pan += self.sens * 10
            print('Mode Search')

        if self.camera_info.pan >= 180.0 or self.camera_info.pan <= -180.0:
            self.sens = - self.sens
            self.camera_info.pan += self.sens * 10.0

        print(self.camera_info.pan)
 
            
if __name__ == '__main__':

    rospy.init_node('pan_tilt', anonymous=True)
    pantilt_controller = PanTiltController()

    rospy.spin()
