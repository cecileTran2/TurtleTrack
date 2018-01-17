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
        self.i = 0

        # Subscribers
        self.camera_info_sub = rospy.Subscriber('/state', 
            Axis, self.camera_callback, queue_size = 1)
        self.coordinates_turtle_sub = rospy.Subscriber('/coordinates', 
            geometry_msgs.msg.Point, self.coordinates_callback, queue_size = 1)

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd', Axis, queue_size = 1)



    def camera_callback(self, msg):
        self.camera_info = msg
        print('-'*30)
        print('CAMERA : ', msg)
        self.convert()
        print('CAMERA : ', msg)

        time.sleep(0.2)
        self.cmd_pub.publish(self.camera_info)

    def coordinates_callback(self, msg):
        self.coordinates = msg
        

    def convert(self):

        print('#####################"CONVERTING')
        print('COORDS : ', self.coordinates)
        print('camera_ifo : ', self.camera_info)
        print('#####################"CONVERTING')

        zoom = self.camera_info.zoom
        tilt = self.camera_info.tilt
        pan = self.camera_info.pan
        brightness = self.camera_info.brightness
        iris = IRIS_VALUE
        autofocus = self.camera_info.autofocus

        if self.coordinates.x or self.coordinates.y:
            print('>>>>>>> JE DETECTE UN TRUC LOL')

            PI = math.pi
            #xc, yc = 263, 352
            xc, yc = 352, 263

            theta = 4.189301e+001-6.436043e-003*zoom+2.404497e-007*zoom*zoom

            focale = xc/math.tan((PI*theta/180.0)/2)

            #x, y = 20.0, 20.0
            #x = xc - x
            #y = yc - y
            x = self.coordinates.x - xc
            y = self.coordinates.y - yc
            #x = xc - self.coordinates.x
            #y = yc - self.coordinates.y
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

        else:
            print('///////JE VOIS RIEN MDR XDDDDDD')

            # self.camera_info.pan = pan 
            # self.camera_info.tilt = tilt
            # self.camera_info.zoom = zoom
            # self.camera_info.iris = IRIS_VALUE
 
            
if __name__ == '__main__':

    
    rospy.init_node('pan_tilt', anonymous=True)
    pantilt_controller = PanTiltController()

    rospy.spin()
    