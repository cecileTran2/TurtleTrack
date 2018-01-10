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



class PanTiltController:

    def __init__(self):
        self.camera_info_sub = rospy.Subscriber('/state', 
            Axis, self.camera_callback, queue_size = 1)
        self.coordinates_turtle = rospy.Subscriber('/coordinates', 
            geometry_msgs.msg.Point, self.coordinates_callback, queue_size = 1)

    def camera_callback(self, msg):
        print('CAMERA : ', msg)

    def coordinates_callback(self, msg):
        print('COORDS : ', msg)


if __name__ == '__main__':
    
    rospy.init_node('pan_tilt', anonymous=True)
    pantilt_controller = PanTiltController()

    rospy.spin()
    