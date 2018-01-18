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
import message_filters


def erode_dilate_filter(im):

def contourDetect(im):


#Callback function for rqt_config
def callback_reconfigure(config, level):
    #get values from rqt_config
    
def coordinates_callback(ros_data):
    

if __name__ == '__main__':

    POSITIONS_FILE = 'positions.txt'
    
    rospy.init_node('sampler', anonymous=True)

    #connect to server for rqt_config to find good parameters
    srv = Server(HSVParamsConfig, callback_reconfigure)
    
    #pub_cmd = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    
    rospy.Subscriber("/image_raw/compressed", sensor_msgs.msg.CompressedImage, coordinates_callback)
    #msg = geometry_msgs.msg.Pose2D()
    coordinates_pub = rospy.Publisher('/coordinates', geometry_msgs.msg.Point, queue_size=10)

    rospy.spin()
    
