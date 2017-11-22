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

# global variables
#cv2.namedWindow('image')
[h,s,v] = [0,0,0]

h_values = [0, 179]
s_values = [0, 255]
v_values = [0, 255]

def nothing(x):
    print(x)


def callback_reconfigure(config, level):
    #print(config, type(config))
    h_values[0] = config['h_low']
    h_values[1] = config['h_up']
    s_values[0] = config['s_low']
    s_values[1] = config['s_up']
    v_values[0] = config['v_low']
    v_values[1] = config['v_up']
    return config
    
def callback(ros_data):

    cv2.namedWindow('mask')
    #h_low = cv2.getTrackbarPos('H_low', 'mask')
    #s_low = cv2.getTrackbarPos('S_low', 'mask')
    #v_low = cv2.getTrackbarPos('V_low', 'mask')
    
    #cv2.createTrackbar('H_low','mask',h,360,nothing)
    #cv2.createTrackbar('S_low','mask',s,255,nothing)
    #cv2.createTrackbar('V_low','mask',v,255,nothing)

    #h_up = cv2.getTrackbarPos('H_up', 'mask')
    #s_up = cv2.getTrackbarPos('S_up', 'mask')
    #v_up = cv2.getTrackbarPos('V_up', 'mask')
    
    #cv2.createTrackbar('H_up','mask',h,360,nothing)
    #cv2.createTrackbar('S_up','mask',s,255,nothing)
    #cv2.createTrackbar('V_up','mask',v,255,nothing)
    #cv2.waitKey(1000) # GUI needs time to launch...

    h_low, h_up = h_values
    s_low, s_up = s_values
    v_low, v_up = v_values
    
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    hsv_color = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
    
    lower_range = np.array([h_low, s_low, v_low], dtype=np.uint8)
    upper_range = np.array([h_up, s_up, v_up], dtype=np.uint8)

    mask = cv2.inRange(hsv_color, lower_range, upper_range)
        
    #cv2.imshow('image',image_np)
    cv2.imshow('mask',mask)
    cv2.waitKey(1)
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    
    rospy.init_node('test_cam', anonymous=True)

    srv = Server(HSVParamsConfig, callback_reconfigure)
    #pub_cmd = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    rospy.Subscriber("/image_raw/compressed", sensor_msgs.msg.CompressedImage, callback)
    
    rospy.spin()
    print('toto')
    
    #while not rospy.is_shutdown():
     #   cv2.waitKey(10)
