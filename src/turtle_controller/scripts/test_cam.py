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


# global variables
[h,s,v] = [0,0,0]

h_values = [0, 179]
s_values = [0, 255]
v_values = [0, 255]


def erode_dilate_filter(im):
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    im2 = copy.copy(im)
    im2 = cv2.erode(im2, element, iterations = 1)
    im2 = cv2.dilate(im2, element, iterations = 1)
    return im2
            

def contourDetect(im):
    cnts = cv2.findContours(im.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    return cnts[1]

    
#Callback function for rqt_config
def callback_reconfigure(config, level):
    #get values from rqt_config
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

    # attribute values from rqt_config
    h_low, h_up = h_values
    s_low, s_up = s_values
    v_low, v_up = v_values
    
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    hsv_color = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

    #Detection of pink color variables
    h_low = 132
    h_up = 179
    s_low = 27
    s_up = 220
    v_low = 216
    v_up = 255
    
    lower_range = np.array([h_low, s_low, v_low], dtype=np.uint8)
    upper_range = np.array([h_up, s_up, v_up], dtype=np.uint8)
    
    mask = cv2.inRange(hsv_color, lower_range, upper_range)
    mask = erode_dilate_filter(mask)
    cnts = contourDetect(mask)

    all_bars = []
    for c in cnts:
        try:
	    # compute the center of the contour
	    M = cv2.moments(c)
	    cX = int(M["m10"] / M["m00"])
	    cY = int(M["m01"] / M["m00"])
 
	    # draw the contour and center of the shape on the image
	    cv2.drawContours(mask, [c], -1, (0, 255, 0), 2)
	    cv2.circle(mask, (cX, cY), 7, (255, 255, 255), -1)
	    cv2.putText(mask, "center", (cX - 20, cY - 20),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            all_bars.append((cX, cY))
        except Exception as e:
            print("An exception has occurred : ", e)

    print(all_bars)

    cv2.imshow('mask', mask)
    
    cv2.waitKey(1)


if __name__ == '__main__':
    
    rospy.init_node('test_cam', anonymous=True)

    #connect to server for rqt_config to find good parameters
    srv = Server(HSVParamsConfig, callback_reconfigure)
    
    #pub_cmd = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    
    rospy.Subscriber("/image_raw/compressed", sensor_msgs.msg.CompressedImage, callback)
    print('toto')
    rospy.spin()
    
    
    #while not rospy.is_shutdown():
     #   cv2.waitKey(10)
