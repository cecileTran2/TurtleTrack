#!/usr/bin/env python
import cv2
import numpy as np

img = cv2.imread('Couleur.png', 1)

#cv2.imshow('Couleur.png',img)


#hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#mask = cv2.inRange(hsv, lower_range, upper_range)
 
#cv2.imshow('mask',mask)
cv2.imshow('image', img)
 
k=cv2.waitKey(5) & 0xFF

#while(1):
 # k = cv2.waitKey(0)
 # if(k == 27):
  #  break
 
#cv2.destroyAllWindows()
