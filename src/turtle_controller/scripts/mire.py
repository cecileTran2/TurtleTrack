# Generates a colored image to see which Hue to select for a given RGB color

import cv2
import numpy as np

width=180*2
height=50
image = np.zeros((height,width,3), np.uint8)
for i in range(height):
    for j in range(width):
        image[i,j,0] = j * 180/width
        image[i,j,1] = 255
        image[i,j,2] = 255

rgb_image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
cv2.imwrite('mire.png', rgb_image)

