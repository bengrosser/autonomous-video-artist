#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : test_hsv.py
 #Creation Date : 31-05-2017
 #Last Modified : Wed May 31 10:57:56 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.
import numpy as np
import cv2

img = cv2.imread("result.jpg")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
row, collumn, wutever= np.shape(img)
for i in range(row):
    for j in range(collumn):
        hsv[i][j][2] = 125 
cv2.imwrite( "hsv_result.jpg", hsv)
