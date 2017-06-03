#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : demo_hrcorner.py
 #Creation Date : 02-06-2017
 #Last Modified : Sat Jun  3 09:59:05 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

import cv2
import numpy as np
import math
from matplotlib import pyplot as plt


filename = './src_picture/skyline.png'
img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

gray = np.float32(gray)
dst = cv2.cornerHarris(gray,2,3,0.04)

#result is dilated for marking the corners, not important
dst = cv2.dilate(dst,None)

# Threshold for an optimal value, it may vary depending on the image.
img[dst>0.1*dst.max()]=[0,0,255]

cv2.imshow('dst',img)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()
