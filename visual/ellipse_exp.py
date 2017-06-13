#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : ellipse_exp.py
 #Creation Date : 12-06-2017
 #Last Modified : Tue Jun 13 12:23:09 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

import numpy as np
import cv2


img = np.zeros((512,512,3), np.uint8)
img = cv2.ellipse(img, (256,256), (5,6), -45, 0, 360, (255, 255, 255), 1) 
cv2.imshow("result", img)
cv2.waitKey(0)
img = cv2.ellipse(img, (256,256), (100,50), 45, 0, 360, (255, 255, 255), 1) 
cv2.imshow("result", img)
cv2.waitKey(0)
