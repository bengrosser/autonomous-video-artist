#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : demo_hrcorner.py
 #Creation Date : 02-06-2017
 #Last Modified : Tue Jun 13 14:04:44 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

'''
This is a demo that shows and visualizes the procedure of how computer determines features(corners) 
in mordern computer vision libraries
'''

import cv2
import numpy as np
import math
from matplotlib import pyplot as plt


def harris_visual(img): #TODO: Not quite sure about the size of the kernal
    #TODO: Need to check the output to determine
    def gaussian_blur(img):
        return cv2.GaussianBlur(img, (5,5), 0)


    def sobel_filter(img):
        sobelx = cv2.Sobel(img.copy(), cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(img.copy(), cv2.CV_64F, 0, 1, ksize=5)
        return (sobelx, sobely) 

    #TODO: need to find out which one is actually faster
    #TODO: The np library call or point to point calculation?
    #TODO: Also we need to figure out a good k value
    #TODO: For the visual purpose 
    def harris_measure(Sx2, Sy2, Sxy):
        # harris_measure_matrix = np.array([Sx2,Sxy],[Sxy,Sy2])
        # trace_square_matrix = ( np.trace(harris_measure_matrix) )^2
        # det_matrix = np.linalg.det(harris_measure_matrix)
        trace_matrix =  Sx2 + Sy2
        trace_square_matrix = trace_matrix * trace_matrix
        det_matrix = Sx2 * Sy2 - Sxy * Sxy
        harris_matrix = det_matrix - 0.01 * trace_square_matrix 
        #Harris_matrix now bears the "score" of how possible it can be the corner
        #TODO: Figure out whether I should dilate it or not
        harris_matrix = cv2.dilate(harris_matrix, None)
        return harris_matrix


    print "start process the frame"
    sobelx, sobely = sobel_filter(img.copy())
    Ix2 = sobelx * sobelx
    Iy2 = sobely * sobely
    Ixy = sobelx * sobely
    Sx2 = gaussian_blur(Ix2)
    Sy2 = gaussian_blur(Iy2)
    Sxy = gaussian_blur(Ixy)
    harris_result = harris_measure(Sx2, Sy2, Sxy)
    row, column = harris_result.shape
    harris_max = harris_result.max()
    scale_harris_min = abs(harris_result.min())
    counter = 1
    for i in range(row):
        for j in range(column):
            if harris_result[i][j] > 0.15*harris_max:
                element = harris_result[i][j]
                harris_result[i][j] = int(255 - (element/harris_max)*255)
                counter += 1 
            else:
                harris_result[i][j] = 255
    print counter
    return harris_result


#Static image testing
img = cv2.imread("./src_picture/exp.jpg")
grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
result = harris_visual(grey)
cv2.imwrite("result_justpoint.jpg", result)



# camera = cv2.VideoCapture("./src_video/test_clip.mp4")
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# frame_rate = 24 
# resolution = (1280, 720)
# out = cv2.VideoWriter("test_result.avi" ,fourcc, frame_rate, resolution)

# while True:
    # grabbed, frame = camera.read()
    # if grabbed:
        # img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # harris_result = harris_visual(img)
        # harris_result = np.uint8(harris_result)
        # harris_result = cv2.cvtColor(harris_result, cv2.COLOR_GRAY2RGB)
        # out.write(harris_result)
        # # print harris_result
        # if cv2.waitKey(1) & 0xFF == ord('q'):
            # break
    # else:
        # print("No video feed available")
        # break
# camera.release()
# out.release()
# cv2.destroyAllWindows()









