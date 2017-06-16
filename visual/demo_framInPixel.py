 
#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : demo_hrcorner.py
 #Creation Date : 02-06-2017
 #Last Modified : Fri Jun 16 12:22:59 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

'''
This is simple an experiment of dealing with each frames simplest values
'''

import cv2
import numpy as np
from imutils.video import count_frames 


def get_sum(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    x,y = gray.shape
    total_num = x*y
    _, binary_img = cv2.threshold(gray, 127,255,cv2.THRESH_BINARY)
    number_of_white = int(np.sum(binary_img)/255)
    ratio = number_of_white/float(total_num)
    pixel_value = int(255*ratio)
    print "finished one frame"
    return pixel_value
    

def get_height(total_frame, base):
    for i in range(0,base):
        if i*base > total_frame:
            return i
    return 0


def get_size(total_frame):
    for i in range(total_frame):
        height = get_height(total_frame, i)
        if height == 0:
            continue
        if abs(height - i) <= 20:
            return (i, height)
    
 
# Statis image testing    
# img = cv2.imread("./src_picture/exp.jpg")
# result = get_sum(img)
# cv2.imwrite("result_justpoint.jpg", result)

camera = cv2.VideoCapture("./src_video/matrix-woman-red.mp4")
total_frame = count_frames("./src_video/matrix-woman-red.mp4")
width, height = get_size(total_frame)
result_size = width*height
print width, height, total_frame 
# result = np.zeros((height, width), np.uint8)
storage_array = [] 
while True:
    grabbed, frame = camera.read()
    if grabbed:
        result_pixel_value = get_sum(frame)
        storage_array.append(result_pixel_value)
               # print harris_result
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        while(len(storage_array) != result_size):
            storage_array.append(0)
        final_result = np.array(storage_array)
        final_result.shape = (height,width)
        final_result.astype(np.uint8)
        cv2.imwrite("temp_result.jpg", final_result)
        print("No video feed available")
        break
camera.release()
cv2.destroyAllWindows()


    

