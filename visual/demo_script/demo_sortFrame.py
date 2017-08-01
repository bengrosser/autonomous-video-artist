'''
This is an experiment trying to sort each frame by certain criterial, 
like intensity of the pixel, H,S,V values
'''

import cv2
import numpy as np
import time


def get_sorted(img, which_method):
    gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
    color_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
    x,y = gray.shape
    total_num = x*y
    binary_img = None
    if which_method == 0:
        flattened = gray.copy()
        flattened.shape = (total_num, )
        sorted_result = np.sort(flattened)
        sorted_result.shape = (x,y)
        binary_img = cv2.cvtColor(sorted_result, cv2.COLOR_GRAY2RGB) 
    elif which_method == 1:
        flattened = color_hsv.copy()
        flattened.shape = (x*y, 3)
        result = flattened[flattened[:,0].argsort()]
        result.shape = (x,y,3)
        binary_img = cv2.cvtColor(result, cv2.COLOR_HSV2RGB) 
    elif which_method == 2:
        flattened = color_hsv.copy()
        flattened.shape = (x*y, 3)
        result = flattened[flattened[:,1].argsort()]
        result.shape = (x,y,3)
        binary_img = cv2.cvtColor(result, cv2.COLOR_HSV2RGB) 
    elif which_method == 3:
        flattened = color_hsv.copy()
        flattened.shape = (x*y, 3)
        result = flattened[flattened[:,2].argsort()]
        result.shape = (x,y,3)
        binary_img = cv2.cvtColor(result, cv2.COLOR_HSV2RGB) 
    print "finished one frame"
    return binary_img 
        


# Statis image testing    
# img = cv2.imread("./src_picture/exp.jpg") 
# result = get_sorted(img,1)
# cv2.imshow("result_justpoint", result)
# cv2.waitKey(0)

camera = cv2.VideoCapture("./src_video/matrix-woman-red.mp4")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
frame_rate = 24 
resolution = (1920, 800)
out = cv2.VideoWriter("sort_with_I.avi" ,fourcc, frame_rate, resolution)
start_time = time.time()
while True:
    grabbed, frame = camera.read()
    if grabbed:
        result = get_sorted(frame, 0)
               # print harris_result
        # cv2.imshow("result", result)
        out.write(result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("No video feed available")
        break
end_time = time.time()
print end_time - start_time 
camera.release()
out.release()
cv2.destroyAllWindows()


    

